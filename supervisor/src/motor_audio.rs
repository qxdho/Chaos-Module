//! 振动马达音频通路（独立实现，不包含第三方播放器代码）。
//!
//! 目标设备：小米手环 10 Pro / 3.101.043。
//!
//! 原理：固件的 LRA 驱动把马达芯片当作可编程波形输出设备。这里通过
//! `/dev/lra0` 取得驱动对象，按芯片类型写 RTP/FIFO 寄存器，把 8-bit
//! 单声道 PCM 以 12/24 kHz 送进 FIFO。它不是系统 `vibrate()` 接口，
//! 也不改固件分区。
//!
//! 安全约束：
//! - 只在用户点击“播放/测试音”后写寄存器；
//! - 打开后先校验芯片编号、基址、ops 指针；
//! - 保存并恢复被改写的寄存器；
//! - 音量上限 200/255，测试音幅度 48/127；
//! - 任何一步失败立即 stop + restore。
//!
//! 这是按固件驱动行为重写的独立实现，没有引用 WavAudio 的源码或素材。

use core::mem::transmute;
use core::ptr;

use crate::fw_api;

// ===== 固件入口（与 fw_api 同一套 3.101.043 地址） =====
const FN_RESOLVE: u32 = 0x0C1B_9701; // resolve(fd, &base, &chip)

const CHIP1_READ: u32 = 0x0C40_0BAD; // read8(base, reg, &v)
const CHIP1_WRITE8: u32 = 0x0C40_0D99; // write8(base, reg, v)
const CHIP1_WRITE_BLOCK: u32 = 0x0C40_0C9D; // write(base, reg, len, buf)

const CHIP2_WRITE_BLOCK: u32 = 0x0C3F_D31D; // write(base, reg, len, buf)
const CHIP2_READ_BLOCK: u32 = 0x0C3F_D3C9; // read(base, reg, len, buf)

const REG_FIFO_CTRL: u32 = 0x13;
const REG_FIFO_STAT: u32 = 0xAF;
const REG_FIFO_DATA: u32 = 0x30;

const CHIP1_OPS_OFF: u32 = 0x5AC;
const CHIP2_OPS_OFF: u32 = 0x574;
const CHIP1_RESET_OFF: u32 = 0x80;
const CHIP2_RESET_OFF: u32 = 0x74;
const OPS_STOP: u32 = 0x0C;
const OPS_START: u32 = 0x14;
const OPS_MODE: u32 = 0x28;

const CHIP1_REGS: [u8; 9] = [0x11, 0x21, 0x78, 0x31, 0x58, 0x59, 0x5F, 0xAB, 0x13];
const CHIP2_REGS: [u8; 5] = [0x06, 0x07, 0x34, 0x79, 0x03];

const LRA_PATH: &[u8] = b"/dev/lra0\0";

const OK: i32 = 0;
const E_NO_DEV: i32 = -100;
const E_OPEN: i32 = -101;
const E_RESOLVE: i32 = -102;
const E_CHIP: i32 = -103;
const E_RATE: i32 = -104;
const E_READ: i32 = -105;
const E_WRITE: i32 = -106;
const E_OPS: i32 = -107;
const E_STATE: i32 = -108;
const E_STALL: i32 = -110;

type FnResolve = unsafe extern "C" fn(i32, *mut u32, *mut u32) -> i32;
type FnRead8 = unsafe extern "C" fn(u32, u32, *mut u8) -> i32;
type FnWrite8 = unsafe extern "C" fn(u32, u32, u32) -> i32;
type FnReadBlock = unsafe extern "C" fn(u32, u32, u32, *mut u8) -> i32;
type FnWriteBlock = unsafe extern "C" fn(u32, u32, u32, *const u8) -> i32;
type FnOps0 = unsafe extern "C" fn(u32) -> i32;
type FnOps1 = unsafe extern "C" fn(u32, u32) -> i32;

struct Motor {
    fd: i32,
    chip: u32,
    base: u32,
    ops: u32,
    ready: bool,
    running: bool,
    fifo_ready: bool,
    count: usize,
    regs: [u8; 9],
    vals: [u8; 9],
    cfg: [u8; 3],
    id_snap: u8,
    stat_snap: u8,
    cfg_ok: bool,
}

impl Motor {
    const fn new() -> Self {
        Self {
            fd: -1,
            chip: 0,
            base: 0,
            ops: 0,
            ready: false,
            running: false,
            fifo_ready: false,
            count: 0,
            regs: [0; 9],
            vals: [0; 9],
            cfg: [0; 3],
            id_snap: 0,
            stat_snap: 0,
            cfg_ok: false,
        }
    }
}

static mut MOTOR: Motor = Motor::new();
static mut PROBED_CHIP: i32 = 0;

fn rd32_at(address: u32) -> u32 {
    unsafe { ptr::read_volatile(address as *const u32) }
}

unsafe fn op_at(ops: u32, off: u32) -> u32 {
    rd32_at(ops.wrapping_add(off))
}

fn chip_from_fp(fp: u32) -> u32 {
    match fp {
        0x0C40_12F5 => 1,
        0x0C3F_DDE5 => 2,
        _ => 0,
    }
}

unsafe fn id_at(base: u32, chip: u32) -> u8 {
    ptr::read_volatile(base.wrapping_add(if chip == 1 { 0x470 } else { 0x480 }) as *const u8)
}

// ===== 芯片寄存器访问 =====

unsafe fn read8(reg: u32) -> i32 {
    if MOTOR.fd < 0 || MOTOR.base == 0 { return E_STATE; }
    match MOTOR.chip {
        1 => {
            let f: FnRead8 = transmute(CHIP1_READ as usize);
            let mut v = 0u8;
            let rc = f(MOTOR.base, reg as u32, &mut v as *mut u8);
            if rc < 0 { rc } else { v as i32 }
        }
        2 => {
            let f: FnReadBlock = transmute(CHIP2_READ_BLOCK as usize);
            let mut v = 0u8;
            let rc = f(MOTOR.base, reg as u32, 1, &mut v as *mut u8);
            if rc < 0 { rc } else { v as i32 }
        }
        _ => E_CHIP,
    }
}

unsafe fn write8(reg: u32, value: u8) -> i32 {
    if MOTOR.fd < 0 || MOTOR.base == 0 { return E_STATE; }
    match MOTOR.chip {
        1 => {
            let f: FnWrite8 = transmute(CHIP1_WRITE8 as usize);
            f(MOTOR.base, reg as u32, value as u32)
        }
        2 => {
            let f: FnWriteBlock = transmute(CHIP2_WRITE_BLOCK as usize);
            f(MOTOR.base, reg as u32, 1, &value as *const u8)
        }
        _ => E_CHIP,
    }
}

unsafe fn write_block(reg: u32, data: *const u8, len: u32) -> i32 {
    if MOTOR.fd < 0 || MOTOR.base == 0 { return E_STATE; }
    match MOTOR.chip {
        1 => {
            let f: FnWriteBlock = transmute(CHIP1_WRITE_BLOCK as usize);
            f(MOTOR.base, reg as u32, len, data)
        }
        2 => {
            let f: FnWriteBlock = transmute(CHIP2_WRITE_BLOCK as usize);
            f(MOTOR.base, reg as u32, len, data)
        }
        _ => E_CHIP,
    }
}

unsafe fn bits(reg: u32, mask: u8, value: u8) -> i32 {
    let old = read8(reg);
    if old < 0 { return old; }
    let next = (old as u8 & !mask) | (value & mask);
    write8(reg, next)
}

// ===== 打开 / 解析 =====

unsafe fn motor_open() -> i32 {
    if MOTOR.fd >= 0 { return E_STATE; }
    let fd = fw_api::open(LRA_PATH.as_ptr(), 3, 0);
    if fd < 0 { return E_OPEN; }
    // 0x0c1b9701 返回的是驱动对象；芯片对象在 object+0x10 -> +0x18 -> +0x0。
    // 这段遍历与固件 LRA 驱动的对象布局对应，不能直接拿 object 当 base。
    let mut first = 0u32;
    let mut second = 0u32;
    let f: FnResolve = transmute(FN_RESOLVE as usize);
    let mut rc = f(fd, &mut first as *mut u32, &mut second as *mut u32);
    if rc < 0 {
        crate::log::kv("resolve 失败 ", (-rc) as u32);
        // 驱动可能刚被 vibratord 占用/复位，给两次短重试。
        for _ in 0..2 {
            let _ = fw_api::nsh_usleep(20_000);
            first = 0;
            second = 0;
            rc = f(fd, &mut first as *mut u32, &mut second as *mut u32);
            if rc >= 0 { break; }
        }
        if rc < 0 {
            fw_api::close(fd);
            return E_RESOLVE;
        }
    }
    // 少数实现把 object 放在返回值里；返回值为正且像指针时补到 first。
    if first == 0 && crate::plausible_ptr(rc as u32) {
        first = rc as u32;
    }
    // 这个 ioctl 只保证写对象指针；芯片号由对象首字的函数指纹决定。
    let object = if crate::plausible_ptr(first) {
        first
    } else if crate::plausible_ptr(second) {
        second
    } else {
        crate::log::kv("resolve A ", first);
        crate::log::kv("resolve B ", second);
        fw_api::close(fd);
        return E_RESOLVE;
    };
    let stage1 = rd32_at(object.wrapping_add(0x10));
    let stage2 = if crate::plausible_ptr(stage1) {
        rd32_at(stage1.wrapping_add(0x18))
    } else { 0 };
    let nested = if crate::plausible_ptr(stage2) { rd32_at(stage2) } else { 0 };
    let mut base = 0u32;
    for cand in [nested, stage2, stage1, object] {
        if cand != 0 && crate::plausible_ptr(cand) && chip_from_fp(rd32_at(cand)) != 0 {
            base = cand;
            break;
        }
    }
    if base == 0 {
        crate::log::kv("resolve A ", first);
        crate::log::kv("resolve B ", second);
        crate::log::kv("resolve fp ", rd32_at(object));
        crate::log::kv("resolve s1 ", stage1);
        crate::log::kv("resolve s2 ", stage2);
        crate::log::kv("resolve n ", nested);
        fw_api::close(fd);
        return E_CHIP;
    }
    let chip = chip_from_fp(rd32_at(base));
    let ops_off = if chip == 1 { CHIP1_OPS_OFF } else { CHIP2_OPS_OFF };
    let ops = rd32_at(base.wrapping_add(ops_off));
    if !crate::plausible_ptr(ops) {
        fw_api::close(fd);
        return E_OPS;
    }
    MOTOR.fd = fd;
    MOTOR.base = base;
    MOTOR.chip = chip;
    MOTOR.ops = ops;
    MOTOR.ready = false;
    MOTOR.running = false;
    MOTOR.fifo_ready = false;
    OK
}

unsafe fn motor_configure(rate: u32, gain: u32) -> i32 {
    if MOTOR.fd < 0 { return E_NO_DEV; }
    if MOTOR.ready { return E_STATE; }
    if rate != 12_000 && rate != 24_000 { return E_RATE; }
    // 已知芯片 2 只走 24k。
    if MOTOR.chip == 2 && rate != 24_000 { return E_RATE; }

    let reset_off = if MOTOR.chip == 1 { CHIP1_RESET_OFF } else { CHIP2_RESET_OFF };
    let reset: FnOps1 = transmute(op_at(MOTOR.ops, reset_off) as usize);
    let rc = reset(MOTOR.base, 0);
    if rc == 0 { motor_abort(); return E_OPS; }

    // 保存芯片 ID / 状态，退出前若被别的路径改过则放弃恢复。
    MOTOR.count = if MOTOR.chip == 1 { 9 } else { 5 };
    let table: &[u8] = if MOTOR.chip == 1 { &CHIP1_REGS } else { &CHIP2_REGS };
    MOTOR.regs = [0; 9];
    let mut vals = [0u8; 9];
    for i in 0..MOTOR.count {
        MOTOR.regs[i] = table[i];
        let v = read8(table[i] as u32);
        if v < 0 { motor_abort(); return E_READ; }
        vals[i] = v as u8;
    }
    MOTOR.vals = vals;
    MOTOR.id_snap = id_at(MOTOR.base, MOTOR.chip);
    MOTOR.stat_snap = ptr::read_volatile(MOTOR.base.wrapping_add(0x3FC) as *const u8);
    MOTOR.ready = true;
    if MOTOR.chip == 1 {
        let a = read8(0x5D9);
        let b = read8(0x5DA);
        if a >= 0 && b >= 0 {
            MOTOR.cfg[0] = a as u8;
            MOTOR.cfg[1] = b as u8;
            MOTOR.cfg_ok = true;
        }
    }

    // 芯片侧的“进入 RTP/使能输出”板级位。
    let rc = if MOTOR.chip == 1 {
        bits(0xAB, 1, 1)
    } else {
        bits(0x03, 2, 2)
    };
    if rc < 0 { motor_abort(); return E_WRITE; }

    let mode: FnOps1 = transmute(op_at(MOTOR.ops, OPS_MODE) as usize);
    let _ = mode(MOTOR.base, 2);

    if MOTOR.chip == 1 {
        let zero = read8(0x5D5);
        if zero != 0 { motor_abort(); return E_STATE; }
        let st = read8(0x11);
        if st < 0 { motor_abort(); return E_READ; }
        if (st as u8 & 0x30) == 0 { motor_abort(); return E_STATE; }
    } else {
        let rc = bits(0x34, 0x40, 0x40);
        if rc < 0 { motor_abort(); return E_WRITE; }
    }

    if motor_gain(gain) < 0 { motor_abort(); return E_WRITE; }

    OK
}

unsafe fn motor_start() -> i32 {
    if !MOTOR.ready { return E_STATE; }
    let start: FnOps1 = transmute(op_at(MOTOR.ops, OPS_START) as usize);
    let _ = start(MOTOR.base, 1);
    MOTOR.running = true;
    MOTOR.fifo_ready = false;
    OK
}

/// 芯片 start 后需要一点时间才接受 FIFO 使能；由 10 ms 泵重试，不阻塞 UI。
unsafe fn ensure_fifo() -> bool {
    if MOTOR.fifo_ready { return true; }
    let rc = bits(REG_FIFO_CTRL, 0x03, 0x03);
    if rc < 0 { return false; }
    let v = read8(REG_FIFO_CTRL);
    if v < 0 || (v as u8 & 0x03) != 0x03 { return false; }
    MOTOR.fifo_ready = true;
    crate::log::kv("马达FIFO就绪 ", MOTOR.chip);
    true
}

unsafe fn motor_gain(gain: u32) -> i32 {
    let g = gain.min(200);
    let v = ((g * 0x0A3D80) >> 19).min(255) as u8;
    let reg = if MOTOR.chip == 1 { 0x5F } else { 0x07 };
    write8(reg, v)
}

unsafe fn motor_feed(buf: *const u8, len: usize) -> i32 {
    if !MOTOR.ready || !MOTOR.running { return E_STATE; }
    if len == 0 || len > 256 { return -22; }
    let stat_reg = if MOTOR.chip == 1 { REG_FIFO_STAT } else { 0x0E };
    let busy_bit = if MOTOR.chip == 1 { 0x80 } else { 0x02 };
    let st = read8(stat_reg);
    if st < 0 { return st; }
    if (st as u8 & busy_bit) != 0 { return 0; }
    let data_reg = if MOTOR.chip == 1 { REG_FIFO_DATA } else { 0x2B };
    let rc = write_block(data_reg, buf, len as u32);
    if rc < 0 { return rc; }
    len as i32
}

unsafe fn motor_abort() {
    if MOTOR.fd < 0 { return; }
    if MOTOR.ready {
        // 驱动被重载/复位过时，旧值已无意义，宁可不写回。
        let id_now = id_at(MOTOR.base, MOTOR.chip);
        let stat_now = ptr::read_volatile(MOTOR.base.wrapping_add(0x3FC) as *const u8);
        if id_now != MOTOR.id_snap || stat_now != MOTOR.stat_snap {
            MOTOR.ready = false;
            MOTOR.running = false;
            fw_api::close(MOTOR.fd);
            MOTOR.fd = -1;
            MOTOR.base = 0;
            MOTOR.ops = 0;
            MOTOR.chip = 0;
            return;
        }
        // 尽力恢复；不再检查返回值。
        for i in 0..MOTOR.count {
            let _ = write8(MOTOR.regs[i] as u32, MOTOR.vals[i]);
        }
        if MOTOR.chip == 1 && MOTOR.cfg_ok {
            let _ = write8(0x5D9, MOTOR.cfg[0]);
            let _ = write8(0x5DA, MOTOR.cfg[1]);
        }
    }
    MOTOR.ready = false;
    MOTOR.running = false;
    MOTOR.fifo_ready = false;
    fw_api::close(MOTOR.fd);
    MOTOR.fd = -1;
    MOTOR.base = 0;
    MOTOR.ops = 0;
    MOTOR.chip = 0;
}

unsafe fn motor_stop() {
    if MOTOR.fd < 0 { return; }
    if MOTOR.ready {
        let stop: FnOps0 = transmute(op_at(MOTOR.ops, OPS_STOP) as usize);
        let _ = stop(MOTOR.base);
    }
    motor_abort();
}

// ===== 播放器 =====

const SRC_MAX: usize = 256;
const OUT_MAX: usize = 512;
const PATH_MAX: usize = 96;

struct Player {
    fd: i32,
    playing: bool,
    tone: bool,
    tone_left: u32,
    tone_phase: u32,
    rate: u32,
    volume: u32,
    err: i32,
    src: [u8; SRC_MAX],
    out: [u8; OUT_MAX],
    path: [u8; PATH_MAX],
    path_len: usize,
    pending_off: usize,
    pending_len: usize,
    stall: u32,
    logged_feed: bool,
}

impl Player {
    const fn new() -> Self {
        Self {
            fd: -1,
            playing: false,
            tone: false,
            tone_left: 0,
            tone_phase: 0,
            rate: 12_000,
            volume: 60,
            err: 0,
            src: [0; SRC_MAX],
            out: [0; OUT_MAX],
            path: [0; PATH_MAX],
            path_len: 0,
            pending_off: 0,
            pending_len: 0,
            stall: 0,
            logged_feed: false,
        }
    }
}

static mut PLAYER: Player = Player::new();

/// 0=空闲，1=播放中，2=播放结束，<0=错误。
pub fn status() -> i32 {
    unsafe {
        if PLAYER.playing { 1 } else if PLAYER.err != 0 { PLAYER.err } else { 0 }
    }
}

pub fn is_playing() -> bool {
    unsafe { PLAYER.playing }
}

pub fn volume() -> u32 {
    unsafe { PLAYER.volume }
}

pub fn set_volume(v: u32) {
    unsafe {
        PLAYER.volume = v.min(100);
        if PLAYER.playing {
            let _ = motor_gain(PLAYER.volume * 2);
        }
    }
}

pub fn last_error() -> i32 {
    unsafe { PLAYER.err }
}

/// 只读探测：打开 /dev/lra0、解析芯片，然后立即关闭。不写任何寄存器。
pub fn probe() -> i32 {
    unsafe {
        let rc = motor_open();
        if rc < 0 { PROBED_CHIP = rc; return rc; }
        let chip = MOTOR.chip as i32;
        // 只读一次芯片 ID 所在的对象字段，验证对象可访问；不写任何寄存器。
        let _id = ptr::read_volatile(MOTOR.base.wrapping_add(if chip == 1 { 0x470 } else { 0x480 }) as *const u8);
        motor_abort();
        PROBED_CHIP = chip;
        chip
    }
}

/// 播放 12 kHz、单声道、signed 8-bit 裸文件。
pub fn play_file(path: &[u8]) -> i32 {
    unsafe {
        if PROBED_CHIP <= 0 {
            let _ = probe();
        }
        if PROBED_CHIP <= 0 { return PROBED_CHIP; }
        crate::log::kv("马达探测 ", PROBED_CHIP.max(0) as u32);
        stop();
        if path.is_empty() || path.len() >= PATH_MAX { return -22; }
        let mut p = [0u8; PATH_MAX];
        p[..path.len()].copy_from_slice(path);
        let fd = fw_api::open(p.as_ptr(), 1, 0);
        if fd < 0 { PLAYER.err = E_OPEN; return E_OPEN; }
        PLAYER.fd = fd;
        PLAYER.path = p;
        PLAYER.path_len = path.len();
        PLAYER.tone = false;
        PLAYER.volume = PLAYER.volume.min(100);

        let rc = motor_open();
        if rc < 0 { stop(); PLAYER.err = rc; return rc; }
        let rate = if MOTOR.chip == 2 { 24_000 } else { 12_000 };
        let gain = PLAYER.volume * 2;
        let rc = motor_configure(rate, gain);
        if rc < 0 { stop(); PLAYER.err = rc; return rc; }
        let rc = motor_start();
        if rc < 0 { stop(); PLAYER.err = rc; return rc; }

        PLAYER.rate = rate;
        PLAYER.err = 0;
        PLAYER.playing = true;
        OK
    }
}

/// 内置测试音：220 Hz、2 秒、低幅度。用于验证 FIFO 通路。
pub fn play_tone() -> i32 {
    unsafe {
        if PROBED_CHIP <= 0 {
            let _ = probe();
        }
        if PROBED_CHIP <= 0 { return PROBED_CHIP; }
        stop();
        let rc = motor_open();
        if rc < 0 {
            crate::log::kv("马达测试打开失败 ", (-rc) as u32);
            PLAYER.err = rc; return rc;
        }
        let rate = if MOTOR.chip == 2 { 24_000 } else { 12_000 };
        let rc = motor_configure(rate, 80);
        if rc < 0 {
            crate::log::kv("马达测试配置失败 ", (-rc) as u32);
            stop(); PLAYER.err = rc; return rc;
        }
        let rc = motor_start();
        if rc < 0 {
            crate::log::kv("马达测试启动失败 ", (-rc) as u32);
            stop(); PLAYER.err = rc; return rc;
        }
        crate::log::kv("马达测试音 ", rate);
        PLAYER.rate = rate;
        PLAYER.tone = true;
        PLAYER.tone_left = rate * 2;
        PLAYER.tone_phase = 0;
        PLAYER.err = 0;
        PLAYER.playing = true;
        OK
    }
}

pub fn stop() {
    unsafe {
        PLAYER.playing = false;
        PLAYER.tone = false;
        PLAYER.pending_off = 0;
        PLAYER.pending_len = 0;
        PLAYER.stall = 0;
        PLAYER.logged_feed = false;
        if PLAYER.fd >= 0 {
            fw_api::close(PLAYER.fd);
            PLAYER.fd = -1;
        }
        if MOTOR.fd >= 0 { motor_stop(); }
    }
}

fn tone_byte(phase: &mut u32, step: u32) -> u8 {
    // 三角波近似正弦，免浮点；幅度 48。
    *phase = phase.wrapping_add(step) & 0xFFFF;
    let p = *phase;
    let tri = if p < 0x8000 { p } else { 0xFFFF - p };
    // tri 0..0x8000 -> -48..48
    let v = (tri as i32 * 96 / 0x8000) - 48;
    v as i8 as u8
}

/// 10 ms 定时器/页面 tick 调用。返回本次写入 FIFO 的字节数。
pub fn pump() -> i32 {
    unsafe {
        if !PLAYER.playing { return 0; }
        if !ensure_fifo() {
            PLAYER.stall = PLAYER.stall.saturating_add(1);
            if PLAYER.stall > 300 { fail(E_STALL); }
            return 0;
        }

        // 先清掉上次因 FIFO 满而留下的尾巴，避免丢样本。
        let mut total = feed_pending();
        if !PLAYER.playing { return total; }
        if PLAYER.pending_off < PLAYER.pending_len {
            PLAYER.stall = PLAYER.stall.saturating_add(1);
            if PLAYER.stall > 300 {
                fail(E_STALL);
            }
            return total;
        }

        if PLAYER.tone {
            if PLAYER.tone_left == 0 {
                stop();
                PLAYER.err = 2;
                return total;
            }
            let n = (PLAYER.tone_left as usize).min(OUT_MAX);
            let step = (220u32 * 65536) / PLAYER.rate.max(1);
            for i in 0..n {
                PLAYER.out[i] = tone_byte(&mut PLAYER.tone_phase, step);
            }
            PLAYER.pending_off = 0;
            PLAYER.pending_len = n;
            PLAYER.tone_left -= n as u32;
            total += feed_pending();
            if !PLAYER.playing { return total; }
            if PLAYER.tone_left == 0 && PLAYER.pending_off == PLAYER.pending_len {
                stop();
                PLAYER.err = 2;
            }
            return total;
        }

        let got = fw_api::read(PLAYER.fd, PLAYER.src.as_mut_ptr(), SRC_MAX as u32);
        if got < 0 { fail(got); return total; }
        if got == 0 {
            stop();
            PLAYER.err = 2;
            return total;
        }
        let n = got as usize;
        let out_n = if PLAYER.rate == 24_000 {
            for i in 0..n {
                let v = PLAYER.src[i];
                PLAYER.out[i * 2] = v;
                PLAYER.out[i * 2 + 1] = v;
            }
            n * 2
        } else {
            PLAYER.out[..n].copy_from_slice(&PLAYER.src[..n]);
            n
        };
        PLAYER.pending_off = 0;
        PLAYER.pending_len = out_n;
        total += feed_pending();
        if total > 0 {
            PLAYER.stall = 0;
            if !PLAYER.logged_feed {
                PLAYER.logged_feed = true;
                crate::log::kv("马达FIFO写入 ", total as u32);
            }
        }
        total
    }
}

unsafe fn feed_pending() -> i32 {
    let mut total = 0i32;
    while PLAYER.pending_off < PLAYER.pending_len {
        let off = PLAYER.pending_off;
        let chunk = (PLAYER.pending_len - off).min(256);
        let rc = motor_feed(PLAYER.out.as_ptr().add(off), chunk);
        if rc < 0 { fail(rc); return total; }
        if rc == 0 { break; }
        PLAYER.pending_off += rc as usize;
        total += rc;
    }
    total
}

fn fail(code: i32) {
    unsafe {
        PLAYER.err = code;
        stop();
    }
}
