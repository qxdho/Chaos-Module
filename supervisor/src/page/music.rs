//! 页37：马达音乐（S8 曲库 / 测试音）。
//!
//! 只读 `/data/wav/*.s8`：12 kHz、单声道、signed 8-bit 裸音频。
//! 播放引擎在 `crate::motor_audio`，这里只做扫描、选择和 UI。

use crate::page::*;
use crate::page::Page;
use crate::motor_audio;

use core::ptr;

pub(crate) const MUSIC_PID: u32 = 37;

const DIR: &[u8] = b"/data/wav/\0";
const MAX_TRACKS: usize = 6;
const NAME_MAX: usize = 56;
const PATH_MAX: usize = 96;
const STATUS_MAX: usize = 64;

pub(crate) struct MusicPage;

impl Page for MusicPage {
    fn fill_title(&self, buf: *mut u8) {
        unsafe { put_str(buf, "马达音乐".as_bytes()); }
    }

    fn render(&self, ctx: &PageCtx) -> PageResult {
        unsafe {
            if TRACK_N == 0 {
                scan();
            }
            let n = TRACK_N;
            if STATUS[0] == 0 {
                status_idle(n);
            }

            let st = core::slice::from_raw_parts(ptr::addr_of!(STATUS) as *const u8, STATUS_MAX);
            let mut last = build_row(ctx, ctx.title, 0, &RowSpec::display(st, fw_api::NONE_PTR));
            if last == 0 { return Err(PageError::RowFailed(0)); }
            let play_label: &[u8] = if motor_audio::is_playing() {
                "停止播放\0".as_bytes()
            } else {
                "播放选中曲目\0".as_bytes()
            };
            let r = build_row(ctx, last, 1, &RowSpec::clickable(play_label));
            if r != 0 { last = r; }

            vol_text();
            let r = build_row(ctx, last, 2, &RowSpec::tap(
                "音量\0".as_bytes(), ptr::addr_of!(VOL_TXT) as *const u8));
            if r != 0 { last = r; }
            let r = build_row(ctx, last, 3, &RowSpec::clickable(
                "测试音\0".as_bytes()));
            if r != 0 { last = r; }

            let mut i = 0usize;
            while i < n && i < MAX_TRACKS {
                let slot = 4 + i;
                let nm = core::slice::from_raw_parts(ptr::addr_of!(NAMES[i]) as *const u8, NAME_MAX);
                let spec = RowSpec::check(nm, b"S8\0".as_ptr());
                let row = build_row(ctx, last, slot, &spec);
                if row != 0 {
                    last = row;
                    fw_api::row_update(row, fw_api::NONE_PTR,
                        ptr::addr_of!(NAMES[i]) as *const u8,
                        b"S8\0".as_ptr(), 0, (i == SEL) as u8);
                }
                i += 1;
            }

            let r = build_row(ctx, last, 10, &RowSpec::clickable("重新扫描\0".as_bytes()));
            if r != 0 { last = r; }
            if build_row(ctx, last, 11, &RowSpec::clickable("返回\0".as_bytes())) == 0 {
                return Err(PageError::RowFailed(11));
            }
            Ok(())
        }
    }

    fn on_click(&self, idx: usize) {
        unsafe {
            match idx {
                1 => {
                    if motor_audio::is_playing() {
                        motor_audio::stop();
                        set_timer(500);
                        ARM = 0;
                        status_text(b"\xe5\xb7\xb2\xe5\x81\x9c\xe6\xad\xa2\0");
                        render_req(MUSIC_PID as usize);
                    } else if ARM == 0 {
                        ARM = 1;
                        status_text(b"\xe5\x86\x8d\xe7\x82\xb9\xe4\xb8\x80\xe6\xac\xa1\xe6\x92\xad\xe6\x94\xbe\0");
                        render_req(MUSIC_PID as usize);
                    } else {
                        ARM = 0;
                        play_selected();
                    }
                }
                2 => {
                    let v = match motor_audio::volume() { 0..=20 => 40, 21..=50 => 70, 51..=80 => 100, _ => 20 };
                    motor_audio::set_volume(v);
                    vol_text();
                    render_req(MUSIC_PID as usize);
                }
                3 => {
                    if ARM == 0 {
                        ARM = 1;
                        status_text(b"\xe5\x86\x8d\xe7\x82\xb9\xe4\xb8\x80\xe6\xac\xa1\xe6\xb5\x8b\xe8\xaf\x95\0");
                        render_req(MUSIC_PID as usize);
                    } else {
                        ARM = 0;
                        play_tone();
                    }
                }
                4..=9 => {
                    let i = idx - 4;
                    if i < TRACK_N {
                        SEL = i;
                        render_req(MUSIC_PID as usize);
                    }
                }
                10 => {
                    ARM = 0;
                    TRACK_N = 0;
                    render_req(MUSIC_PID as usize);
                }
                11 => nav_back(),
                _ => {}
            }
        }
    }

    fn needs_tick(&self) -> bool {
        motor_audio::is_playing() || unsafe { DIRTY != 0 }
    }

    fn tick(&self) {
        unsafe {
            if TIMER == 0 { let _ = motor_audio::pump(); }
            if DIRTY != 0 {
                DIRTY = 0;
                render_req(MUSIC_PID as usize);
            }
        }
    }
}

static mut TRACK_N: usize = 0;
static mut SEL: usize = 0;
static mut NAMES: [[u8; NAME_MAX]; MAX_TRACKS] = [[0; NAME_MAX]; MAX_TRACKS];
static mut STATUS: [u8; STATUS_MAX] = [0; STATUS_MAX];
static mut VOL_TXT: [u8; 16] = [0; 16];
static mut PATH: [u8; PATH_MAX] = [0; PATH_MAX];
static mut TIMER: u32 = 0;
static mut TIMER_PERIOD: u32 = 0;
static mut DIRTY: u32 = 0;
static mut ARM: u32 = 0;

pub(crate) static PAGE_MUSIC: MusicPage = MusicPage;

// ===== 状态文本 =====

unsafe fn status_text(msg: &[u8]) {
    let mut n = 0usize;
    while n + 1 < STATUS_MAX && n < msg.len() && msg[n] != 0 {
        STATUS[n] = msg[n];
        n += 1;
    }
    STATUS[n] = 0;
}

unsafe fn status_idle(n: usize) {
    let mut b = [0u8; STATUS_MAX];
    let pre = b"\xe6\x89\xbe\xe5\x88\xb0 ";
    let mut w = 0usize;
    for &c in pre { if w + 1 < STATUS_MAX { b[w] = c; w += 1; } }
    if n >= 10 { b[w] = b'0' + (n / 10) as u8; w += 1; }
    b[w] = b'0' + (n % 10) as u8; w += 1;
    for &c in b" \xe9\xa6\x96\0" { if w + 1 < STATUS_MAX { b[w] = c; w += 1; } }
    b[w] = 0;
    STATUS = b;
}

unsafe fn status_err(code: i32) {
    let mut b = [0u8; STATUS_MAX];
    let mut w = 0usize;
    for &c in b"\xe5\xa4\xb1\xe8\xb4\xa5 " { b[w] = c; w += 1; }
    if code < 0 {
        b[w] = b'-'; w += 1;
        let v = (-code) as u32;
        if v >= 100 { b[w] = b'0' + (v / 100) as u8; w += 1; }
        if v >= 10 { b[w] = b'0' + ((v / 10) % 10) as u8; w += 1; }
        b[w] = b'0' + (v % 10) as u8; w += 1;
    } else {
        b[w] = b'0'; w += 1;
    }
    b[w] = 0;
    STATUS = b;
}

unsafe fn vol_text() {
    let v = motor_audio::volume();
    let mut b = [0u8; 16];
    let mut w = 0usize;
    for &c in b"\xe9\x9f\xb3\xe9\x87\x8f " { b[w] = c; w += 1; }
    if v >= 100 { b[w] = b'1'; w += 1; b[w] = b'0'; w += 1; b[w] = b'0'; w += 1; }
    else if v >= 10 { b[w] = b'0' + (v / 10) as u8; w += 1; b[w] = b'0' + (v % 10) as u8; w += 1; }
    else { b[w] = b'0' + v as u8; w += 1; }
    b[w] = 0;
    VOL_TXT = b;
}

// ===== 曲库扫描 =====

unsafe fn scan() {
    TRACK_N = 0;
    SEL = 0;
    for i in 0..MAX_TRACKS { NAMES[i] = [0; NAME_MAX]; }

    let dir = fw_api::opendir(DIR.as_ptr());
    if dir == 0 {
        status_text(b"\xe6\x97\xa0 /data/wav\0");
        return;
    }
    let mut seen = 0usize;
    let mut guard = 0usize;
    loop {
        guard += 1;
        if guard > 512 { break; }
        let de = fw_api::readdir(dir);
        if de == 0 { break; }
        let name = (de + 1) as *const u8;
        let mut len = 0usize;
        while len < NAME_MAX - 1 && *name.add(len) != 0 { len += 1; }
        if len < 4 || len >= NAME_MAX - 1 { continue; }
        if *name.add(len - 3) != b'.' || *name.add(len - 2) != b's' || *name.add(len - 1) != b'8' {
            continue;
        }
        if TRACK_N >= MAX_TRACKS { continue; }
        let mut k = 0usize;
        while k < len { NAMES[TRACK_N][k] = *name.add(k); k += 1; }
        NAMES[TRACK_N][len] = 0;
        TRACK_N += 1;
        seen += 1;
    }
    fw_api::closedir(dir);
    if seen == 0 {
        status_text(b"\xe6\x97\xa0 .s8 \xe6\x9b\xb2\xe7\x9b\xae\0");
    } else {
        status_idle(seen);
    }
}

// ===== 播放控制 =====

unsafe fn build_path(i: usize) -> usize {
    let mut w = 0usize;
    let pre = b"/data/wav/";
    for &c in pre { if w + 1 < PATH_MAX { PATH[w] = c; w += 1; } }
    let mut k = 0usize;
    while k < NAME_MAX && NAMES[i][k] != 0 && w + 1 < PATH_MAX {
        PATH[w] = NAMES[i][k];
        w += 1;
        k += 1;
    }
    PATH[w] = 0;
    w
}

unsafe fn play_selected() {
    if TRACK_N == 0 {
        status_text(b"\xe8\xaf\xb7\xe5\x85\x88\xe6\x94\xbe\xe5\x85\xa5 .s8\0");
        render_req(MUSIC_PID as usize);
        return;
    }
    ARM = 0;
    if SEL >= TRACK_N { SEL = 0; }
    let n = build_path(SEL);
    let rc = motor_audio::play_file(&PATH[..n]);
    if rc < 0 {
        crate::log::kv("马达播放失败 ", (-rc) as u32);
        status_err(rc);
        set_timer(500);
        render_req(MUSIC_PID as usize);
        return;
    }
    status_text(b"\xe6\x92\xad\xe6\x94\xbe\xe4\xb8\xad\0");
    set_timer(10);
    DIRTY = 1;
}

unsafe fn play_tone() {
    ARM = 0;
    let rc = motor_audio::play_tone();
    if rc < 0 {
        crate::log::kv("马达测试音失败 ", (-rc) as u32);
        status_err(rc);
        set_timer(500);
        render_req(MUSIC_PID as usize);
        return;
    }
    status_text(b"\xe6\xb5\x8b\xe8\xaf\x95\xe9\x9f\xb3 220Hz\0");
    set_timer(10);
    DIRTY = 1;
}

// ===== 10 ms 泵 =====

unsafe extern "C" fn pump_cb(_t: u32) {
    let was_playing = motor_audio::is_playing();
    let _ = motor_audio::pump();
    if was_playing && !motor_audio::is_playing() {
        if motor_audio::status() == 2 {
            status_text(b"\xe6\x92\xad\xe6\x94\xbe\xe7\xbb\x93\xe6\x9d\x9f\0");
        } else if motor_audio::status() < 0 {
            status_err(motor_audio::last_error());
        }
        DIRTY = 1;
        set_timer(500);
    }
}

unsafe fn set_timer(period: u32) {
    if TIMER == 0 {
        let t = fw_api::timer_create(pump_cb as *const () as u32, period, 0);
        if t != 0 {
            TIMER = t;
            TIMER_PERIOD = period;
        }
    } else if TIMER_PERIOD != period {
        fw_api::timer_set_period(TIMER, period);
        TIMER_PERIOD = period;
    }
}
