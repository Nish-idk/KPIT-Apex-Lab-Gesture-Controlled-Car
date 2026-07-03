// =============================================================
//  TX_ESP32_IMU.ino  –  Gesture Glove + Telemetry Dashboard
//  PWM Differential Drive version
// =============================================================

#include <WiFi.h>
#include <Wire.h>
#include <WebServer.h>
#include <esp_now.h>
#include <esp_mac.h>

// ── Dashboard AP ──────────────────────────────────────────────
const char* AP_SSID = "FireBird_Dashboard";
const char* AP_PASS = "dashboard123";

// ── ESP-01 MAC address ────────────────────────────────────────
uint8_t ESP01_MAC[] = {0x84, 0xF3, 0xEB, 0x7C, 0x9A, 0xEC};

// ── MPU6050 ───────────────────────────────────────────────────
const int     MPU_ADDR   = 0x68;
const int16_t TILT_ENTER = 4500;   // deadband entry threshold (raw accel units)
const int16_t PWM_CLAMP  = 13000;  // practical range out of ±16384 (1g)
const uint32_t SEND_MS   = 100;
uint32_t last_mpu_retry  = 0;

// ── Web dashboard ─────────────────────────────────────────────
WebServer dashboard(8080);

// ── Link / device status flags ────────────────────────────────
bool is_espnow_connected = false;
bool is_mpu_connected    = false;

// ── Last sent PWM values (for dashboard) ─────────────────────
uint8_t g_left_pwm  = 0;
uint8_t g_right_pwm = 0;
uint8_t g_dir_byte  = 0x00;

// ── Telemetry struct ──────────────────────────────────────────
struct Telemetry {
    float    bat_v;
    uint8_t  wl[3];
    uint8_t  ir[3];
    uint16_t dist_mm;
    uint32_t enc_left;
    uint32_t enc_right;
    char     cmd;
    float    pitch;
    float    roll;
    uint32_t last_update;
} telem = {0,{0,0,0},{0,0,0},0,0,0,'S',0.0f,0.0f,0};

// ── Telemetry line buffer ─────────────────────────────────────
#define TELEM_BUF 160
char    telem_line[TELEM_BUF];
uint8_t telem_idx = 0;

// ── Parse telemetry packet ────────────────────────────────────
void parse_telemetry(const char* line)
{
    unsigned int  bat_int, bat_frac;
    unsigned int  wl1, wl2, wl3;
    unsigned int  ir2, ir3, ir4;
    unsigned int  dist;
    unsigned long encL, encR;
    char          cmd;

    int n = sscanf(line,
        "$BAT:%u.%u,WL:%u:%u:%u,IR:%u:%u:%u,DIST:%u,ENC:%lu:%lu,CMD:%c",
        &bat_int, &bat_frac,
        &wl1, &wl2, &wl3,
        &ir2, &ir3, &ir4,
        &dist, &encL, &encR, &cmd);

    if (n == 12)
    {
        telem.bat_v     = bat_int + bat_frac / 100.0f;
        telem.wl[0]     = wl1; telem.wl[1] = wl2; telem.wl[2] = wl3;
        telem.ir[0]     = ir2; telem.ir[1] = ir3; telem.ir[2] = ir4;
        telem.dist_mm   = dist;
        telem.enc_left  = encL;
        telem.enc_right = encR;
        telem.cmd       = cmd;
        telem.last_update = millis();
    }
}

// ── ESP-NOW send callback (transition-only prints) ────────────
void onSent(const wifi_tx_info_t *info, esp_now_send_status_t status)
{
    if (status == ESP_NOW_SEND_SUCCESS)
    {
        if (!is_espnow_connected)
        {
            Serial.println(">> ESP-NOW LINK ACTIVE: RX acknowledged.");
            is_espnow_connected = true;
        }
    }
    else
    {
        if (is_espnow_connected)   // only print on transition connected→failed
        {
            Serial.println(">> ESP-NOW LINK FAILED: Delivery unacknowledged.");
            is_espnow_connected = false;
        }
    }
}

// ── ESP-NOW receive callback ──────────────────────────────────
void onReceive(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
    Serial.printf(">> Packet from ESP-01 (%d bytes): %.*s\n", len, len, (char*)data);
    if (len >= TELEM_BUF) len = TELEM_BUF - 1;
    memcpy(telem_line, data, len);
    telem_line[len] = '\0';
    Serial.printf(">> Received Packet: %s\n", telem_line);
    parse_telemetry(telem_line);
}

// ── Send 3-byte PWM command via ESP-NOW ──────────────────────
//   byte[0] = PORTA direction nibble (0x06 fwd, 0x09 bwd, 0x00 stop)
//   byte[1] = left  motor PWM (0-255)
//   byte[2] = right motor PWM (0-255)
void sendCommand(uint8_t dir, uint8_t lp, uint8_t rp)
{
    uint8_t pkt[3] = {dir, lp, rp};
    esp_now_send(ESP01_MAC, pkt, 3);
}

// ── IMU pitch/roll for dashboard ─────────────────────────────
void update_imu(int16_t ax, int16_t ay, int16_t az)
{
    float axf = ax / 16384.0f;
    float ayf = ay / 16384.0f;
    float azf = az / 16384.0f;
    telem.pitch = atan2f(axf, sqrtf(ayf*ayf + azf*azf)) * 180.0f / M_PI;
    telem.roll  = atan2f(ayf, sqrtf(axf*axf + azf*azf)) * 180.0f / M_PI;
}

// ── /data JSON endpoint ───────────────────────────────────────
void handleData()
{
    const char* cmdLabel = "STOP";
    switch (telem.cmd) {
        case 'F': cmdLabel = "FORWARD";  break;
        case 'B': cmdLabel = "BACKWARD"; break;
    }

    uint32_t age       = millis() - telem.last_update;
    bool isRobotOnline = (age < 1200) && (telem.last_update > 0);

    char json[900];
    snprintf(json, sizeof(json),
        "{"
        "\"bat\":%.2f,"
        "\"wl\":[%u,%u,%u],"
        "\"ir\":[%u,%u,%u],"
        "\"dist\":%u,"
        "\"enc\":[%lu,%lu],"
        "\"cmd\":\"%s\","
        "\"pitch\":%.1f,"
        "\"roll\":%.1f,"
        "\"espnow\":%s,"
        "\"robot\":%s,"
        "\"mpu\":%s,"
        "\"lpwm\":%u,"
        "\"rpwm\":%u"
        "}",
        telem.bat_v,
        telem.wl[0], telem.wl[1], telem.wl[2],
        telem.ir[0], telem.ir[1], telem.ir[2],
        telem.dist_mm,
        telem.enc_left, telem.enc_right,
        cmdLabel,
        telem.pitch, telem.roll,
        is_espnow_connected ? "true" : "false",
        isRobotOnline       ? "true" : "false",
        is_mpu_connected    ? "true" : "false",
        g_left_pwm,
        g_right_pwm
    );
    dashboard.send(200, "application/json", json);
}

// ── Dashboard HTML ────────────────────────────────────────────
void handleRoot()
{
    dashboard.send(200, "text/html", R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1.0">
<title>Firebird V Telemetry</title>
<style>
  @import url('https://fonts.googleapis.com/css2?family=Share+Tech+Mono&family=Orbitron:wght@400;700&display=swap');
  :root{--bg:#0a0a0f;--panel:#111118;--border:#1e1e2e;--accent:#00ff88;
    --accent2:#00ccff;--warn:#ffaa00;--danger:#ff4444;--text:#c8d0e0;--dim:#444466;}
  *{margin:0;padding:0;box-sizing:border-box;}
  body{background:var(--bg);color:var(--text);font-family:'Share Tech Mono',monospace;
    min-height:100vh;padding:16px;}
  body::before{content:'';position:fixed;inset:0;pointer-events:none;z-index:0;
    background:radial-gradient(ellipse at 50% 0%,#00ff8811 0%,transparent 60%),
               radial-gradient(ellipse at 80% 80%,#00ccff08 0%,transparent 50%);}
  .wrap{position:relative;z-index:1;max-width:700px;margin:0 auto;}
  h1{font-family:'Orbitron',monospace;font-size:clamp(1rem,4vw,1.5rem);color:var(--accent);
    letter-spacing:.2em;text-shadow:0 0 20px #00ff8866;margin-bottom:4px;}
  .sub{font-size:.7rem;color:var(--dim);letter-spacing:.15em;margin-bottom:20px;}
  .srow{display:flex;align-items:center;gap:20px;margin-bottom:20px;flex-wrap:wrap;}
  .status-item{display:flex;align-items:center;gap:8px;}
  .dot{width:8px;height:8px;border-radius:50%;background:var(--accent);
    box-shadow:0 0 8px var(--accent);animation:pulse 2s infinite;}
  .dot.stale{background:var(--danger);box-shadow:0 0 8px var(--danger);animation:none;}
  @keyframes pulse{0%,100%{opacity:1}50%{opacity:.4}}
  .stxt{font-size:.72rem;color:var(--text);letter-spacing:.1em;}
  .grid{display:grid;grid-template-columns:1fr 1fr;gap:12px;margin-bottom:12px;}
  @media(max-width:480px){.grid{grid-template-columns:1fr;}}
  .card{background:var(--panel);border:.5px solid var(--border);border-radius:10px;padding:14px;}
  .ctitle{font-size:.65rem;color:var(--dim);letter-spacing:.15em;margin-bottom:10px;}
  .bigv{font-family:'Orbitron',monospace;font-size:1.8rem;font-weight:700;color:var(--accent);}
  .unit{font-size:.8rem;color:var(--dim);margin-left:4px;}
  .badge{display:inline-block;font-family:'Orbitron',monospace;font-size:1.1rem;
    font-weight:700;padding:6px 18px;border-radius:6px;letter-spacing:.2em;}
  .FORWARD{background:#00ff8822;border:1px solid #00ff8866;color:var(--accent);}
  .BACKWARD{background:#00ccff22;border:1px solid #00ccff66;color:var(--accent2);}
  .STOP{background:#ff444411;border:1px solid #ff444433;color:var(--danger);}
  .brow{display:flex;align-items:center;gap:10px;margin-bottom:8px;}
  .blbl{font-size:.65rem;color:var(--dim);width:28px;flex-shrink:0;}
  .btrk{flex:1;height:10px;background:#1e1e2e;border-radius:5px;overflow:hidden;}
  .bfil{height:100%;border-radius:5px;transition:width .25s ease;}
  .wl .bfil{background:var(--accent2);}
  .ir .bfil{background:var(--warn);}
  .pwm .bfil{background:var(--accent);}
  .bval{font-size:.65rem;color:var(--text);width:30px;text-align:right;flex-shrink:0;}
  .erow{display:flex;justify-content:space-between;margin-bottom:6px;font-size:.8rem;}
  .elbl{color:var(--dim);}  .eval{color:var(--accent);}
  .irow{display:flex;gap:24px;flex-wrap:wrap;}
  .iitem{display:flex;flex-direction:column;align-items:center;gap:4px;}
  .iang{font-family:'Orbitron',monospace;font-size:1.4rem;color:var(--accent2);}
  .ilbl{font-size:.6rem;color:var(--dim);}
  footer{margin-top:20px;text-align:center;font-size:.6rem;color:var(--dim);}
</style>
</head>
<body>
<div class="wrap">
  <h1>FIREBIRD V</h1>
  <div class="sub">TELEMETRY DASHBOARD · 192.168.4.1:8080</div>

  <div class="srow">
    <div class="status-item">
      <div class="dot" id="espDot"></div>
      <span class="stxt" id="espTxt">ESP-NOW · CHECKING</span>
    </div>
    <div class="status-item">
      <div class="dot" id="robDot"></div>
      <span class="stxt" id="robTxt">ROBOT · CHECKING</span>
    </div>
    <div class="status-item">
      <div class="dot" id="mpuDot"></div>
      <span class="stxt" id="mpuTxt">MPU6050 · CHECKING</span>
    </div>
  </div>

  <div class="grid">
    <!-- Battery -->
    <div class="card">
      <div class="ctitle">BATTERY</div>
      <div><span class="bigv" id="bat">--</span><span class="unit">V</span></div>
    </div>
    <!-- Command -->
    <div class="card">
      <div class="ctitle">COMMAND</div>
      <span class="badge STOP" id="cmd">STOP</span>
    </div>
  </div>

  <!-- Motor PWM -->
  <div class="card" style="margin-bottom:12px;">
    <div class="ctitle">MOTOR PWM</div>
    <div class="brow pwm">
      <span class="blbl">L</span>
      <div class="btrk"><div class="bfil" id="pwmL" style="width:0%"></div></div>
      <span class="bval" id="pwmLv">0</span>
    </div>
    <div class="brow pwm">
      <span class="blbl">R</span>
      <div class="btrk"><div class="bfil" id="pwmR" style="width:0%"></div></div>
      <span class="bval" id="pwmRv">0</span>
    </div>
  </div>

  <!-- IMU -->
  <div class="card" style="margin-bottom:12px;">
    <div class="ctitle">IMU ANGLES</div>
    <div class="irow">
      <div class="iitem">
        <span class="iang" id="pitch">--</span>
        <span class="ilbl">PITCH</span>
      </div>
      <div class="iitem">
        <span class="iang" id="roll">--</span>
        <span class="ilbl">ROLL</span>
      </div>
    </div>
  </div>

  <!-- White line -->
  <div class="card" style="margin-bottom:12px;">
    <div class="ctitle">WHITE LINE SENSORS</div>
    <div class="brow wl">
      <span class="blbl">WL1</span>
      <div class="btrk"><div class="bfil" id="wl0" style="width:0%"></div></div>
      <span class="bval" id="wl0v">0</span>
    </div>
    <div class="brow wl">
      <span class="blbl">WL2</span>
      <div class="btrk"><div class="bfil" id="wl1" style="width:0%"></div></div>
      <span class="bval" id="wl1v">0</span>
    </div>
    <div class="brow wl">
      <span class="blbl">WL3</span>
      <div class="btrk"><div class="bfil" id="wl2" style="width:0%"></div></div>
      <span class="bval" id="wl2v">0</span>
    </div>
  </div>

  <!-- IR -->
  <div class="card" style="margin-bottom:12px;">
    <div class="ctitle">IR SENSORS</div>
    <div class="brow ir">
      <span class="blbl">IR1</span>
      <div class="btrk"><div class="bfil" id="ir0" style="width:0%"></div></div>
      <span class="bval" id="ir0v">0</span>
    </div>
    <div class="brow ir">
      <span class="blbl">IR2</span>
      <div class="btrk"><div class="bfil" id="ir1" style="width:0%"></div></div>
      <span class="bval" id="ir1v">0</span>
    </div>
    <div class="brow ir">
      <span class="blbl">IR3</span>
      <div class="btrk"><div class="bfil" id="ir2" style="width:0%"></div></div>
      <span class="bval" id="ir2v">0</span>
    </div>
  </div>

  <!-- Distance + Encoders -->
  <div class="card">
    <div class="ctitle">DISTANCE &amp; ENCODERS</div>
    <div class="erow"><span class="elbl">SHARP (mm)</span><span class="eval" id="dist">--</span></div>
    <div class="erow"><span class="elbl">ENC LEFT</span><span class="eval" id="encL">--</span></div>
    <div class="erow"><span class="elbl">ENC RIGHT</span><span class="eval" id="encR">--</span></div>
    <div class="erow"><span class="elbl">DISTANCE TRAVEL</span><span class="eval" id="distT">--</span></div>
    <div class="erow"><span class="elbl">ANGLE (hard turn)</span><span class="eval" id="angH">--</span></div>
    <div class="erow"><span class="elbl">ANGLE (soft turn)</span><span class="eval" id="angS">--</span></div>
  </div>

  <footer>FIREBIRD V · ESP32 + ESP-01 + ATMEGA2560</footer>
</div>
<script>
function setDot(id, txtId, ok, label) {
  const d = document.getElementById(id);
  const t = document.getElementById(txtId);
  if (ok) { d.classList.remove('stale'); t.textContent = label + ' · ONLINE'; }
  else     { d.classList.add('stale');    t.textContent = label + ' · OFFLINE'; }
}

function update() {
  fetch('/data').then(r=>r.json()).then(d=>{

    // ── Status dots ──
    setDot('espDot','espTxt', d.espnow, 'ESP-NOW');
    setDot('robDot','robTxt', d.robot,  'ROBOT');
    setDot('mpuDot','mpuTxt', d.mpu,    'MPU6050');

    // ── Battery ──
    document.getElementById('bat').textContent = d.bat.toFixed(2);

    // ── Command badge ──
    const el = document.getElementById('cmd');
    el.textContent = d.cmd;
    el.className = 'badge ' + d.cmd;

    // ── IMU ──
    document.getElementById('pitch').textContent = d.pitch.toFixed(1) + '°';
    document.getElementById('roll').textContent  = d.roll.toFixed(1)  + '°';

    // ── Motor PWM ──
    document.getElementById('pwmL').style.width  = (d.lpwm / 255 * 100) + '%';
    document.getElementById('pwmR').style.width  = (d.rpwm / 255 * 100) + '%';
    document.getElementById('pwmLv').textContent = d.lpwm;
    document.getElementById('pwmRv').textContent = d.rpwm;

    // ── White line sensors ──
    d.wl.forEach((v,i)=>{
      document.getElementById('wl'+i).style.width   = (v/255*100) + '%';
      document.getElementById('wl'+i+'v').textContent = v;
    });

    // ── IR sensors ──
    d.ir.forEach((v,i)=>{
      document.getElementById('ir'+i).style.width   = (v/255*100) + '%';
      document.getElementById('ir'+i+'v').textContent = v;
    });

    // ── Sharp distance ──
    document.getElementById('dist').textContent = d.dist;

    // ── Encoder pulses ──
    document.getElementById('encL').textContent = d.enc[0];
    document.getElementById('encR').textContent = d.enc[1];

    // ── Derived encoder values ──
    const LINEAR_RES = 5.44;
    const HARD_RES   = 4.090;
    const SOFT_RES   = 2.045;
    const avgPulses  = (d.enc[0] + d.enc[1]) / 2.0;
    const distMM     = avgPulses * LINEAR_RES;
    const distCM     = distMM / 10.0;
    document.getElementById('distT').textContent =
        distMM.toFixed(1) + ' mm (' + distCM.toFixed(1) + ' cm)';
    const hardAngle = Math.max(d.enc[0], d.enc[1]) * HARD_RES;
    document.getElementById('angH').textContent = hardAngle.toFixed(1) + '°';
    const softAngle = Math.max(d.enc[0], d.enc[1]) * SOFT_RES;
    document.getElementById('angS').textContent = softAngle.toFixed(1) + '°';

  }).catch(()=>{
    ['espDot','robDot','mpuDot'].forEach(id=>document.getElementById(id).classList.add('stale'));
    document.getElementById('espTxt').textContent = 'ESP-NOW · NO RESPONSE';
    document.getElementById('robTxt').textContent = 'ROBOT · OFFLINE';
    document.getElementById('mpuTxt').textContent = 'MPU6050 · UNKNOWN';
  });
}
setInterval(update, 300);
update();
</script>
</body>
</html>
)rawliteral");
}

// ── Setup ─────────────────────────────────────────────────────
void setup()
{
    Serial.begin(115200);
    delay(100);

    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS, 1);

    uint8_t baseMac[6];
    esp_read_mac(baseMac, ESP_MAC_WIFI_SOFTAP);
    Serial.print("TX ESP32 AP MAC (copy into RX_ESP01 TX_MAC[]): ");
    Serial.printf("%02X:%02X:%02X:%02X:%02X:%02X\n",
                  baseMac[0], baseMac[1], baseMac[2],
                  baseMac[3], baseMac[4], baseMac[5]);
    Serial.print("Dashboard IP: ");
    Serial.println(WiFi.softAPIP());

    // ── MPU6050 init ──────────────────────────────────────────
    int mpu_retries = 0;
    while (!is_mpu_connected && mpu_retries < 8) {
        Serial.printf("Connecting to MPU6050 (Attempt %d/8)...\n", mpu_retries + 1);
        Wire.begin(21, 22);
        delay(100);
        Wire.beginTransmission(MPU_ADDR);
        byte error = Wire.endTransmission();
        if (error == 0) {
            Wire.beginTransmission(MPU_ADDR);
            Wire.write(0x6B);
            Wire.write(0x00);
            if (Wire.endTransmission() == 0) {
                is_mpu_connected = true;
                Serial.println("-> MPU6050 successfully initialized.");
            } else {
                Serial.println("-> Device found but power write rejected.");
            }
        } else {
            Serial.printf("-> No device at 0x%02X.\n", MPU_ADDR);
        }
        if (!is_mpu_connected) { mpu_retries++; delay(500); }
    }

    // ── ESP-NOW ───────────────────────────────────────────────
    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW init failed!"); return;
    }
    esp_now_register_send_cb(onSent);
    esp_now_register_recv_cb(onReceive);

    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, ESP01_MAC, 6);
    peer.channel = 1;
    peer.encrypt = false;
    peer.ifidx   = WIFI_IF_AP;
    if (esp_now_add_peer(&peer) != ESP_OK) {
        Serial.println("Failed to add ESP-01 peer!"); return;
    }

    dashboard.on("/",     handleRoot);
    dashboard.on("/data", handleData);
    dashboard.begin();
    Serial.println("Dashboard ready.");
}

// ── Loop ─────────────────────────────────────────────────────
void loop()
{
    dashboard.handleClient();

    // ── MPU6050 reconnect ─────────────────────────────────────
    if (!is_mpu_connected) {
        if (millis() - last_mpu_retry > 2000) {
            last_mpu_retry = millis();
            Wire.begin(21, 22);
            Wire.beginTransmission(MPU_ADDR);
            if (Wire.endTransmission() == 0) {
                Wire.beginTransmission(MPU_ADDR);
                Wire.write(0x6B);
                Wire.write(0x00);
                if (Wire.endTransmission() == 0) {
                    is_mpu_connected = true;
                    Serial.println("MPU6050 reconnected.");
                }
            }
        }
    }

    int16_t ax = 0, ay = 0, az = 0;

    if (is_mpu_connected) {
        Wire.beginTransmission(MPU_ADDR);
        Wire.write(0x3B);
        if (Wire.endTransmission(false) == 0) {
            int received = Wire.requestFrom(MPU_ADDR, 6, true);
            if (received >= 6) {
                ax = (Wire.read()<<8)|Wire.read();
                ay = (Wire.read()<<8)|Wire.read();
                az = (Wire.read()<<8)|Wire.read();
                update_imu(ax, ay, az);
            } else {
                is_mpu_connected = false;
            }
        } else {
            is_mpu_connected = false;
        }
    }

    // ── PWM Differential Drive Mapping ───────────────────────
    // pitch (ax): forward/backward base speed
    // roll  (ay): differential between left and right motors
    //
    // Deadband: if both axes below TILT_ENTER → STOP
    // Clamp range: ±PWM_CLAMP (13000 out of ±16384)

    uint8_t dir     = 0x00;
    uint8_t left_pwm  = 0;
    uint8_t right_pwm = 0;

    int16_t ax_c = ax < -PWM_CLAMP ? -PWM_CLAMP : (ax > PWM_CLAMP ? PWM_CLAMP : ax);
    int16_t ay_c = ay < -PWM_CLAMP ? -PWM_CLAMP : (ay > PWM_CLAMP ? PWM_CLAMP : ay);

    int16_t base_speed_raw = abs(ax_c);
    uint8_t base_speed = (uint8_t)(base_speed_raw * 255L / PWM_CLAMP);

    // Deadband: below TILT_ENTER on pitch → treat as zero base speed
    if (base_speed_raw < TILT_ENTER) base_speed = 0;

    // Differential from roll: ±255 range
    int16_t differential = (int16_t)(ay_c * 255L / PWM_CLAMP);

    if (base_speed == 0 && abs(differential) < (TILT_ENTER * 255L / PWM_CLAMP)) {
        // Both axes in deadband → full stop
        dir       = 0x00;
        left_pwm  = 0;
        right_pwm = 0;
    } else {
        // Direction from pitch sign
        if (ax_c < 0)       dir = 0x06; // FORWARD
        else if (ax_c > 0)  dir = 0x09; // BACKWARD
        else                dir = 0x06; // pure spin defaults to forward port config

        // Blend: left gets +differential, right gets -differential
        // (positive ay = tilt right → left motor faster → curves right)
        int16_t lp = (int16_t)base_speed + differential;
        int16_t rp = (int16_t)base_speed - differential;

        // Clamp to 0-255
        bool clamped = false;
        if (lp > 255) { lp = 255; clamped = true; }
        if (lp < 0)   { lp = 0;   clamped = true; }
        if (rp > 255) { rp = 255; clamped = true; }
        if (rp < 0)   { rp = 0;   clamped = true; }

        if (clamped) {
            Serial.printf(">> PWM CLAMPED: base=%u diff=%d L=%d R=%d\n",
                          base_speed, differential, lp, rp);
        }

        left_pwm  = (uint8_t)lp;
        right_pwm = (uint8_t)rp;
    }

    // ── Debug print on change ─────────────────────────────────
    if (left_pwm != g_left_pwm || right_pwm != g_right_pwm || dir != g_dir_byte) {
        const char* dirLabel = (dir == 0x06) ? "FORWARD"
                             : (dir == 0x09) ? "BACKWARD"
                             :                 "STOP";
        Serial.printf(">> CMD: %s | L_PWM: %u | R_PWM: %u\n",
                      dirLabel, left_pwm, right_pwm);
        g_dir_byte  = dir;
        g_left_pwm  = left_pwm;
        g_right_pwm = right_pwm;
    }

    sendCommand(dir, left_pwm, right_pwm);

    delay(SEND_MS);
}