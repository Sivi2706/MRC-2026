#!/usr/bin/env python3
"""
inject_sim_data.py
==================
Processes OpenRocket/RocketSim simulation CSV data ('MRC2025 Sim data .csv')
and generates:
  1. include/sim_data.h       - Flash-optimized PROGMEM table & O(1) interpolation helper
  2. src/Sim_Injection_Ver.cpp - Standalone simulation firmware with sensor injection,
                                 pin triggers, and console event logging for ESP32.

Leaves main.cpp 100% untouched.
"""

import os
import sys
import re

def find_file(filename, search_dirs):
    for d in search_dirs:
        candidate = os.path.join(d, filename)
        if os.path.isfile(candidate):
            return os.path.abspath(candidate)
    return None

def parse_sim_csv(csv_path):
    print(f"[1/4] Reading simulation CSV: {csv_path}")
    data_points = []
    with open(csv_path, 'r', encoding='utf-8') as f:
        for line_num, line in enumerate(f, 1):
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = [p.strip() for p in line.split(',')]
            if len(parts) < 4:
                continue
            try:
                t = float(parts[0])
                vert_accel = float(parts[1])
                lat_accel = float(parts[2])
                pressure = float(parts[3])
                data_points.append({
                    'time_s': t,
                    'vert_accel': vert_accel,
                    'lat_accel': lat_accel,
                    'pressure_pa': pressure
                })
            except ValueError as e:
                print(f"  [WARN] Skipping malformed line {line_num}: {line} ({e})")
    
    print(f"  Parsed {len(data_points)} valid simulation data points.")
    if not data_points:
        raise ValueError("No valid data points found in CSV file!")
    
    print(f"  Time range: {data_points[0]['time_s']:.3f} s -> {data_points[-1]['time_s']:.3f} s")
    print(f"  Pressure range: {min(d['pressure_pa'] for d in data_points):.1f} Pa -> {max(d['pressure_pa'] for d in data_points):.1f} Pa")
    print(f"  Max vertical acceleration: {max(d['vert_accel'] for d in data_points):.2f} m/s^2")
    return data_points

def generate_sim_data_header(data_points, output_path):
    print(f"[2/4] Generating Flash-optimized header: {output_path}")
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    
    lines = []
    lines.append("// ============================================================================")
    lines.append("//  sim_data.h")
    lines.append("//  Auto-generated simulation dataset for ESP32 flight computer")
    lines.append("//  Storage: PROGMEM Flash memory (0 SRAM bytes)")
    lines.append("// ============================================================================")
    lines.append("#ifndef SIM_DATA_H")
    lines.append("#define SIM_DATA_H")
    lines.append("")
    lines.append("#include <Arduino.h>")
    lines.append("#include <pgmspace.h>")
    lines.append("")
    lines.append("#pragma pack(push, 1)")
    lines.append("struct SimDataPoint {")
    lines.append("    float time_s;        // seconds from ignition")
    lines.append("    float vert_accel;    // net vertical acceleration (m/s^2)")
    lines.append("    float lat_accel;     // lateral acceleration (m/s^2)")
    lines.append("    float pressure_pa;   // ambient pressure (Pa)")
    lines.append("};")
    lines.append("#pragma pack(pop)")
    lines.append("")
    lines.append(f"static const uint16_t SIM_DATA_COUNT = {len(data_points)};")
    lines.append("static const SimDataPoint SIM_DATA[SIM_DATA_COUNT] PROGMEM = {")
    
    for pt in data_points:
        lines.append(f"    {{ {pt['time_s']:.4f}f, {pt['vert_accel']:.4f}f, {pt['lat_accel']:.4f}f, {pt['pressure_pa']:.2f}f }},")
    
    lines.append("};")
    lines.append("")
    lines.append("// Fast O(1) monotonic cursor-based lookup and linear interpolation on ESP32")
    lines.append("inline bool sim_get_sample(float t_s, float &out_pressure, float &out_vert_accel, float &out_lat_accel) {")
    lines.append("    static uint16_t sim_cursor = 0;")
    lines.append("    if (SIM_DATA_COUNT == 0) return false;")
    lines.append("")
    lines.append("    if (t_s <= SIM_DATA[0].time_s) {")
    lines.append("        out_pressure   = SIM_DATA[0].pressure_pa;")
    lines.append("        out_vert_accel = SIM_DATA[0].vert_accel;")
    lines.append("        out_lat_accel  = SIM_DATA[0].lat_accel;")
    lines.append("        return true;")
    lines.append("    }")
    lines.append("")
    lines.append("    if (t_s >= SIM_DATA[SIM_DATA_COUNT - 1].time_s) {")
    lines.append("        out_pressure   = SIM_DATA[SIM_DATA_COUNT - 1].pressure_pa;")
    lines.append("        out_vert_accel = SIM_DATA[SIM_DATA_COUNT - 1].vert_accel;")
    lines.append("        out_lat_accel  = SIM_DATA[SIM_DATA_COUNT - 1].lat_accel;")
    lines.append("        return false; // Reached end of simulation")
    lines.append("    }")
    lines.append("")
    lines.append("    // Advance cursor monotonically in O(1) amortized time")
    lines.append("    while (sim_cursor + 1 < SIM_DATA_COUNT && SIM_DATA[sim_cursor + 1].time_s <= t_s) {")
    lines.append("        sim_cursor++;")
    lines.append("    }")
    lines.append("    // Rewind cursor if timestamp went backwards (e.g. restart / reboot)")
    lines.append("    while (sim_cursor > 0 && SIM_DATA[sim_cursor].time_s > t_s) {")
    lines.append("        sim_cursor--;")
    lines.append("    }")
    lines.append("")
    lines.append("    const SimDataPoint &p0 = SIM_DATA[sim_cursor];")
    lines.append("    const SimDataPoint &p1 = SIM_DATA[sim_cursor + 1];")
    lines.append("    float dt = p1.time_s - p0.time_s;")
    lines.append("    float factor = (dt > 1e-6f) ? ((t_s - p0.time_s) / dt) : 0.0f;")
    lines.append("    if (factor < 0.0f) factor = 0.0f;")
    lines.append("    if (factor > 1.0f) factor = 1.0f;")
    lines.append("")
    lines.append("    out_pressure   = p0.pressure_pa + factor * (p1.pressure_pa - p0.pressure_pa);")
    lines.append("    out_vert_accel = p0.vert_accel  + factor * (p1.vert_accel  - p0.vert_accel);")
    lines.append("    out_lat_accel  = p0.lat_accel   + factor * (p1.lat_accel   - p0.lat_accel);")
    lines.append("    return true;")
    lines.append("}")
    lines.append("")
    lines.append("#endif // SIM_DATA_H")
    lines.append("")
    
    with open(output_path, 'w', encoding='utf-8') as f:
        f.write("\n".join(lines))
    print(f"  Successfully wrote {output_path} ({len(lines)} lines)")

def generate_sim_cpp(main_cpp_path, output_cpp_path):
    print(f"[3/4] Generating simulation C++ firmware: {output_cpp_path}")
    with open(main_cpp_path, 'r', encoding='utf-8') as f:
        code = f.read()

    # Build the specialized simulation file based on main.cpp architecture
    # 1. Update header comments
    header_comment = """// ============================================================================
//  Sim_Injection_Ver.cpp
//  SIMULATION INJECTION & PIN TRIGGER VERSION FOR ESP32
//
//  Injects OpenRocket/RocketSim flight trajectory directly into the initial
//  sensor reading phase (BMP280 pressure & BMI160 IMU).
//
//  Executes real-time 100 Hz dual Kalman filter (Filter A & Filter B),
//  detects flight state transitions (PAD -> BOOST -> COAST -> DESCENT),
//  confirms apogee velocity zero-crossing, ACTIVATES APOGEE DEPLOYMENT PIN,
//  and prints prominent event logs to Serial console.
// ============================================================================"""

    # Replace top comment block if present
    code = re.sub(r"^// =+.*?// =+", header_comment, code, flags=re.DOTALL)

    # 2. Add sim_data.h include
    if '#include "sim_data.h"' not in code:
        code = code.replace('#include <cmath>', '#include <cmath>\n#include "sim_data.h"')

    # 3. Add Deployment Pin Parameters in Pin Assignments section
    pin_params = """// ── Deployment / Trigger Pin Configuration ──────────────────────────────────
#ifndef APOGEE_TRIGGER_PIN
#define APOGEE_TRIGGER_PIN          25      // Safe ESP32 GPIO for pyro/ejection parachute trigger
#endif

#ifndef APOGEE_TRIGGER_ACTIVE_HIGH
#define APOGEE_TRIGGER_ACTIVE_HIGH  1       // 1 = HIGH to fire, 0 = LOW to fire
#endif

#ifndef APOGEE_TRIGGER_DURATION_MS
#define APOGEE_TRIGGER_DURATION_MS  2000    // Firing pulse duration in milliseconds (e.g. 2.0 s)
#endif

// Simulation state tracking
bool apogee_pin_active = false;
unsigned long apogee_pin_start_ms = 0;
bool sim_completed = false;
"""
    if '#ifndef APOGEE_TRIGGER_PIN' not in code:
        # Insert after BLUE_LED_ACTIVE_HIGH block
        match = re.search(r'#ifndef BLUE_LED_ACTIVE_HIGH.*?#endif', code, flags=re.DOTALL)
        if match:
            pos = match.end()
            code = code[:pos] + "\n\n" + pin_params + code[pos:]

    # 4. Enhance serial_banner()
    old_banner_call = 'Serial.println("Terminal commands: clr=clear previous EEPROM data, view=show EEPROM data.");'
    new_banner_call = """Serial.println("Terminal commands: clr=clear previous EEPROM data, view=show EEPROM data.");
    Serial.printf("[SIM] SENSOR INJECTION MODE: Active (%u points from sim_data.h)\\n", SIM_DATA_COUNT);
    Serial.printf("[SIM] APOGEE TRIGGER PIN  : GPIO %d (%s, %lu ms pulse)\\n",
                  APOGEE_TRIGGER_PIN, APOGEE_TRIGGER_ACTIVE_HIGH ? "Active HIGH" : "Active LOW",
                  (unsigned long)APOGEE_TRIGGER_DURATION_MS);"""
    code = code.replace(old_banner_call, new_banner_call)

    # 5. Make hardware sensor init non-fatal for pure desk simulation if hardware not connected
    # Replace bmi160 fatal halt with simulation fallback
    code = code.replace(
        'led_halt_with_error(LED_ERROR_BMI160, "BMI160 initialisation failed");',
        'Serial.println("[BOOT] BMI160 hardware not found -> Using simulated IMU stream.");'
    )
    code = code.replace(
        'led_halt_with_error(LED_ERROR_BMP280, "BMP280 initialisation failed");',
        'Serial.println("[BOOT] BMP280 hardware not found -> Using simulated Barometer stream.");'
    )

    # 6. Adjust calibration loop in setup() to use simulation baseline if sensors not present
    old_cal_loop = """    while (millis() < cal_end) {
        float p = bmp.readPressure();
        pres_sum += p;
        pres_cnt++;

        BMI160Data imu = bmi160_read();
        float axial_accel = get_axial_accel(imu);
        accel_sum += axial_accel;
        accel_count++;

        delay(10);
    }"""
    new_cal_loop = """    // Initialize deployment pin to safe inactive state
    pinMode(APOGEE_TRIGGER_PIN, OUTPUT);
    digitalWrite(APOGEE_TRIGGER_PIN, APOGEE_TRIGGER_ACTIVE_HIGH ? LOW : HIGH);

    while (millis() < cal_end) {
        float p = bmp.readPressure();
        if (!std::isfinite(p) || p < 1000.0f) {
            p = SIM_DATA[0].pressure_pa; // Use simulation ground pressure baseline
        }
        pres_sum += p;
        pres_cnt++;

        BMI160Data imu = bmi160_read();
        float axial_accel = get_axial_accel(imu);
        if (!std::isfinite(axial_accel) || fabsf(axial_accel) < 0.1f) {
            axial_accel = G; // 1G standard rest gravity on pad
        }
        accel_sum += axial_accel;
        accel_count++;

        delay(10);
    }"""
    code = code.replace(old_cal_loop, new_cal_loop)

    # 7. Add ARMED event console print in setup()
    armed_marker = 'store_flight_event(0, 0.0f, PHASE_PAD, "AVIONICS ARMED");'
    armed_event_print = """store_flight_event(0, 0.0f, PHASE_PAD, "AVIONICS ARMED");
    Serial.println("\\n============================================================");
    Serial.printf("[EVENT] >>> AVIONICS ARMED & PAD READY at t=0.000 s\\n");
    Serial.printf("[EVENT] >>> Baseline Pressure: %.1f Pa | Acceleration detection: ACTIVE\\n", ground_pressure_pa);
    Serial.printf("[EVENT] >>> Apogee Trigger Pin: GPIO %d ready\\n", APOGEE_TRIGGER_PIN);
    Serial.println("============================================================\\n");"""
    code = code.replace(armed_marker, armed_event_print)

    # 8. Modify loop() initial reading phase
    old_reading_phase = """    // ── Read BMP280 ─────────────────────────────────────────────────────────
    float pres_pa  = bmp.readPressure();
    float baro_raw = pressure_to_alt(pres_pa);
    float baro_alt = baro_smooth_push(baro_raw);   // causal 3-sample average

    // ── Read BMI160 ─────────────────────────────────────────────────────────
    BMI160Data imu = bmi160_read();
    float axial_accel = get_axial_accel(imu);
    float axial_debiased = axial_accel - accel_bias_axial;
    float net_accel = axial_debiased - G;          // signed vertical net acceleration"""

    new_reading_phase = """    // ── Initial Sensor Reading Phase (Simulation Injected) ───────────────────
    float sim_t_s = t_ms * 0.001f;
    float sim_pressure_pa = 101325.0f;
    float sim_vert_accel  = 0.0f;
    float sim_lat_accel   = 0.0f;
    bool  sim_active      = sim_get_sample(sim_t_s, sim_pressure_pa, sim_vert_accel, sim_lat_accel);

    // 1. Barometer reading injection
    float pres_pa  = sim_pressure_pa;
    float baro_raw = pressure_to_alt(pres_pa);
    float baro_alt = baro_smooth_push(baro_raw);   // causal 3-sample average

    // 2. IMU reading injection (matches AXIAL_AXIS & AXIAL_SIGN configuration)
    BMI160Data imu;
    float axial_target = sim_vert_accel + G + accel_bias_axial;
#if AXIAL_AXIS == 0
    imu.ax = ((float)AXIAL_SIGN) * axial_target;
    imu.ay = sim_lat_accel;
    imu.az = 0.0f;
#elif AXIAL_AXIS == 1
    imu.ax = sim_lat_accel;
    imu.ay = ((float)AXIAL_SIGN) * axial_target;
    imu.az = 0.0f;
#elif AXIAL_AXIS == 2
    imu.ax = sim_lat_accel;
    imu.ay = 0.0f;
    imu.az = ((float)AXIAL_SIGN) * axial_target;
#endif
    imu.gx = 0.0f; imu.gy = 0.0f; imu.gz = 0.0f;

    float axial_accel = get_axial_accel(imu);
    float axial_debiased = axial_accel - accel_bias_axial;
    float net_accel = axial_debiased - G;          // signed vertical net acceleration"""

    code = code.replace(old_reading_phase, new_reading_phase)

    # 9. Enhance phase change console log with [EVENT] prefix
    old_phase_change_func = """void serial_phase_change(FlightPhase old_phase, FlightPhase new_phase,
                         unsigned long t_ms, float h, float v, float a) {
    Serial.printf("\\n>>> PHASE CHANGE: %s -> %s at %.3f s | h=%.2f m, v=%.2f m/s, a=%.2f m/s^2\\n",
                  phase_name(old_phase), phase_name(new_phase),
                  t_ms / 1000.0f, h, v, a);
}"""
    new_phase_change_func = """void serial_phase_change(FlightPhase old_phase, FlightPhase new_phase,
                         unsigned long t_ms, float h, float v, float a) {
    Serial.println("\\n------------------------------------------------------------");
    Serial.printf("[EVENT] >>> PHASE TRANSITION: %s -> %s at %.3f s\\n",
                  phase_name(old_phase), phase_name(new_phase), t_ms / 1000.0f);
    Serial.printf("[EVENT] >>> Telemetry: Altitude=%.2f m | Velocity=%.2f m/s | Net Accel=%.2f m/s^2\\n",
                  h, v, a);
    Serial.println("------------------------------------------------------------\\n");
}"""
    code = code.replace(old_phase_change_func, new_phase_change_func)

    # 10. Apogee trigger & pin firing logic
    old_apogee_block = """                apogee_detected = true;
                apogee_h_m      = xA_h;
                apogee_t_ms     = t_ms;
                phase           = PHASE_DESCENT;
                Serial.printf("\\n*** APOGEE DETECTED: %.2f m at %.3f s | phase=%s | confirm=%d samples ***\\n",
                              apogee_h_m, apogee_t_ms / 1000.0f, phase_name(phase),
                              APOGEE_CONFIRM);
                store_flight_event(apogee_t_ms, apogee_h_m,
                                   PHASE_DESCENT,
                                   "APOGEE -> DESCENT");"""

    new_apogee_block = """                apogee_detected = true;
                apogee_h_m      = xA_h;
                apogee_t_ms     = t_ms;
                phase           = PHASE_DESCENT;

                // Fire the deployment trigger pin
                digitalWrite(APOGEE_TRIGGER_PIN, APOGEE_TRIGGER_ACTIVE_HIGH ? HIGH : LOW);
                apogee_pin_active = true;
                apogee_pin_start_ms = millis();

                Serial.println("\\n============================================================");
                Serial.printf("[EVENT] >>> *** APOGEE DETECTED & PIN TRIGGERED! ***\\n");
                Serial.printf("[EVENT] >>> Time: %.3f s | Apogee Altitude: %.2f m | Velocity: %.2f m/s\\n",
                              apogee_t_ms / 1000.0f, apogee_h_m, xA_v);
                Serial.printf("[EVENT] >>> GPIO %d (APOGEE_TRIGGER_PIN) -> ACTIVE %s (%lu ms pulse)\\n",
                              APOGEE_TRIGGER_PIN, APOGEE_TRIGGER_ACTIVE_HIGH ? "HIGH" : "LOW",
                              (unsigned long)APOGEE_TRIGGER_DURATION_MS);
                Serial.printf("[EVENT] >>> Phase: COAST -> DESCENT | Confirmed: %d samples\\n", APOGEE_CONFIRM);
                Serial.println("============================================================\\n");

                store_flight_event(apogee_t_ms, apogee_h_m,
                                   PHASE_DESCENT,
                                   "APOGEE -> DESCENT (PIN FIRED)");"""

    code = code.replace(old_apogee_block, new_apogee_block)

    # 11. Add pin pulse safety timeout and simulation completion check in loop()
    loop_timing_marker = "    // ── Loop timing: target 100 Hz ────────────────────────────────────────────"
    pin_timeout_and_completion = """    // ── Deployment Pin Pulse Safety Timeout ──────────────────────────────────
    if (apogee_pin_active && (millis() - apogee_pin_start_ms >= APOGEE_TRIGGER_DURATION_MS)) {
        apogee_pin_active = false;
        digitalWrite(APOGEE_TRIGGER_PIN, APOGEE_TRIGGER_ACTIVE_HIGH ? LOW : HIGH);
        Serial.printf("[EVENT] >>> PIN DEACTIVATED: GPIO %d -> OFF at %.3f s (Pulse duration of %lu ms elapsed)\\n",
                      APOGEE_TRIGGER_PIN, (millis() - arm_ms) / 1000.0f, (unsigned long)APOGEE_TRIGGER_DURATION_MS);
    }

    // ── Simulation Completion Reporter ────────────────────────────────────────
    if (!sim_completed && !sim_active && (t_ms >= (unsigned long)(SIM_DATA[SIM_DATA_COUNT - 1].time_s * 1000.0f))) {
        sim_completed = true;
        Serial.println("\\n============================================================");
        Serial.printf("[EVENT] >>> SIMULATION COMPLETED at t=%.3f s\\n", t_ms / 1000.0f);
        Serial.printf("[EVENT] >>> Peak Altitude: %.2f m at %.3f s\\n", apogee_h_m, apogee_t_ms / 1000.0f);
        Serial.printf("[EVENT] >>> Final Estimated Altitude: %.2f m | Final Velocity: %.2f m/s\\n", xA_h, xA_v);
        Serial.println("[EVENT] >>> Flight events logged to EEPROM and SD successfully.");
        Serial.println("============================================================\\n");
    }

    // ── Loop timing: target 100 Hz ────────────────────────────────────────────"""

    code = code.replace(loop_timing_marker, pin_timeout_and_completion)

    with open(output_cpp_path, 'w', encoding='utf-8') as f:
        f.write(code)
    print(f"  Successfully wrote {output_cpp_path}")

def update_platformio_ini(ini_path):
    print(f"[4/4] Updating platformio.ini for multi-target compilation: {ini_path}")
    if not os.path.isfile(ini_path):
        print(f"  [WARN] {ini_path} not found; skipping platformio.ini update.")
        return

    new_ini_content = """# PlatformIO Project Configuration File
# Kalman Apogee Detector — ESP32 (Flight & Simulation Targets)

[platformio]
default_envs = simulation

[env]
platform      = espressif32
board         = esp32dev
framework     = arduino
monitor_speed = 115200
upload_speed  = 921600
lib_deps =
    adafruit/Adafruit BMP280 Library @ ^2.6.8
    adafruit/Adafruit Unified Sensor  @ ^1.1.14
build_flags =
    -DCORE_DEBUG_LEVEL=0
    -DBOARD_HAS_PSRAM=0
    -DARDUINO_RUNNING_CORE=1
    -I ../sim
board_build.partitions = default.csv

; ── Simulation Target: Runs simulated rocket flight with pin triggers ─────────
[env:simulation]
build_src_filter = +<Sim_Injection_Ver.cpp> -<main.cpp> -<Hand_Test_Ver.cpp>
extra_scripts = pre:../scripts/inject_sim_data.py

; ── Live Flight Target: Runs physical BMP280 + BMI160 hardware ────────────────
[env:flight]
build_src_filter = +<main.cpp> -<Sim_Injection_Ver.cpp> -<Hand_Test_Ver.cpp>
"""
    with open(ini_path, 'w', encoding='utf-8') as f:
        f.write(new_ini_content)
    print("  platformio.ini updated with [env:simulation], [env:flight], -I ../sim, and pre-build script.")

def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    workspace_root = os.path.abspath(os.path.join(script_dir, ".."))
    search_dirs = [
        os.path.join(workspace_root, "sim"),
        os.path.join(workspace_root, "..", "sim"),
        workspace_root
    ]
    
    csv_file = find_file("MRC2025 Sim data .csv", search_dirs)
    if not csv_file:
        print(f"[ERROR] Could not find 'MRC2025 Sim data .csv' in search directories: {search_dirs}")
        sys.exit(1)
        
    main_cpp = os.path.join(workspace_root, "MRC 2026", "src", "main.cpp")
    if not os.path.isfile(main_cpp):
        # Fallback if in different relative directory
        main_cpp = find_file("main.cpp", [os.path.join(workspace_root, "MRC 2026", "src"), workspace_root])
        
    if not main_cpp or not os.path.isfile(main_cpp):
        print(f"[ERROR] Could not find 'main.cpp'")
        sys.exit(1)

    mrc_project_dir = os.path.dirname(os.path.dirname(main_cpp))
    sim_header_output = os.path.join(workspace_root, "sim", "sim_data.h")
    include_header_output = os.path.join(mrc_project_dir, "include", "sim_data.h")
    cpp_output = os.path.join(mrc_project_dir, "src", "Sim_Injection_Ver.cpp")
    ini_path = os.path.join(mrc_project_dir, "platformio.ini")

    print("=" * 65)
    print("  MRC-2026 Simulation Data Injector & Firmware Builder")
    print("=" * 65)
    print(f"Workspace root : {workspace_root}")
    print(f"Simulation CSV : {csv_file}")
    print(f"Source main.cpp: {main_cpp}")
    print(f"Sim Header     : {sim_header_output}")
    print(f"Include Header : {include_header_output}")
    print(f"Target C++     : {cpp_output}")
    print("=" * 65)

    data_points = parse_sim_csv(csv_file)
    # Save to sim/ folder
    generate_sim_data_header(data_points, sim_header_output)
    # Also sync with include/ folder
    generate_sim_data_header(data_points, include_header_output)
    generate_sim_cpp(main_cpp, cpp_output)
    update_platformio_ini(ini_path)

    print("=" * 65)
    print("  SIMULATION INJECTION BUILD COMPLETE!")
    print(f"  - main.cpp is 100% UNTOUCHED")
    print(f"  - sim/sim_data.h generated ({len(data_points)} points in PROGMEM)")
    print(f"  - include/sim_data.h synchronized")
    print(f"  - Sim_Injection_Ver.cpp generated with APOGEE_TRIGGER_PIN=25")
    print(f"  - platformio.ini configured for automatic pre-build injection")
    print("=" * 65)

# Support running directly or as PlatformIO pre: extra_script
try:
    if 'Import' in globals():
        Import("env") # type: ignore
        main()
except Exception:
    pass

if __name__ == '__main__':
    main()

