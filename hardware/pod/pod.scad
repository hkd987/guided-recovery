// Guided Recovery Pod, S size: printed parts and component envelopes.
//
// One file is the single source of truth. Render one thing at a time:
//   openscad -D 'part="sled"' -o sled.stl pod.scad
// part = base | sled | winch | roof | sleeve | spool | rbf_tag   (printed)
//        env_<name>                                    (component envelopes, for the fit check)
//        assembly | exploded                           (pictures)
//
// Coordinates: z up the pod's axis (0 = outside of the base, the rocket end;
// the top is the canopy end). +x is the side with the Heltec's screen.
// Millimetres. Component sizes come from the manufacturers' drawings and
// board files; see README.md for sources and which ones to measure.

part = "assembly";
kit = true;          // draw the backup kit's second barometer and GPS
$fn = 64;

// ---------------------------------------------------------------- sizes
OD = 62;             // fits 2.6 in tubes (ID 64.8-65.0 mm) and larger
WALL = 1.2;          // three 0.4 mm perimeters; the sleeve carries no load
R_IN = OD / 2 - WALL;            // 29.8
CLR = 0.15;                      // printed part to sleeve
DECK_R = R_IN - CLR;
SPINE_OD = 10; SPINE_ID = 7;     // Kevlar runs through the spine

FLOOR_T = 2.4;
Z_A = 18;            // sled deck bottom
DECK_T = 2;
Z_B = 86;            // winch deck bottom
Z_RP = 122.4;        // roof plate bottom
ROOF_T = 1.6;
H = Z_RP + ROOF_T;   // 124

// Motors: Pololu micro metal gearmotor 10 x 12 mm, 32.5 mm with encoder
// (side-entry board), 3 mm D output shaft 10 mm long.
MOT_W = 10; MOT_H = 12; MOT_L = 32.5; SHAFT_L = 10;
MOT_Y = SPINE_OD / 2 + 0.5 + MOT_W / 2;   // 10.5: motor axis offset from the centre line
MOT_Z = 101;                               // motor axis height
FACE_X = 9.2;        // gearbox face, |x|
SPOOL_D = 16; FLANGE_D = 22; SPOOL_W = 7;
SPOOL_X0 = FACE_X + 2.5;                   // spool inner face, |x|
LINE_X = SPOOL_X0 + SPOOL_W / 2;           // 15.2: where the line runs up
LINE_Y = MOT_Y + SPOOL_D / 2;              // 18.5

// ---------------------------------------------------------------- helpers
module box(x0, x1, y0, y1, z0, z1) { translate([x0, y0, z0]) cube([x1 - x0, y1 - y0, z1 - z0]); }
module disc(r, z0, z1) { translate([0, 0, z0]) cylinder(r = r, h = z1 - z0); }
module tube(ro, ri, z0, z1) { difference() { disc(ro, z0, z1); disc(ri, z0 - 1, z1 + 1); } }
module mirror_lr() { children(); rotate([0, 0, 180]) children(); }   // left winch -> right winch

// A radial screw boss at angle a (deg), for an M2 screw through the sleeve.
module boss(a, z0) {
  rotate([0, 0, a]) difference() {
    intersection() { box(R_IN - 6.5, R_IN, -3, 3, z0, z0 + 6); disc(DECK_R, z0, z0 + 6); }
    translate([R_IN - 8, 0, z0 + 3]) rotate([0, 90, 0]) cylinder(d = 1.7, h = 10);
  }
}
module boss_hole(a, z0) { rotate([0, 0, a]) translate([R_IN - 1, 0, z0 + 3]) rotate([0, 90, 0]) cylinder(d = 2.3, h = 6); }

// ---------------------------------------------------------------- component envelopes
// Each is a solid of the space the part (and its solder joints) occupies.

// Heltec WiFi LoRa 32 V3, 50.2 x 25.5 x 10.2 mm, standing on its USB end,
// screen facing +x. PCB at x 9.1..10.7, back parts toward the spine.
HZ0 = Z_A + DECK_T + 1.6;        // PCB bottom edge (USB-C overhangs 1.3 below)
module env_heltec() {
  box(9.1, 10.7, -12.75, 12.75, HZ0, HZ0 + 50.2);          // PCB
  box(10.7, 16.0, -11.0, 11.0, HZ0, HZ0 + 50.2);           // screen side
  box(5.6, 9.1, -11.0, 11.0, HZ0, HZ0 + 50.2);             // back side (ICs, battery socket)
  box(6.9, 12.9, -4.6, 4.6, HZ0 - 1.4, HZ0 + 7.5);         // USB-C receptacle
}
module env_usb_plug() { box(6.6, 13.2, -6.3, 6.3, -12, HZ0 - 1.4); }   // the cable's plug, from below

// Tattu 2S 450 mAh, long version, 61 x 16 x 15 mm; leads bend over the top.
module env_battery() { box(-20.5, -5.5, -8, 8, Z_A + DECK_T + 0.6, Z_A + DECK_T + 0.6 + 61); }
module env_battery_leads() { box(-19, -7, -6, 6, Z_A + DECK_T + 61.6, Z_B - 0.4); }

// Adafruit breakouts 25.4 x 17.78 mm standing on end. y-columns: PCB slab
// plus 3 mm of parts facing the spine and 1 mm of solder behind.
module breakout_y(z0, side) {   // side +1: +y column, -1: -y column
  mirror([0, side < 0 ? 1 : 0, 0]) {
    box(-8.89, 8.89, 18.6, 20.2, z0, z0 + 25.4);           // PCB
    box(-7.89, 7.89, 15.6, 18.6, z0, z0 + 25.4);           // parts, STEMMA QT
    box(-7.89, 7.89, 20.2, 21.2, z0, z0 + 25.4);           // solder
  }
}
ZLO = Z_A + DECK_T + 2;          // lower slot
ZHI = Z_B - 27.4;                // upper slot
module env_drv8833() { breakout_y(ZHI, +1); }
module env_imu()     { breakout_y(ZLO, +1); }
module env_baro1()   { breakout_y(ZHI, -1); }
module env_baro2()   { breakout_y(ZLO, -1); }

// -x column, outside the battery: boards face the battery.
module env_adxl375() {          // optional high-g logger
  box(-26.6, -25.0, -8.89, 8.89, ZHI, ZHI + 25.4);
  box(-25.0, -22.3, -7.89, 7.89, ZHI, ZHI + 25.4);
  box(-27.3, -26.6, -7.89, 7.89, ZHI, ZHI + 25.4);
}
module env_buck() {             // Pololu D24V10F5, 17.8 x 12.7 x 3.6
  box(-26.6, -25.0, -8.9, 8.9, ZLO, ZLO + 12.7);
  box(-25.0, -22.3, -7.9, 7.9, ZLO, ZLO + 12.7);
  box(-27.3, -26.6, -7.9, 7.9, ZLO, ZLO + 12.7);
}

// Winches: left motor at +y, spool at -x; the right one is the same turned 180 deg.
module motor_body() {
  box(-FACE_X, -FACE_X + MOT_L, MOT_Y - MOT_W / 2, MOT_Y + MOT_W / 2, MOT_Z - MOT_H / 2, MOT_Z + MOT_H / 2);
  box(-FACE_X + MOT_L - 6.5, -FACE_X + MOT_L, MOT_Y - MOT_W / 2, MOT_Y + MOT_W / 2,
      MOT_Z + MOT_H / 2, MOT_Z + MOT_H / 2 + 4.4);                                   // encoder connector
  translate([-FACE_X, MOT_Y, MOT_Z]) rotate([0, -90, 0]) cylinder(d = 3, h = SHAFT_L);  // output shaft
}
module spool_at() { translate([-SPOOL_X0 - SPOOL_W, MOT_Y, MOT_Z]) rotate([0, 90, 0]) cylinder(d = FLANGE_D, h = SPOOL_W); }
module env_motor_l() { motor_body(); }
module env_motor_r() { rotate([0, 0, 180]) motor_body(); }
module env_spool_l() { spool_at(); }
module env_spool_r() { rotate([0, 0, 180]) spool_at(); }
// Brake line from the spool straight up through the roof guide, and 20 mm beyond.
module env_line_l() { translate([-LINE_X, LINE_Y, MOT_Z]) cylinder(r = 0.8, h = H - MOT_Z + 20); }
module env_line_r() { rotate([0, 0, 180]) env_line_l(); }

// u-blox M10 (HGLRC M100 Mini) 15 x 15 x 5.2 under the roof, patch up.
GPS_Z1 = Z_RP - 0.2;
module env_gps1() { box(7, 22, -7.5, 7.5, GPS_Z1 - 5.2, GPS_Z1); }
module env_gps2() { box(-22, -7, -7.5, 7.5, GPS_Z1 - 5.2, GPS_Z1); }

// Base level.
module env_reed() { box(-24, -10, -14.5, 14.5, FLOOR_T + 0.4, FLOOR_T + 0.4 + 9); }      // Adafruit 375 switch half, lying flat
module env_buzzer() { translate([0, 17, 11]) rotate([-90, 0, 0]) cylinder(d = 12.2, h = 9.6); }
module env_watchdog() { box(-11, 11, -26.4, -14.6, FLOOR_T + 0.4, FLOOR_T + 0.4 + 9.5); } // perfboard + parts
SMA = [18, -10];
module env_sma() {   // bulkhead SMA: thread through the floor, body and nut inside
  translate([SMA[0], SMA[1], -8]) cylinder(d = 6.3, h = 8 + FLOOR_T);
  translate([SMA[0], SMA[1], FLOOR_T]) cylinder(d = 9.5, h = 9);
}

// Kevlar and its stoppers (a washer under each knot or sewn loop).
module env_kevlar() { disc(SPINE_ID / 2 - 0.6, -15, H + 15); }
module env_stopper_bottom() { disc(6, -3.5, -0.1); }
module env_stopper_top() { disc(6, H + 0.1, H + 3.5); }

// ---------------------------------------------------------------- printed parts

module spine(z0, z1) { tube(SPINE_OD / 2, SPINE_ID / 2, z0, z1); }

// Base: floor, spine, mounts for the reed switch, buzzer, watchdog board and
// antenna; holes for the Kevlar, the USB plug and the SMA connector.
module base() {
  difference() {
    union() {
      disc(DECK_R, 0, FLOOR_T);
      spine(FLOOR_T - 0.01, Z_A);
      // reed switch: end stops and a rib behind it (cable-tie slots in the floor)
      box(-24.6, -11.5, -15.3, -14.7, FLOOR_T, FLOOR_T + 6);
      box(-24.6, -11.5, 14.7, 15.3, FLOOR_T, FLOOR_T + 6);
      box(-9.6, -8.4, -10, 10, FLOOR_T, Z_A);                       // rib, also carries the deck
      // buzzer cradle: a ring segment around its body
      difference() {
        translate([0, 16.4, 11]) rotate([-90, 0, 0]) cylinder(d = 15.6, h = 8);
        translate([0, 15, 11]) rotate([-90, 0, 0]) cylinder(d = 12.4, h = 12);
        box(-10, 10, 10, 30, 13.5, 30);                               // open top
      }
      box(-7.8, 7.8, 16.4, 24.4, FLOOR_T, 4.6);
      // watchdog board: four standoffs (M2 self-tapping)
      for (x = [-9, 9]) for (y = [-24.4, -16.6]) translate([x, y, FLOOR_T]) cylinder(d = 3.6, h = 0.4);
      // deck supports
      for (a = [15, 180]) rotate([0, 0, a]) box(DECK_R - 4, DECK_R, -2, 2, FLOOR_T, Z_A);
      boss(45, FLOOR_T); boss(225, FLOOR_T);
    }
    disc(SPINE_ID / 2, -1, Z_A + 1);
    env_usb_plug();
    translate([SMA[0], SMA[1], -1]) cylinder(d = 6.6, h = FLOOR_T + 2);
    // cable-tie slots for the reed switch
    for (y = [-9, 9]) box(-22, -12, y - 1.5, y + 1.5, -1, FLOOR_T + 1);
    for (x = [-9, 9]) for (y = [-24.4, -16.6]) translate([x, y, -1]) cylinder(d = 1.6, h = 5);
    // keep the motorised parts' wiring path open: a slot through the rib
    box(-10, -8, -3, 3, FLOOR_T + 4, Z_A - 2);
  }
}

// Sled: the deck the electronics stand on, rails that hold each board by its
// edges, the battery cradle, and the spine.
module rail_y(side) {     // a pair of grooved rails for a y-column board, full column height
  mirror([0, side < 0 ? 1 : 0, 0]) for (sx = [-1, 1]) mirror([sx < 0 ? 1 : 0, 0, 0])
    difference() {
      box(7.95, 10.6, 15.0, 22.2, Z_A + DECK_T, Z_B - 0.5);
      box(7.6, 9.25, 18.45, 20.35, Z_A + DECK_T + 1, Z_B);          // groove for the PCB edge
    }
}
module sled() {
  zt = Z_A + DECK_T;
  difference() {
    union() {
      disc(DECK_R, Z_A, zt);
      spine(zt - 0.01, Z_B);
      rail_y(+1); rail_y(-1);
      // Heltec: grooved rails on both long edges, open at the top
      for (s = [-1, 1]) mirror([0, s < 0 ? 1 : 0, 0]) difference() {
        box(5.8, 14.8, 11.15, 14.6, zt, HZ0 + 50.2);
        box(8.95, 10.85, 12.0, 13.0, HZ0 - 0.2, Z_B);               // PCB groove
        box(8.95, 10.85, 11.0, 12.9, HZ0 - 0.2, Z_B);
      }
      // battery cradle: side walls and an outer wall the -x boards rest on
      for (s = [-1, 1]) mirror([0, s < 0 ? 1 : 0, 0]) box(-20.6, -6.0, 8.15, 9.35, zt, zt + 40);
      box(-22.15, -20.65, -9.35, 9.35, zt, Z_B - 0.5);
      // -x column lips (boards sit between these and the sleeve)
      for (s = [-1, 1]) mirror([0, s < 0 ? 1 : 0, 0]) box(-24.9, -22.1, 7.95, 9.9, zt, Z_B - 0.5);
      boss(135, zt); boss(315, zt);
    }
    disc(SPINE_ID / 2, Z_A - 1, Z_B + 1);
    env_usb_plug();
    // strap slots through the cradle walls (hook-and-loop strap round battery and spine)
    for (z = [zt + 10, zt + 30]) box(-21, -5, -10, 10, z, z + 3.2);
    // wire passes in the deck
    for (a = [60, 120, 240, 300]) rotate([0, 0, a]) translate([20, 0, Z_A - 1]) cylinder(d = 5, h = 4);
  }
}

// Winch deck: saddles and face plates for both motors, the spine, and the
// spool bays.
module motor_mount() {
  zt = Z_B + DECK_T;
  difference() {
    union() {
      // face plate between gearbox and spool
      box(-FACE_X - 1.9, -FACE_X - 0.1, MOT_Y - 6.5, MOT_Y + 6.5, zt, MOT_Z + 8);
      // two saddles under the body
      for (x = [-3, 12]) box(x, x + 4, MOT_Y - 6.5, MOT_Y + 6.5, zt, MOT_Z - MOT_H / 2);
    }
    translate([-FACE_X - 3, MOT_Y, MOT_Z]) rotate([0, 90, 0]) cylinder(d = 4.5, h = 4);   // shaft
    for (dz = [-4.5, 4.5]) translate([-FACE_X - 3, MOT_Y, MOT_Z + dz]) rotate([0, 90, 0]) cylinder(d = 1.8, h = 4);  // M1.6
    for (x = [-3, 12]) box(x + 1.2, x + 2.8, MOT_Y - 8, MOT_Y + 8, MOT_Z - MOT_H / 2 - 2.2, MOT_Z - MOT_H / 2 - 0.8); // tie slot
  }
}
module winch() {
  zt = Z_B + DECK_T;
  difference() {
    union() {
      disc(DECK_R, Z_B, zt);
      spine(zt - 0.01, Z_RP);
      motor_mount();
      rotate([0, 0, 180]) motor_mount();
      boss(45, zt); boss(225, zt);
    }
    disc(SPINE_ID / 2, Z_B - 1, Z_RP + 1);
    for (a = [60, 120, 240, 300]) rotate([0, 0, a]) translate([22, 0, Z_B - 1]) cylinder(d = 5, h = 4);
  }
}

// Roof: GPS pockets under a thin plate, PTFE line guides, the top Kevlar hole.
module gps_pocket(x0, x1) {
  difference() {
    box(x0 - 1.2, x1 + 1.2, -8.7, 8.7, GPS_Z1 - 5.8, Z_RP);
    box(x0 - 0.1, x1 + 0.1, -7.6, 7.6, GPS_Z1 - 7, Z_RP + 0.01);
    box(x0 + 3, x1 - 3, -9, 9, GPS_Z1 - 7, GPS_Z1 - 2.5);   // cable exit
  }
}
module roof() {
  difference() {
    union() {
      disc(DECK_R, Z_RP, H);
      gps_pocket(7, 22);
      gps_pocket(-22, -7);
      for (s = [0, 180]) rotate([0, 0, s]) translate([-LINE_X, LINE_Y, Z_RP - 6]) cylinder(d = 5, h = 6.01);  // guide bosses
      boss(90, Z_RP - 6); boss(270, Z_RP - 6);
    }
    disc(SPINE_ID / 2, Z_RP - 10, H + 1);
    for (s = [0, 180]) rotate([0, 0, s]) translate([-LINE_X, LINE_Y, Z_RP - 7]) cylinder(d = 2.15, h = 10);   // PTFE 2 mm OD
  }
}

// Sleeve: slides over the stack; screws into the bosses.
module sleeve() {
  difference() {
    tube(OD / 2, R_IN, 0, H);
    box(25, 40, -10, 10, HZ0 + 17, HZ0 + 45);                     // screen window
    for (z = [8, 11, 14]) translate([0, 20, z]) rotate([-90, 0, 0]) cylinder(d = 2, h = 20);  // buzzer
    // barometer vents: 45 deg away from the sensors, so light can't reach them
    for (a = [-45, -135]) for (z = [ZHI + 6, ZHI + 12, ZHI + 18]) rotate([0, 0, a]) translate([20, 0, z]) rotate([0, 90, 0]) cylinder(d = 1.5, h = 20);
    // RBF magnet pad: a shallow flat over the reed switch
    box(-OD / 2 - 1, -OD / 2 + 0.6, -8, 8, FLOOR_T + 0.4, FLOOR_T + 9.4);
    boss_hole(45, FLOOR_T); boss_hole(225, FLOOR_T);
    boss_hole(135, Z_A + DECK_T); boss_hole(315, Z_A + DECK_T);
    boss_hole(45, Z_B + DECK_T); boss_hole(225, Z_B + DECK_T);
    boss_hole(90, Z_RP - 6); boss_hole(270, Z_RP - 6);
  }
}

// Spool: 16 mm drum, 22 mm flanges, D bore for the 3 mm shaft, a hole to tie the line.
module spool() {
  difference() {
    union() {
      cylinder(d = FLANGE_D, h = 1.2);
      cylinder(d = SPOOL_D, h = SPOOL_W);
      translate([0, 0, SPOOL_W - 1.2]) cylinder(d = FLANGE_D, h = 1.2);
    }
    difference() { translate([0, 0, -1]) cylinder(d = 3.1, h = SPOOL_W + 2); translate([1.05, -2, -2]) cube([2, 4, SPOOL_W + 4]); }
    translate([0, 0, SPOOL_W / 2]) rotate([90, 0, 0]) cylinder(d = 1.2, h = 20, center = true);
  }
}

// Remove-before-flight tag: holds a 12.7 x 3.2 mm N42 disc magnet against the flat.
module rbf_tag() {
  difference() {
    hull() { cube([16, 18, 5]); translate([8, 60, 0]) cylinder(d = 12, h = 2); }
    translate([8, 9, 1.2]) cylinder(d = 12.9, h = 5);
    translate([8, 62, -1]) cylinder(d = 5, h = 4);
  }
}

// ---------------------------------------------------------------- pictures
module envelopes() {
  color("#3b82c4") env_heltec();
  color("#d9a441") env_battery();
  color("#2e8b57") { env_drv8833(); env_imu(); env_baro1(); if (kit) env_baro2(); env_adxl375(); env_buck(); }
  color("#555") { env_motor_l(); env_motor_r(); }
  color("#c8410b") { env_spool_l(); env_spool_r(); env_line_l(); env_line_r(); }
  color("#222") { env_gps1(); if (kit) env_gps2(); }
  color("#888") { env_reed(); env_buzzer(); env_watchdog(); env_sma(); }
  color("#8a5300") { env_kevlar(); env_stopper_bottom(); env_stopper_top(); }
}
module printed(e) {
  color("#e9eef3") translate([0, 0, 0 * e]) base();
  color("#dfe6ec") translate([0, 0, 1 * e]) sled();
  color("#d5dde5") translate([0, 0, 2 * e]) winch();
  color("#cbd5de") translate([0, 0, 3 * e]) roof();
}

if (part == "base") base();
else if (part == "sled") sled();
else if (part == "winch") winch();
else if (part == "roof") roof();
else if (part == "sleeve") sleeve();
else if (part == "spool") spool();
else if (part == "rbf_tag") rbf_tag();
else if (part == "assembly") { printed(0); envelopes(); color("#e9eef3", 0.35) sleeve(); }
else if (part == "cutaway") { printed(0); envelopes(); color("#e9eef3") difference() { sleeve(); box(-40, 40, -40, 0, -1, H + 1); } }
else if (part == "exploded") {
  printed(28);
  color("#3b82c4") translate([0, 0, 28]) env_heltec();
  color("#d9a441") translate([0, 0, 28]) env_battery();
  color("#2e8b57") translate([0, 0, 28]) { env_drv8833(); env_imu(); env_baro1(); env_baro2(); }
  color("#555") translate([0, 0, 56]) { env_motor_l(); env_motor_r(); }
  color("#c8410b") translate([0, 0, 56]) { env_spool_l(); env_spool_r(); }
  color("#222") translate([0, 0, 84]) { env_gps1(); env_gps2(); }
  color("#888") { env_reed(); env_buzzer(); env_watchdog(); }
  color("#e9eef3", 0.5) translate([85, 0, 0]) sleeve();
}
else if (part == "env_heltec") env_heltec();
else if (part == "env_usb_plug") env_usb_plug();
else if (part == "env_battery") env_battery();
else if (part == "env_battery_leads") env_battery_leads();
else if (part == "env_drv8833") env_drv8833();
else if (part == "env_imu") env_imu();
else if (part == "env_baro1") env_baro1();
else if (part == "env_baro2") env_baro2();
else if (part == "env_adxl375") env_adxl375();
else if (part == "env_buck") env_buck();
else if (part == "env_motor_l") env_motor_l();
else if (part == "env_motor_r") env_motor_r();
else if (part == "env_spool_l") env_spool_l();
else if (part == "env_spool_r") env_spool_r();
else if (part == "env_line_l") env_line_l();
else if (part == "env_line_r") env_line_r();
else if (part == "env_gps1") env_gps1();
else if (part == "env_gps2") env_gps2();
else if (part == "env_reed") env_reed();
else if (part == "env_buzzer") env_buzzer();
else if (part == "env_watchdog") env_watchdog();
else if (part == "env_sma") env_sma();
else if (part == "env_kevlar") env_kevlar();
else if (part == "env_stopper_bottom") env_stopper_bottom();
else if (part == "env_stopper_top") env_stopper_top();
