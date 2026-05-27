//--------------------------------------------------------------------------
// ME327 Team 10 - Pantograph Haptic Shape Renderer
// Penetration depth + Force feedback function
//
// This file implements:
//   (1) A closed-loop shape (default: fat round-cornered star, 180 pts)
//       with a clearly marked slot for swapping in real Python-generated shapes
//   (2) Arc-length precomputation for texture rendering
//   (3) CCW winding check at startup (signed area via shoelace formula)
//   (4) Nearest-segment search + signed distance (penetration depth)
//   (5) Ray-casting inside/outside test
//   (6) Force feedback model with three regions on SIGNED penetration depth
//       (d > 0 = inside, d < 0 = outside):
//         d > 0:          linear wall spring + damping + (optional) texture
//         P_T < d <= 0:   dead zone, F = 0
//         d <= P_T:       exponential guidance pulling toward outline
//   (7) Two modes: MODE_CONTROL (no texture) and MODE_TEXTURED (sinusoidal)
//   (8) Forward kinematics (geometric, with corrected signs)
//   (9) Jacobian transpose (Cartesian force -> motor torques)
//   (10) PWM motor command output with direction control
//
// [TUNE WHEN HARDWARE READY] markers throughout the file flag values
// that need to be confirmed or re-calibrated once the pantograph is
// physically built.
//--------------------------------------------------------------------------

#include <math.h>
#include <avr/pgmspace.h>
#include <SoftwareSerial.h>

// ============================================================
// BOARD ROLE
// ============================================================
// Both Hapkit boards run identical firmware EXCEPT for this single line.
// Comment/uncomment to indicate which motor this board controls:
//
//   #define IS_MOTOR_1   <- this board drives motor 1 (theta_1 = own encoder)
//   (commented out)      <- this board drives motor 5 (theta_5 = own encoder)
//
// Both boards independently compute the full force model and both
// torques (tau1, tau5) via the Jacobian transpose. Each board then
// writes ONLY ITS OWN torque to its local motor:
//   IS_MOTOR_1 defined   -> writes tau1
//   IS_MOTOR_1 undefined -> writes tau5
//
// Hardware communication: the two boards exchange joint angles via a
// SoftwareSerial link on pins D9 (TX) and D10 (RX), keeping the hardware
// UART on D0/D1 free for USB Serial Monitor debugging. No host PC is
// needed in the control loop. Each board sends its own theta as
// "A<value>\n" and receives the partner's theta the same way.
//
// Wiring between boards (in addition to common GND):
//   Board 1 D9 (TX)  ->  Board 2 D10 (RX)
//   Board 1 D10 (RX) <-  Board 2 D9  (TX)
#define IS_MOTOR_1


// ============================================================
// PANTOGRAPH LINK LENGTHS  [meters]
// ============================================================
// Used by forward kinematics and Jacobian (stubs at the bottom of this
// file). Modify here when CAD is finalized. Convention follows Campion
// et al. 2005, with motor pivots coincident (a5 = 0) for wider workspace.
//
// Note: named LINK_A1..LINK_A5 (not A1..A5) because Arduino reserves the
// short A1..A5 names for analog pin aliases (PIN_A1, PIN_A2, ...).
//
//        P3 (pen tip)
//       /  \
//     a3    a2
//     /      \
//    P4      P2
//    |        |
//    a4       a1
//    |        |
//      P5/P1   <-- both motor pivots; a5 = 0 (coincident)
//
// [TUNE WHEN HARDWARE READY: set these to the actual CAD lengths]
const float LINK_A1 = 0.060f;    // proximal link, motor 1 side
const float LINK_A2 = 0.080f;    // distal link, motor 1 side (P2 -> P3)
const float LINK_A3 = 0.080f;    // distal link, motor 5 side (P4 -> P3)
const float LINK_A4 = 0.060f;    // proximal link, motor 5 side
const float LINK_A5 = 0.000f;    // distance between motor pivots (coincident)


// ============================================================
// HAPKIT HARDWARE PIN ASSIGNMENTS
// ============================================================
// From HapkitBoardPinMapping.pdf (v 11.14.2013):
//   D5  Motor 1 PWM
//   D6  Motor 2 PWM (unused in two-board pantograph; used in single-board)
//   D7  Motor 2 direction (unused in two-board)
//   D8  Motor 1 direction
//   D9  Grove connector output (used here as SoftwareSerial TX to partner)
//   D10 Grove connector output (used here as SoftwareSerial RX from partner)
//   A2  MR sensor (this board's encoder)
// [TUNE WHEN HARDWARE READY: if this is the "motor 5" board rather than
//  the "motor 1" board, change neither the pin numbers nor the variable
//  names -- just swap which torque the local motor receives in
//  sendMotorCommand below.]
const int sensorPosPin   = A2;     // MR sensor input
const int PWM_PIN_LOCAL  = 5;      // D5 PWM for this board's motor
const int DIR_PIN_LOCAL  = 8;      // D8 direction for this board's motor

// SoftwareSerial pins for the inter-board UART (D9 TX, D10 RX). The
// hardware UART (D0/D1) is reserved for USB Serial Monitor debugging.
const int LINK_TX_PIN    = 9;      // D9 -> partner's D10
const int LINK_RX_PIN    = 10;     // D10 <- partner's D9
SoftwareSerial linkSerial(LINK_RX_PIN, LINK_TX_PIN);  // (rx, tx)


// ============================================================
// CAPSTAN DRIVE TRANSMISSION  (per Hapkit board)
// ============================================================
// On the Hapkit, the motor pulley rotates the sector pulley via a
// capstan-drive cable, with a transmission ratio set by their radii.
// Tp = F * (r_handle * r_pulley) / r_sector             (A3 Hapkit, scalar)
// For the pantograph, the SAME capstan transmission converts motor torque
// to torque-at-the-proximal-link, but the link rotation directly drives
// the corresponding theta (no separate "handle" variable here; the link
// itself acts as the handle).
//
// Default values are the A3 Hapkit radii. Re-measure for the pantograph
// build if the capstan radii differ (e.g. if you change the drum or use
// a different sector size).
// [TUNE WHEN HARDWARE READY]
const float R_PULLEY  = 0.0045f;    // m, motor capstan/drum radius   (A3: 0.0045)
const float R_SECTOR  = 0.075f;     // m, sector pulley radius         (A3: 0.075)
// R_HANDLE was 0.087 m on the A3 Hapkit but it represents the distance
// from the sector axis to the user's handle. On the pantograph the user
// doesn't grip the sector directly -- they hold the pen tip P3 -- so
// R_HANDLE doesn't apply the same way. The Jacobian transpose already
// maps F at P3 to torque at each motor's sector, so we only need the
// capstan ratio (R_PULLEY / R_SECTOR) here.
//
// Capstan ratio for converting sector torque to motor torque:
//   tau_motor = tau_sector * (R_PULLEY / R_SECTOR)
const float CAPSTAN_RATIO = R_PULLEY / R_SECTOR;


// ============================================================
// TORQUE-TO-PWM CALIBRATION
// ============================================================
// Conversion formula from A3: duty = sqrt(|tau_motor| / TORQUE_TO_DUTY_K)
// The constant 0.0183 was calibrated for the single-Hapkit setup
// (relates motor-pulley torque to PWM duty cycle through the motor's
// torque constant + amplifier gain).
// [TUNE WHEN HARDWARE READY: re-measure for the pantograph motors if
//  they differ from the A3 Hapkit's, e.g. by applying a known voltage
//  and measuring the stall torque.]
const float TORQUE_TO_DUTY_K = 0.0183f;


// ============================================================
// ENCODER CALIBRATION  (from A3)
// ============================================================
// [TUNE WHEN HARDWARE READY: re-measure on each Hapkit board]
const float ENC_M    = 0.0146f;            // deg per encoder count
const float ENC_B    = -2.64f;             // deg offset
const float SECTOR_GEAR_REDUCTION = 1.0f;  // (motor angle) / (sector angle).
                                           // 1.0 if encoder directly measures
                                           // motor pulley; else the gear ratio.


// ============================================================
// CONDITION TOGGLE
// ============================================================
// Switch between control (no texture) and textured rendering.
// Later this can be set by a serial command from Processing so
// the database can log which condition each trial used.
enum RenderMode { MODE_CONTROL = 0, MODE_TEXTURED = 1 };
RenderMode renderMode = MODE_TEXTURED;   // change for testing


// ============================================================
// FORCE MODEL PARAMETERS (placeholders based on A3/A4 values)
// ============================================================
// Sign convention: penetration_depth d is SIGNED.
//   d > 0   -> user is INSIDE the shape (positive = penetrated)
//   d < 0   -> user is OUTSIDE the shape
//   |d|     -> perpendicular distance to the nearest segment of the outline
//
// Three regions on signed d:
//   d > 0:          F = -K_WALL * d        (linear wall, pushes outward)
//                                          (+ texture in textured mode)
//                                          (+ wall damping when moving in)
//   P_T < d <= 0:   F = 0                  (dead zone near the outline)
//   d <= P_T:       F = A_GUIDE * exp(-C_GUIDE * (d - P_T))
//                                          (guidance pulling toward shape;
//                                           peaks at A_GUIDE at d = P_T,
//                                           grows as d gets more negative)

// Inside-shape linear wall stiffness.
const float K_WALL = 2000.0f;       // N/m

// Wall damping (only active when user is moving INTO the wall, v_n > 0).
// Adds B_WALL * v_normal to the wall force. Improves stability (lets us
// run K_WALL higher than the undamped A2 bound would allow) and makes
// impacts feel more solid by dissipating kinetic energy on contact.
const float B_WALL = 15.0f;         // Ns/m

// Dead zone width (P_T is negative -- it's a depth on the outside).
const float P_T = -0.003f;          // m, 3 mm dead zone outside the outline

// Exponential guidance force parameters:
//   A_GUIDE       = force magnitude at d = P_T (peak at zone edge)
//   C_GUIDE       = decay rate (1/m); larger = sharper drop-off near P_T
//   F_GUIDE_MAX   = saturation ceiling for guidance force; once the exp
//                   exceeds this value, F is clamped to F_GUIDE_MAX
//                   so the motors never command more than they can deliver
const float A_GUIDE     = 0.5f;     // N, peak guidance force at d = P_T
const float C_GUIDE     = 200.0f;   // 1/m; exp doubles every ~3.5 mm further out
const float F_GUIDE_MAX = 2.0f;     // N, clamp ceiling (set to ~motor force limit)

// Texture parameters (for MODE_TEXTURED):
const float A_TEX = 0.30f;          // N, texture amplitude
const float LAMBDA_TEX = 0.0020f;   // m, spatial wavelength (2 mm bumps)


// ============================================================
// SHAPE DATA
// ============================================================
// Three closed-loop shapes derived from SVG outlines by the Python
// preprocessing script (preprocess_shapes.py). Each is 200 points,
// normalized so it fits in a ~7 cm bounding box centered at the
// origin, CCW winding. Coordinates are in meters.
//
// Switch between them at runtime by changing CURRENT_SHAPE, or by
// sending a single ASCII character over Serial: 'B' (bunny),
// 'H' (hammer), 'P' (pear).  Later this command will come from
// Processing when the user clicks "Next shape" in the UI.

enum ShapeId { SHAPE_BUNNY = 0, SHAPE_HAMMER = 1, SHAPE_PEAR = 2 };
ShapeId currentShape = SHAPE_BUNNY;     // change here for testing

// All shapes happen to have the same point count for now (200), but the
// architecture allows different counts per shape if needed.
const int N_BUNNY  = 200;
const int N_HAMMER = 200;
const int N_PEAR   = 200;

// ----- bunny, 200 pts, CCW -----
const float shape_bunny[N_BUNNY][2] PROGMEM = {
    {  0.00328f,  0.03500f},
    {  0.00198f,  0.03445f},
    {  0.00007f,  0.03328f},
    { -0.00150f,  0.03201f},
    { -0.00281f,  0.03115f},
    { -0.00705f,  0.02830f},
    { -0.00908f,  0.02652f},
    { -0.01059f,  0.02515f},
    { -0.01081f,  0.02493f},
    { -0.01092f,  0.02487f},
    { -0.01101f,  0.02487f},
    { -0.01108f,  0.02487f},
    { -0.01114f,  0.02487f},
    { -0.01118f,  0.02489f},
    { -0.01122f,  0.02492f},
    { -0.01125f,  0.02496f},
    { -0.01129f,  0.02500f},
    { -0.01129f,  0.02504f},
    { -0.01123f,  0.02511f},
    { -0.01117f,  0.02531f},
    { -0.01087f,  0.02665f},
    { -0.01087f,  0.02781f},
    { -0.01117f,  0.02907f},
    { -0.01147f,  0.02955f},
    { -0.01210f,  0.03042f},
    { -0.01247f,  0.03064f},
    { -0.01395f,  0.03108f},
    { -0.01449f,  0.03113f},
    { -0.01558f,  0.03088f},
    { -0.01671f,  0.03036f},
    { -0.01714f,  0.02992f},
    { -0.01752f,  0.02924f},
    { -0.01847f,  0.02674f},
    { -0.01906f,  0.02462f},
    { -0.01966f,  0.02211f},
    { -0.01993f,  0.02162f},
    { -0.02006f,  0.02134f},
    { -0.02022f,  0.02119f},
    { -0.02046f,  0.02115f},
    { -0.02206f,  0.02115f},
    { -0.02402f,  0.02085f},
    { -0.02683f,  0.01996f},
    { -0.02883f,  0.01906f},
    { -0.02986f,  0.01810f},
    { -0.03038f,  0.01725f},
    { -0.03068f,  0.01641f},
    { -0.03083f,  0.01477f},
    { -0.03159f,  0.01251f},
    { -0.03187f,  0.01112f},
    { -0.03217f,  0.00946f},
    { -0.03232f,  0.00799f},
    { -0.03247f,  0.00617f},
    { -0.03306f,  0.00375f},
    { -0.03321f,  0.00249f},
    { -0.03306f,  0.00159f},
    { -0.03260f,  0.00043f},
    { -0.03151f, -0.00111f},
    { -0.03034f, -0.00198f},
    { -0.02858f, -0.00270f},
    { -0.02674f, -0.00313f},
    { -0.02614f, -0.00335f},
    { -0.02593f, -0.00343f},
    { -0.02580f, -0.00354f},
    { -0.02577f, -0.00372f},
    { -0.02577f, -0.00418f},
    { -0.02562f, -0.00680f},
    { -0.02497f, -0.00869f},
    { -0.02430f, -0.00981f},
    { -0.02383f, -0.01151f},
    { -0.02313f, -0.01277f},
    { -0.02179f, -0.01395f},
    { -0.02012f, -0.01576f},
    { -0.01841f, -0.01689f},
    { -0.01641f, -0.01784f},
    { -0.01470f, -0.01836f},
    { -0.01451f, -0.01847f},
    { -0.01439f, -0.01852f},
    { -0.01432f, -0.01860f},
    { -0.01430f, -0.01869f},
    { -0.01430f, -0.01879f},
    { -0.01431f, -0.01892f},
    { -0.01446f, -0.01908f},
    { -0.01503f, -0.02009f},
    { -0.01549f, -0.02075f},
    { -0.01549f, -0.02125f},
    { -0.01548f, -0.02189f},
    { -0.01519f, -0.02237f},
    { -0.01460f, -0.02308f},
    { -0.01372f, -0.02381f},
    { -0.01353f, -0.02401f},
    { -0.01343f, -0.02411f},
    { -0.01340f, -0.02421f},
    { -0.01344f, -0.02431f},
    { -0.01355f, -0.02442f},
    { -0.01390f, -0.02477f},
    { -0.01474f, -0.02563f},
    { -0.01559f, -0.02765f},
    { -0.01579f, -0.02894f},
    { -0.01571f, -0.02971f},
    { -0.01522f, -0.03080f},
    { -0.01475f, -0.03142f},
    { -0.01380f, -0.03207f},
    { -0.01329f, -0.03232f},
    { -0.01214f, -0.03247f},
    { -0.01122f, -0.03232f},
    { -0.00899f, -0.03207f},
    { -0.00771f, -0.03172f},
    { -0.00628f, -0.03142f},
    { -0.00553f, -0.03113f},
    { -0.00527f, -0.03113f},
    { -0.00502f, -0.03132f},
    { -0.00413f, -0.03206f},
    { -0.00174f, -0.03415f},
    { -0.00040f, -0.03470f},
    {  0.00098f, -0.03500f},
    {  0.00178f, -0.03485f},
    {  0.00355f, -0.03443f},
    {  0.00617f, -0.03375f},
    {  0.00746f, -0.03351f},
    {  0.00828f, -0.03321f},
    {  0.01215f, -0.03232f},
    {  0.01415f, -0.03202f},
    {  0.01695f, -0.03143f},
    {  0.01793f, -0.03098f},
    {  0.01925f, -0.02990f},
    {  0.01968f, -0.02948f},
    {  0.02098f, -0.02772f},
    {  0.02209f, -0.02602f},
    {  0.02277f, -0.02519f},
    {  0.02317f, -0.02502f},
    {  0.02375f, -0.02487f},
    {  0.02573f, -0.02457f},
    {  0.02693f, -0.02427f},
    {  0.02857f, -0.02326f},
    {  0.02999f, -0.02199f},
    {  0.03133f, -0.02095f},
    {  0.03216f, -0.01967f},
    {  0.03291f, -0.01799f},
    {  0.03321f, -0.01632f},
    {  0.03306f, -0.01461f},
    {  0.03251f, -0.01300f},
    {  0.03185f, -0.01219f},
    {  0.03077f, -0.01141f},
    {  0.02972f, -0.01102f},
    {  0.02953f, -0.01091f},
    {  0.02941f, -0.01079f},
    {  0.02934f, -0.01062f},
    {  0.02934f, -0.01020f},
    {  0.02919f, -0.00838f},
    {  0.02934f, -0.00771f},
    {  0.02934f, -0.00615f},
    {  0.02890f, -0.00443f},
    {  0.02852f, -0.00275f},
    {  0.02800f, -0.00119f},
    {  0.02607f,  0.00253f},
    {  0.02398f,  0.00502f},
    {  0.02308f,  0.00594f},
    {  0.02130f,  0.00744f},
    {  0.01875f,  0.00893f},
    {  0.01602f,  0.01013f},
    {  0.01401f,  0.01057f},
    {  0.01252f,  0.01058f},
    {  0.01129f,  0.01072f},
    {  0.00982f,  0.01056f},
    {  0.00770f,  0.01013f},
    {  0.00629f,  0.00987f},
    {  0.00442f,  0.00948f},
    {  0.00154f,  0.00869f},
    { -0.00014f,  0.00834f},
    { -0.00090f,  0.00834f},
    { -0.00293f,  0.00904f},
    { -0.00535f,  0.00981f},
    { -0.00659f,  0.00983f},
    { -0.00739f,  0.01007f},
    { -0.00770f,  0.01023f},
    { -0.00845f,  0.01113f},
    { -0.00926f,  0.01283f},
    { -0.00968f,  0.01425f},
    { -0.00968f,  0.01525f},
    { -0.00957f,  0.01560f},
    { -0.00925f,  0.01607f},
    { -0.00883f,  0.01638f},
    { -0.00692f,  0.01765f},
    { -0.00462f,  0.01905f},
    { -0.00221f,  0.02040f},
    { -0.00088f,  0.02102f},
    {  0.00012f,  0.02141f},
    {  0.00176f,  0.02220f},
    {  0.00299f,  0.02281f},
    {  0.00432f,  0.02353f},
    {  0.00556f,  0.02478f},
    {  0.00626f,  0.02581f},
    {  0.00670f,  0.02706f},
    {  0.00670f,  0.02909f},
    {  0.00626f,  0.03102f},
    {  0.00570f,  0.03272f},
    {  0.00510f,  0.03347f},
    {  0.00431f,  0.03455f},
    {  0.00397f,  0.03476f},
    {  0.00328f,  0.03500f}
};

// ----- hammer, 200 pts, CCW -----
const float shape_hammer[N_HAMMER][2] PROGMEM = {
    {  0.01861f, -0.01738f},
    {  0.01862f, -0.01740f},
    {  0.01863f, -0.01742f},
    {  0.01864f, -0.01745f},
    {  0.01867f, -0.01748f},
    {  0.01872f, -0.01751f},
    {  0.01886f, -0.01753f},
    {  0.02000f, -0.01752f},
    {  0.02049f, -0.01751f},
    {  0.02071f, -0.01746f},
    {  0.02116f, -0.01727f},
    {  0.02369f, -0.01616f},
    {  0.02512f, -0.01528f},
    {  0.02673f, -0.01401f},
    {  0.02831f, -0.01247f},
    {  0.02961f, -0.01089f},
    {  0.03057f, -0.00940f},
    {  0.03150f, -0.00754f},
    {  0.03217f, -0.00571f},
    {  0.03275f, -0.00326f},
    {  0.03315f, -0.00047f},
    {  0.03318f,  0.00067f},
    {  0.03312f,  0.00140f},
    {  0.03299f,  0.00199f},
    {  0.03278f,  0.00258f},
    {  0.03240f,  0.00334f},
    {  0.03171f,  0.00460f},
    {  0.03157f,  0.00493f},
    {  0.03152f,  0.00513f},
    {  0.03149f,  0.00551f},
    {  0.03150f,  0.00668f},
    {  0.03154f,  0.00701f},
    {  0.03162f,  0.00728f},
    {  0.03179f,  0.00768f},
    {  0.03194f,  0.00797f},
    {  0.03211f,  0.00820f},
    {  0.03234f,  0.00844f},
    {  0.03276f,  0.00878f},
    {  0.03384f,  0.00949f},
    {  0.03401f,  0.00964f},
    {  0.03413f,  0.00976f},
    {  0.03422f,  0.00990f},
    {  0.03431f,  0.01007f},
    {  0.03440f,  0.01031f},
    {  0.03452f,  0.01081f},
    {  0.03496f,  0.01395f},
    {  0.03500f,  0.01436f},
    {  0.03499f,  0.01450f},
    {  0.03496f,  0.01464f},
    {  0.03490f,  0.01483f},
    {  0.03478f,  0.01508f},
    {  0.03457f,  0.01541f},
    {  0.03426f,  0.01578f},
    {  0.03392f,  0.01610f},
    {  0.03364f,  0.01632f},
    {  0.03336f,  0.01648f},
    {  0.03304f,  0.01662f},
    {  0.03256f,  0.01675f},
    {  0.03136f,  0.01692f},
    {  0.02660f,  0.01751f},
    {  0.02614f,  0.01753f},
    {  0.02599f,  0.01752f},
    {  0.02589f,  0.01749f},
    {  0.02581f,  0.01746f},
    {  0.02573f,  0.01742f},
    {  0.02564f,  0.01736f},
    {  0.02555f,  0.01729f},
    {  0.02548f,  0.01722f},
    {  0.02542f,  0.01713f},
    {  0.02536f,  0.01701f},
    {  0.02529f,  0.01682f},
    {  0.02518f,  0.01626f},
    {  0.02468f,  0.01209f},
    {  0.02467f,  0.01161f},
    {  0.02470f,  0.01138f},
    {  0.02474f,  0.01122f},
    {  0.02479f,  0.01108f},
    {  0.02487f,  0.01093f},
    {  0.02503f,  0.01072f},
    {  0.02581f,  0.00986f},
    {  0.02615f,  0.00940f},
    {  0.02635f,  0.00904f},
    {  0.02649f,  0.00872f},
    {  0.02659f,  0.00837f},
    {  0.02665f,  0.00793f},
    {  0.02667f,  0.00749f},
    {  0.02663f,  0.00708f},
    {  0.02654f,  0.00669f},
    {  0.02641f,  0.00630f},
    {  0.02623f,  0.00592f},
    {  0.02599f,  0.00555f},
    {  0.02568f,  0.00518f},
    {  0.02538f,  0.00489f},
    {  0.02513f,  0.00470f},
    {  0.02489f,  0.00457f},
    {  0.02465f,  0.00447f},
    {  0.02435f,  0.00439f},
    {  0.02396f,  0.00433f},
    {  0.02330f,  0.00431f},
    {  0.01924f,  0.00478f},
    {  0.00341f,  0.00674f},
    { -0.00959f,  0.00836f},
    { -0.01064f,  0.00850f},
    { -0.01075f,  0.00852f},
    { -0.01082f,  0.00852f},
    { -0.01088f,  0.00853f},
    { -0.01093f,  0.00854f},
    { -0.01097f,  0.00857f},
    { -0.01098f,  0.00862f},
    { -0.01099f,  0.00868f},
    { -0.01100f,  0.00875f},
    { -0.01107f,  0.00897f},
    { -0.01114f,  0.00911f},
    { -0.01122f,  0.00921f},
    { -0.01131f,  0.00930f},
    { -0.01144f,  0.00940f},
    { -0.01161f,  0.00951f},
    { -0.01184f,  0.00961f},
    { -0.01219f,  0.00972f},
    { -0.01280f,  0.00982f},
    { -0.02603f,  0.01149f},
    { -0.03026f,  0.01199f},
    { -0.03044f,  0.01202f},
    { -0.03097f,  0.01225f},
    { -0.03139f,  0.01245f},
    { -0.03157f,  0.01249f},
    { -0.03176f,  0.01251f},
    { -0.03198f,  0.01251f},
    { -0.03225f,  0.01247f},
    { -0.03254f,  0.01239f},
    { -0.03287f,  0.01227f},
    { -0.03319f,  0.01210f},
    { -0.03347f,  0.01190f},
    { -0.03369f,  0.01170f},
    { -0.03387f,  0.01149f},
    { -0.03398f,  0.01133f},
    { -0.03405f,  0.01119f},
    { -0.03415f,  0.01088f},
    { -0.03455f,  0.00884f},
    { -0.03498f,  0.00527f},
    { -0.03500f,  0.00464f},
    { -0.03498f,  0.00448f},
    { -0.03492f,  0.00425f},
    { -0.03481f,  0.00396f},
    { -0.03465f,  0.00368f},
    { -0.03449f,  0.00345f},
    { -0.03433f,  0.00327f},
    { -0.03415f,  0.00312f},
    { -0.03396f,  0.00299f},
    { -0.03376f,  0.00290f},
    { -0.03355f,  0.00282f},
    { -0.03332f,  0.00277f},
    { -0.03299f,  0.00275f},
    { -0.03152f,  0.00275f},
    { -0.02771f,  0.00232f},
    { -0.01427f,  0.00067f},
    { -0.01329f,  0.00060f},
    { -0.01306f,  0.00061f},
    { -0.01282f,  0.00065f},
    { -0.01263f,  0.00071f},
    { -0.01247f,  0.00078f},
    { -0.01234f,  0.00087f},
    { -0.01220f,  0.00099f},
    { -0.01196f,  0.00128f},
    { -0.01189f,  0.00139f},
    { -0.01184f,  0.00146f},
    { -0.01181f,  0.00151f},
    { -0.01176f,  0.00153f},
    { -0.01170f,  0.00152f},
    { -0.01164f,  0.00151f},
    { -0.01155f,  0.00150f},
    { -0.01141f,  0.00148f},
    { -0.00468f,  0.00062f},
    {  0.00954f, -0.00116f},
    {  0.02021f, -0.00251f},
    {  0.02112f, -0.00271f},
    {  0.02165f, -0.00289f},
    {  0.02207f, -0.00310f},
    {  0.02245f, -0.00335f},
    {  0.02288f, -0.00371f},
    {  0.02328f, -0.00413f},
    {  0.02358f, -0.00455f},
    {  0.02383f, -0.00499f},
    {  0.02402f, -0.00545f},
    {  0.02416f, -0.00594f},
    {  0.02423f, -0.00644f},
    {  0.02425f, -0.00698f},
    {  0.02420f, -0.00746f},
    {  0.02407f, -0.00809f},
    {  0.02373f, -0.00906f},
    {  0.02302f, -0.01054f},
    {  0.02175f, -0.01258f},
    {  0.01997f, -0.01484f},
    {  0.01887f, -0.01610f},
    {  0.01873f, -0.01631f},
    {  0.01866f, -0.01642f},
    {  0.01862f, -0.01657f},
    {  0.01858f, -0.01676f},
    {  0.01857f, -0.01703f},
    {  0.01861f, -0.01738f}
};

// ----- pear, 200 pts, CCW -----
const float shape_pear[N_PEAR][2] PROGMEM = {
    { -0.00260f,  0.03430f},
    { -0.00341f,  0.03469f},
    { -0.00377f,  0.03482f},
    { -0.00411f,  0.03490f},
    { -0.00453f,  0.03496f},
    { -0.00528f,  0.03500f},
    { -0.01129f,  0.03475f},
    { -0.01401f,  0.03468f},
    { -0.01441f,  0.03464f},
    { -0.01462f,  0.03460f},
    { -0.01476f,  0.03455f},
    { -0.01485f,  0.03451f},
    { -0.01493f,  0.03448f},
    { -0.01499f,  0.03444f},
    { -0.01503f,  0.03440f},
    { -0.01507f,  0.03436f},
    { -0.01511f,  0.03432f},
    { -0.01514f,  0.03428f},
    { -0.01516f,  0.03423f},
    { -0.01519f,  0.03418f},
    { -0.01521f,  0.03412f},
    { -0.01522f,  0.03405f},
    { -0.01523f,  0.03397f},
    { -0.01523f,  0.03387f},
    { -0.01522f,  0.03373f},
    { -0.01519f,  0.03353f},
    { -0.01510f,  0.03319f},
    { -0.01481f,  0.03243f},
    { -0.01305f,  0.02866f},
    { -0.01240f,  0.02751f},
    { -0.01193f,  0.02685f},
    { -0.01127f,  0.02609f},
    { -0.01059f,  0.02542f},
    { -0.00997f,  0.02492f},
    { -0.00935f,  0.02450f},
    { -0.00871f,  0.02416f},
    { -0.00807f,  0.02389f},
    { -0.00744f,  0.02369f},
    { -0.00680f,  0.02355f},
    { -0.00617f,  0.02348f},
    { -0.00555f,  0.02346f},
    { -0.00494f,  0.02350f},
    { -0.00435f,  0.02359f},
    { -0.00376f,  0.02374f},
    { -0.00319f,  0.02395f},
    { -0.00263f,  0.02421f},
    { -0.00209f,  0.02453f},
    { -0.00111f,  0.02522f},
    { -0.00076f,  0.02542f},
    { -0.00056f,  0.02552f},
    { -0.00042f,  0.02557f},
    { -0.00032f,  0.02559f},
    { -0.00023f,  0.02561f},
    { -0.00016f,  0.02561f},
    { -0.00010f,  0.02561f},
    { -0.00005f,  0.02560f},
    {  0.00000f,  0.02559f},
    {  0.00005f,  0.02557f},
    {  0.00010f,  0.02555f},
    {  0.00014f,  0.02552f},
    {  0.00018f,  0.02549f},
    {  0.00023f,  0.02545f},
    {  0.00028f,  0.02540f},
    {  0.00033f,  0.02534f},
    {  0.00039f,  0.02525f},
    {  0.00045f,  0.02512f},
    {  0.00053f,  0.02492f},
    {  0.00064f,  0.02455f},
    {  0.00079f,  0.02375f},
    {  0.00085f,  0.02332f},
    {  0.00085f,  0.02321f},
    {  0.00085f,  0.02315f},
    {  0.00084f,  0.02311f},
    {  0.00082f,  0.02307f},
    {  0.00081f,  0.02304f},
    {  0.00079f,  0.02301f},
    {  0.00077f,  0.02298f},
    {  0.00075f,  0.02295f},
    {  0.00072f,  0.02292f},
    {  0.00068f,  0.02289f},
    {  0.00062f,  0.02285f},
    {  0.00054f,  0.02281f},
    {  0.00042f,  0.02277f},
    {  0.00019f,  0.02270f},
    { -0.00070f,  0.02257f},
    { -0.00157f,  0.02241f},
    { -0.00224f,  0.02222f},
    { -0.00290f,  0.02195f},
    { -0.00358f,  0.02160f},
    { -0.00429f,  0.02116f},
    { -0.00504f,  0.02058f},
    { -0.00584f,  0.01983f},
    { -0.00672f,  0.01885f},
    { -0.00769f,  0.01754f},
    { -0.00879f,  0.01575f},
    { -0.01004f,  0.01322f},
    { -0.01149f,  0.00954f},
    { -0.01266f,  0.00631f},
    { -0.01349f,  0.00460f},
    { -0.01505f,  0.00202f},
    { -0.01927f, -0.00416f},
    { -0.02083f, -0.00682f},
    { -0.02176f, -0.00879f},
    { -0.02237f, -0.01046f},
    { -0.02278f, -0.01201f},
    { -0.02304f, -0.01356f},
    { -0.02317f, -0.01525f},
    { -0.02315f, -0.01707f},
    { -0.02299f, -0.01870f},
    { -0.02267f, -0.02030f},
    { -0.02222f, -0.02187f},
    { -0.02162f, -0.02340f},
    { -0.02088f, -0.02488f},
    { -0.01999f, -0.02631f},
    { -0.01897f, -0.02767f},
    { -0.01781f, -0.02895f},
    { -0.01653f, -0.03014f},
    { -0.01511f, -0.03122f},
    { -0.01359f, -0.03219f},
    { -0.01195f, -0.03304f},
    { -0.01021f, -0.03374f},
    { -0.00846f, -0.03428f},
    { -0.00747f, -0.03448f},
    { -0.00439f, -0.03480f},
    {  0.00239f, -0.03500f},
    {  0.00669f, -0.03483f},
    {  0.00801f, -0.03466f},
    {  0.00902f, -0.03441f},
    {  0.01060f, -0.03389f},
    {  0.01221f, -0.03318f},
    {  0.01381f, -0.03231f},
    {  0.01536f, -0.03126f},
    {  0.01682f, -0.03008f},
    {  0.01816f, -0.02878f},
    {  0.01933f, -0.02742f},
    {  0.02032f, -0.02602f},
    {  0.02114f, -0.02463f},
    {  0.02178f, -0.02327f},
    {  0.02226f, -0.02195f},
    {  0.02277f, -0.02000f},
    {  0.02306f, -0.01830f},
    {  0.02317f, -0.01680f},
    {  0.02315f, -0.01531f},
    {  0.02298f, -0.01371f},
    {  0.02260f, -0.01177f},
    {  0.02181f, -0.00908f},
    {  0.01996f, -0.00436f},
    {  0.01721f,  0.00200f},
    {  0.01520f,  0.00772f},
    {  0.01368f,  0.01217f},
    {  0.01251f,  0.01483f},
    {  0.01155f,  0.01657f},
    {  0.01074f,  0.01776f},
    {  0.01004f,  0.01861f},
    {  0.00943f,  0.01923f},
    {  0.00888f,  0.01969f},
    {  0.00838f,  0.02004f},
    {  0.00791f,  0.02030f},
    {  0.00745f,  0.02050f},
    {  0.00701f,  0.02065f},
    {  0.00634f,  0.02079f},
    {  0.00587f,  0.02090f},
    {  0.00564f,  0.02099f},
    {  0.00549f,  0.02105f},
    {  0.00538f,  0.02112f},
    {  0.00530f,  0.02118f},
    {  0.00523f,  0.02124f},
    {  0.00517f,  0.02130f},
    {  0.00512f,  0.02136f},
    {  0.00508f,  0.02143f},
    {  0.00504f,  0.02152f},
    {  0.00498f,  0.02167f},
    {  0.00487f,  0.02226f},
    {  0.00393f,  0.02751f},
    {  0.00380f,  0.02806f},
    {  0.00365f,  0.02849f},
    {  0.00347f,  0.02888f},
    {  0.00328f,  0.02921f},
    {  0.00309f,  0.02949f},
    {  0.00290f,  0.02971f},
    {  0.00272f,  0.02989f},
    {  0.00254f,  0.03002f},
    {  0.00238f,  0.03013f},
    {  0.00223f,  0.03021f},
    {  0.00208f,  0.03027f},
    {  0.00193f,  0.03032f},
    {  0.00161f,  0.03036f},
    {  0.00146f,  0.03039f},
    {  0.00136f,  0.03042f},
    {  0.00126f,  0.03047f},
    {  0.00115f,  0.03054f},
    {  0.00100f,  0.03064f},
    {  0.00081f,  0.03080f},
    {  0.00056f,  0.03107f},
    {  0.00018f,  0.03156f},
    { -0.00047f,  0.03249f},
    { -0.00094f,  0.03302f},
    { -0.00145f,  0.03350f},
    { -0.00197f,  0.03390f},
    { -0.00260f,  0.03430f}
};

// Active shape: a pointer to the current PROGMEM array + its length.
// Shape data lives in flash (PROGMEM) to fit on the ATmega328's small
// RAM (only 2 KB; all three shapes together would be ~4.8 KB if in RAM).
// Access via shapeX(i) / shapeY(i) helpers below, NOT through direct
// indexing -- AVR flash and RAM are on separate buses.
const float (*shape)[2] = shape_bunny;
int N_POINTS = N_BUNNY;

// Read x or y of the i-th vertex of the active shape, from PROGMEM.
inline float shapeX(int i) {
  return pgm_read_float(&shape[i][0]);
}
inline float shapeY(int i) {
  return pgm_read_float(&shape[i][1]);
}



// ============================================================
// PRECOMPUTED TABLES
// ============================================================
// s_arc is sized to the maximum possible point count across all shapes,
// since the active shape can be swapped at runtime. The currently-active
// portion is the first N_POINTS entries.
#define MAX_POINTS 200
float s_arc[MAX_POINTS];
float perimeter = 0.0f;
float lambda_actual = LAMBDA_TEX;


// ============================================================
// VELOCITY TRACKING  (for wall damping)
// ============================================================
// Position from the previous loop, used to compute filtered velocity.
// Same finite-difference + IIR filter pattern as A3/A4.
float xh_prev = 0.0f, yh_prev = 0.0f;
float vx_filt = 0.0f, vy_filt = 0.0f;
const float DT_LOOP = 0.001f;       // s, nominal loop period (1 kHz)


// ============================================================
// FORWARD DECLARATIONS
// ============================================================
void   getPenTipPosition(float &x, float &y);
int    findNearestSegment(float x, float y, float &dist, float &nx, float &ny,
                          float &t_frac);
bool   isInsidePolygon(float x, float y);
void   computeForce(float x, float y, float &Fx, float &Fy);
void   computeMotorTorques(float Fx, float Fy, float &tau1, float &tau5);
void   sendMotorCommand(float tau1, float tau5);
void   precomputeArcLength();
float  signedPolygonArea();
void   checkWinding();
void   setShape(ShapeId s);
void   handleSerialCommands();


// ============================================================
// SETUP
// ============================================================
// PWM frequency divisor helper (copied from A3 template -- DO NOT EDIT).
// Increases PWM frequency on the motor pins to keep motor noise inaudible
// and force-response smooth.
void setPwmFrequency(int pin, int divisor) {
  byte mode;
  if(pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if(pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    } else {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  } else if(pin == 3 || pin == 11) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 32: mode = 0x03; break;
      case 64: mode = 0x04; break;
      case 128: mode = 0x05; break;
      case 256: mode = 0x06; break;
      case 1024: mode = 0x07; break;
      default: return;
    }
    TCCR2B = TCCR2B & 0b11111000 | mode;
  }
}

void setup() {
  Serial.begin(115200);      // USB serial for debugging + user commands
  linkSerial.begin(19200);   // SoftwareSerial inter-board link (D9/D10)
                             // 19200 is the highest baud SoftwareSerial
                             // handles reliably on a 16 MHz AVR.

  // Configure motor PWM at high frequency (matches A3 template).
  setPwmFrequency(PWM_PIN_LOCAL, 1);

  // Configure motor and encoder pins.
  pinMode(sensorPosPin, INPUT);
  pinMode(PWM_PIN_LOCAL, OUTPUT);
  pinMode(DIR_PIN_LOCAL, OUTPUT);
  analogWrite(PWM_PIN_LOCAL, 0);          // motor off at startup
  digitalWrite(DIR_PIN_LOCAL, LOW);

  setShape(currentShape);  // initializes shape pointer, arc-length table, prints CCW check
}


// ============================================================
// MAIN LOOP
// ============================================================
void loop() {
  // 0. Handle any pending shape-switch or mode-toggle commands from serial.
  handleSerialCommands();

  // 1. Get current pen-tip position.
  //    Replace with forward kinematics output once FK is wired,
  //    or with serial input from Processing for mouse-driven testing.
  float xh, yh;
  getPenTipPosition(xh, yh);

  // 2. Compute filtered velocity (used for wall damping).
  //    Same IIR pattern as A3 Section 2.
  float vx = (xh - xh_prev) / DT_LOOP;
  float vy = (yh - yh_prev) / DT_LOOP;
  vx_filt = 0.9f * vx + 0.1f * vx_filt;
  vy_filt = 0.9f * vy + 0.1f * vy_filt;
  xh_prev = xh;
  yh_prev = yh;

  // 3. Compute Cartesian force at the pen tip.
  float Fx, Fy;
  computeForce(xh, yh, Fx, Fy);

  // 4. Map to motor torques via Jacobian transpose (stub for now).
  float tau1, tau5;
  computeMotorTorques(Fx, Fy, tau1, tau5);

  // 5. Send to motors via PWM (stub for now).
  sendMotorCommand(tau1, tau5);

  // 6. Stream pen-tip position back to Processing (rate-limited):
  static int printCounter = 0;
  if (++printCounter >= 30) {
    Serial.print(xh, 4); Serial.print(",");
    Serial.print(yh, 4); Serial.print(",");
    Serial.println((int)renderMode);
    printCounter = 0;
  }
}


// ============================================================
// ENCODER READING + FORWARD KINEMATICS
// ============================================================
// This board reads ONE encoder (its own motor's MR sensor on pin A2) and
// computes θ for that joint. The OTHER joint angle (θ for the other motor)
// must arrive over serial from the partner Hapkit board, or via a host
// (Processing) that polls both boards.
//
// Below, theta1_rad is the angle this board's motor; theta5_rad is the
// other board's motor (received over serial). If this is the "motor 5"
// board, swap the assignments in computeFwdKin() accordingly.
//
// Forward kinematics follows the geometric approach from the wiki:
//   1. P2 = (LINK_A1*cos(theta1), LINK_A1*sin(theta1))
//   2. P4 = (-LINK_A5 + LINK_A4*cos(theta5), LINK_A4*sin(theta5))
//   3. |P2-Pn| = (LINK_A2^2 - LINK_A3^2 + |P4-P2|^2) / (2|P4-P2|)
//   4. Ph = P2 + |P2-Pn|/|P2-P4| * (P4-P2)
//   5. |P3-Ph| = sqrt(LINK_A2^2 - |P2-Pn|^2)
//   6. P3 = Ph + |P3-Ph|/|P4-P2| * (y4-y2, -(x4-x2))    [elbow-up sign]

// --------- Encoder state (mirrors A3 template) ---------
int rawPos = 0, lastRawPos = 0, lastLastRawPos = 0;
int rawDiff = 0, lastRawDiff = 0, rawOffset = 0, lastRawOffset = 0;
int flipNumber = 0;
int tempOffset = 0;
bool flipped = false;
const int flipThresh = 700;
int updatedPos = 0;

// Partner-board angle, received over serial.
// [TUNE WHEN HARDWARE READY: pick a sensible startup angle inside the
//  valid workspace, e.g. PI/2 for the elbow-up configuration.]
float theta_partner_rad = PI / 2.0f;

// Last computed pen-tip position (persists between loops when getPenTip
// has bad/no data this iteration).
float xh_persist = 0.0f, yh_persist = 0.0f;

// --------- Read this board's encoder (returns theta in radians) ---------
float readThisBoardTheta() {
  rawPos = analogRead(sensorPosPin);
  rawDiff = rawPos - lastRawPos;
  lastRawDiff = rawPos - lastLastRawPos;
  rawOffset = abs(rawDiff);
  lastRawOffset = abs(lastRawDiff);
  lastLastRawPos = lastRawPos;
  lastRawPos = rawPos;

  if ((lastRawOffset > flipThresh) && (!flipped)) {
    if (lastRawDiff > 0) flipNumber--; else flipNumber++;
    if (rawOffset > flipThresh) {
      updatedPos = rawPos + flipNumber * rawOffset;
      tempOffset = rawOffset;
    } else {
      updatedPos = rawPos + flipNumber * lastRawOffset;
      tempOffset = lastRawOffset;
    }
    flipped = true;
  } else {
    updatedPos = rawPos + flipNumber * tempOffset;
    flipped = false;
  }

  float ts_deg = ENC_M * updatedPos + ENC_B;            // sector angle (deg)
  float theta_motor_deg = ts_deg * SECTOR_GEAR_REDUCTION;
  float theta_motor_rad = theta_motor_deg * (PI / 180.0f);
  return theta_motor_rad;
}

// --------- Geometric forward kinematics ---------
// Returns true if a valid configuration was found, false otherwise (e.g.
// linkage is in a singular pose).
bool computeFwdKin(float theta1, float theta5, float &x3, float &y3) {
  float x2 = LINK_A1 * cosf(theta1);
  float y2 = LINK_A1 * sinf(theta1);
  float x4 = -LINK_A5 + LINK_A4 * cosf(theta5);
  float y4 = LINK_A4 * sinf(theta5);

  float dx = x4 - x2;
  float dy = y4 - y2;
  float dP2P4 = sqrtf(dx*dx + dy*dy);
  if (dP2P4 < 1e-6f) return false;  // singular: elbows coincident

  // |P2 - Ph|
  float dP2Ph = (LINK_A2*LINK_A2 - LINK_A3*LINK_A3 + dP2P4*dP2P4) / (2.0f * dP2P4);

  // |P3 - Ph| (altitude); guard against floating-point negative
  float h2 = LINK_A2*LINK_A2 - dP2Ph*dP2Ph;
  if (h2 < 0.0f) return false;      // unreachable
  float dP3Ph = sqrtf(h2);

  // Ph coordinates
  float xh_pt = x2 + (dP2Ph / dP2P4) * dx;
  float yh_pt = y2 + (dP2Ph / dP2P4) * dy;

  // Step perpendicular to P2->P4 by dP3Ph. For "elbow up" (positive y3),
  // sign convention is (y4 - y2, -(x4 - x2)) with a + sign on x3 and a -
  // sign on y3 in the formula -- but the actual signs depend on which
  // half-plane the device operates in. Adjust if y3 comes out negative
  // when the linkage is in the working pose.
  x3 = xh_pt + (dP3Ph / dP2P4) * dy;
  y3 = yh_pt - (dP3Ph / dP2P4) * dx;
  return true;
}

// --------- Public function the main loop calls ---------
void getPenTipPosition(float &x, float &y) {
  float theta_self = readThisBoardTheta();

  // Assign theta1 and theta5 based on which board this is.
#ifdef IS_MOTOR_1
  float theta1 = theta_self;
  float theta5 = theta_partner_rad;
#else
  float theta1 = theta_partner_rad;
  float theta5 = theta_self;
#endif

  float x3, y3;
  if (computeFwdKin(theta1, theta5, x3, y3)) {
    xh_persist = x3;
    yh_persist = y3;
  }
  // If computeFwdKin failed (singular/unreachable), reuse last good value.
  x = xh_persist;
  y = yh_persist;
}


// ============================================================
// PRECOMPUTE ARC-LENGTH TABLE
// ============================================================
// Walks the (closed) shape once to fill s_arc[i] and total perimeter.
// Snaps lambda_actual to perimeter/round(perimeter/LAMBDA_TEX) so the
// sine wave closes cleanly at the seam where shape[N-1] -> shape[0].
void precomputeArcLength() {
  s_arc[0] = 0.0f;
  for (int i = 1; i < N_POINTS; i++) {
    float dx = shapeX(i) - shapeX(i-1);
    float dy = shapeY(i) - shapeY(i-1);
    s_arc[i] = s_arc[i-1] + sqrtf(dx*dx + dy*dy);
  }
  float dx_close = shapeX(0) - shapeX(N_POINTS-1);
  float dy_close = shapeY(0) - shapeY(N_POINTS-1);
  perimeter = s_arc[N_POINTS-1] + sqrtf(dx_close*dx_close + dy_close*dy_close);

  int n_cycles = (int)roundf(perimeter / LAMBDA_TEX);
  if (n_cycles < 1) n_cycles = 1;
  lambda_actual = perimeter / (float)n_cycles;
}


// ============================================================
// SIGNED POLYGON AREA  (shoelace formula)
// ============================================================
// Positive area means CCW winding (interior on the left as you walk
// the array forward). Negative means CW. Used to verify the shape
// array has the orientation the rest of the code assumes.
float signedPolygonArea() {
  float area = 0.0f;
  for (int i = 0; i < N_POINTS; i++) {
    int j = (i + 1) % N_POINTS;
    area += shapeX(i) * shapeY(j) - shapeX(j) * shapeY(i);
  }
  return 0.5f * area;
}


// ============================================================
// WINDING CHECK  (warn over serial if CW)
// ============================================================
void checkWinding() {
  float area = signedPolygonArea();
  if (area < 0.0f) {
    Serial.println("!!! WARNING: shape array is CW. Force normals will point");
    Serial.println("!!! INWARD. Reverse the array order in Python (or here)");
    Serial.println("!!! before testing.");
  } else {
    Serial.print("Shape OK (CCW), area = ");
    Serial.print(area, 6);
    Serial.println(" m^2");
  }
}


// ============================================================
// SET SHAPE  (runtime switcher)
// ============================================================
// Swaps the active shape pointer + length, then rebuilds the arc-length
// table and verifies winding. Called once in setup() and again whenever
// a shape-switch command arrives over serial.
void setShape(ShapeId s) {
  currentShape = s;
  switch (s) {
    case SHAPE_BUNNY:
      shape = shape_bunny;
      N_POINTS = N_BUNNY;
      Serial.println("Active shape: BUNNY");
      break;
    case SHAPE_HAMMER:
      shape = shape_hammer;
      N_POINTS = N_HAMMER;
      Serial.println("Active shape: HAMMER");
      break;
    case SHAPE_PEAR:
      shape = shape_pear;
      N_POINTS = N_PEAR;
      Serial.println("Active shape: PEAR");
      break;
  }
  precomputeArcLength();
  checkWinding();
}


// ============================================================
// HANDLE SERIAL COMMANDS  (called once per loop)
// ============================================================
// Two serial streams are processed:
//
// 1. USB Serial (hardware UART, D0/D1):
//    Single-character commands from the Serial Monitor or host PC:
//      'B' / 'b' -> switch to bunny
//      'H' / 'h' -> switch to hammer
//      'P' / 'p' -> switch to pear
//      'C' / 'c' -> control mode (no texture)
//      'T' / 't' -> textured mode
//
// 2. linkSerial (SoftwareSerial, D9 TX / D10 RX):
//    Multi-character commands from the partner board:
//      "A<float>\n" -> set theta_partner_rad (joint angle from partner)
//
// Whitespace and unrecognized bytes are silently consumed.
void handleSerialCommands() {
  // ----- USB Serial: user/host commands -----
  while (Serial.available()) {
    char c = (char)Serial.read();
    switch (c) {
      case 'B': case 'b': setShape(SHAPE_BUNNY);  break;
      case 'H': case 'h': setShape(SHAPE_HAMMER); break;
      case 'P': case 'p': setShape(SHAPE_PEAR);   break;
      case 'C': case 'c':
        renderMode = MODE_CONTROL;
        Serial.println("Mode: CONTROL");
        break;
      case 'T': case 't':
        renderMode = MODE_TEXTURED;
        Serial.println("Mode: TEXTURED");
        break;
      default:
        break;
    }
  }

  // ----- linkSerial: inter-board partner theta -----
  while (linkSerial.available()) {
    char c = (char)linkSerial.read();
    if (c == 'A') {
      float val = linkSerial.parseFloat();
      theta_partner_rad = val;
      // Consume the trailing newline if present
      while (linkSerial.available() &&
             (linkSerial.peek() == '\n' || linkSerial.peek() == '\r')) {
        linkSerial.read();
      }
    }
    // Anything else: ignore.
  }
}


// ============================================================
// NEAREST SEGMENT SEARCH
// ============================================================
// For each segment (shape[i], shape[i+1])  (wrapping at N-1 -> 0),
// computes the perpendicular distance from (x,y) to that segment,
// the segment's outward unit normal (assuming CCW polygon), and the
// fractional position t in [0,1] along the segment.
//
// Outward normal: for tangent (tx,ty), outward direction is (-ty, tx)
// when the polygon is CCW.
int findNearestSegment(float x, float y, float &dist, float &nx, float &ny,
                       float &t_frac) {
  int best_i = 0;
  float best_dist = 1.0e9f;
  float best_nx = 0.0f, best_ny = 0.0f;
  float best_t = 0.0f;

  for (int i = 0; i < N_POINTS; i++) {
    int j = (i + 1) % N_POINTS;

    float ax = shapeX(i), ay = shapeY(i);
    float bx = shapeX(j), by = shapeY(j);
    float tx = bx - ax;
    float ty = by - ay;
    float seg_len2 = tx*tx + ty*ty;

    float t = ((x - ax)*tx + (y - ay)*ty) / seg_len2;
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;

    float px = ax + t*tx;
    float py = ay + t*ty;
    float ddx = x - px;
    float ddy = y - py;
    float d = sqrtf(ddx*ddx + ddy*ddy);

    if (d < best_dist) {
      best_dist = d;
      best_i = i;
      best_t = t;
      float seg_len = sqrtf(seg_len2);
      // Outward normal for a CCW polygon in math (y-up) coords:
      // rotate tangent -90 deg, i.e. (tx, ty) -> (ty, -tx).
      best_nx =  ty / seg_len;
      best_ny = -tx / seg_len;
    }
  }

  dist = best_dist;
  nx = best_nx;
  ny = best_ny;
  t_frac = best_t;
  return best_i;
}


// ============================================================
// INSIDE/OUTSIDE TEST  (ray casting)
// ============================================================
bool isInsidePolygon(float x, float y) {
  bool inside = false;
  for (int i = 0, j = N_POINTS - 1; i < N_POINTS; j = i++) {
    float xi = shapeX(i), yi = shapeY(i);
    float xj = shapeX(j), yj = shapeY(j);

    bool intersect = ((yi > y) != (yj > y)) &&
                     (x < (xj - xi) * (y - yi) / (yj - yi) + xi);
    if (intersect) inside = !inside;
  }
  return inside;
}


// ============================================================
// FORCE FEEDBACK MODEL
// ============================================================
// Uses SIGNED penetration depth (positive = inside, negative = outside).
//
// Three regions:
//   d > 0:         F_wall = -K_WALL * d * n_outward          (linear spring)
//                  + B_WALL * v_normal * (-n_outward)        (damping, only
//                                                              when moving in)
//                  + texture (textured mode only)
//   P_T < d <= 0:  F = 0                                     (dead zone)
//   d <= P_T:      F = A_GUIDE * exp(-C_GUIDE * (d - P_T))
//                       * (-n_outward)                       (pulls toward shape)
//
// Note: n_outward is the unit outward normal of the nearest segment.
// "Pushing the user outward from inside" = force along +n_outward,
// "Pulling the user inward from outside" = force along -n_outward.
//
// The form -K_WALL*d (with d > 0 inside) produces a magnitude K_WALL*|d|
// in the outward direction, consistent with the original Chiara diagram.
void computeForce(float x, float y, float &Fx, float &Fy) {
  // Find nearest segment and unsigned perpendicular distance.
  float unsigned_d, nx, ny, t_frac;
  int seg_i = findNearestSegment(x, y, unsigned_d, nx, ny, t_frac);

  // Convert to SIGNED penetration depth: positive inside, negative outside.
  bool inside = isInsidePolygon(x, y);
  float d = inside ? unsigned_d : -unsigned_d;

  // Velocity in the inward direction (positive = moving deeper into shape).
  // n_outward points outward, so inward velocity = -(v . n_outward).
  float v_inward = -(vx_filt * nx + vy_filt * ny);

  Fx = 0.0f;
  Fy = 0.0f;

  if (d > 0.0f) {
    // ---------- INSIDE the shape: linear wall spring (+ damping + texture)
    float F_wall_mag = K_WALL * d;  // magnitude in the outward-normal direction

    // Wall damping: opposes motion only when moving INTO the wall.
    // Avoids "sticky pull-out" when user retracts.
    if (v_inward > 0.0f) {
      F_wall_mag += B_WALL * v_inward;
    }

    Fx = F_wall_mag * nx;
    Fy = F_wall_mag * ny;

    // Texture force (textured condition only):
    if (renderMode == MODE_TEXTURED) {
      int next_i = (seg_i + 1) % N_POINTS;
      float seg_len = (next_i == 0)
                      ? (perimeter - s_arc[seg_i])
                      : (s_arc[next_i] - s_arc[seg_i]);
      float s = s_arc[seg_i] + t_frac * seg_len;

      float F_tex = A_TEX * sinf(2.0f * (float)PI * s / lambda_actual);
      Fx += F_tex * nx;
      Fy += F_tex * ny;
    }

  } else if (d > P_T) {
    // ---------- DEAD ZONE: no force, user can move freely near the outline
    Fx = 0.0f;
    Fy = 0.0f;

  } else {
    // ---------- FAR OUTSIDE: exponential guidance pulling toward the shape
    // F = A_GUIDE * exp(-C_GUIDE * (d - P_T))
    //   At d = P_T:   F = A_GUIDE  (peak guidance at dead-zone edge)
    //   At d < P_T:   F grows exponentially as user goes further out
    //   Clamped at F_GUIDE_MAX so the motors never have to deliver more
    //   force than they can handle.
    float F_guide = A_GUIDE * expf(-C_GUIDE * (d - P_T));
    if (F_guide > F_GUIDE_MAX) F_guide = F_GUIDE_MAX;
    // Sign flip: pull INWARD, opposite to outward normal.
    Fx = -F_guide * nx;
    Fy = -F_guide * ny;
  }
}


// ============================================================
// JACOBIAN TRANSPOSE  (Cartesian force -> motor torques)
// ============================================================
// Implements tau = J^T * F where J is the 2x2 Jacobian of the forward
// kinematics map. Follows the chain-rule expressions in Campion et al.
// 2005 (Pantograph Mk-II) with the corrected signs on the leaf partials
// of P2 and P4 (see the wiki Differential Kinematics section -- the
// paper's Eqs. (14)-(15) are missing negative signs on dx2/dt1 and
// dx4/dt5).
//
// Conventions:
//   theta1 = this board's motor angle (or whichever is "motor 1")
//   theta5 = partner board's motor angle
//   theta1 and theta5 are in RADIANS.
//
// Intermediate quantities (matching Campion's notation):
//   d = |P2 - P4|
//   b = |P2 - Ph|
//   h = |P3 - Ph|
//
// Indices throughout: subscript "1" = partial wrt theta1, "5" = wrt theta5.
// For brevity we compute the 8 entries we need: dx2/dt1, dy2/dt1, dx4/dt5,
// dy4/dt5, then chain through d, b, Ph, h, to x3, y3.
//
// The four partials d_i x3, d_i y3 (i in {1,5}) form the 2x2 J. J^T * F
// = [d1x3*Fx + d1y3*Fy ; d5x3*Fx + d5y3*Fy] = (tau1, tau5).
void computeMotorTorques(float Fx, float Fy, float &tau1, float &tau5) {
  // We need the current pen-tip position state for this calculation, so
  // we recompute the geometry locally here. Could be optimized by caching
  // from the previous getPenTipPosition call, but cleaner to keep
  // independent for now.
  float theta_self = readThisBoardTheta();
#ifdef IS_MOTOR_1
  float theta1 = theta_self;
  float theta5 = theta_partner_rad;
#else
  float theta1 = theta_partner_rad;
  float theta5 = theta_self;
#endif

  // Elbow positions
  float x2 = LINK_A1 * cosf(theta1);
  float y2 = LINK_A1 * sinf(theta1);
  float x4 = -LINK_A5 + LINK_A4 * cosf(theta5);
  float y4 = LINK_A4 * sinf(theta5);

  float dx = x4 - x2;
  float dy = y4 - y2;
  float d  = sqrtf(dx*dx + dy*dy);
  if (d < 1e-6f) { tau1 = 0; tau5 = 0; return; }

  // b = |P2 - Ph|;  h = |P3 - Ph|
  float b  = (LINK_A2*LINK_A2 - LINK_A3*LINK_A3 + d*d) / (2.0f * d);
  float h2 = LINK_A2*LINK_A2 - b*b;
  if (h2 < 0.0f) { tau1 = 0; tau5 = 0; return; }
  float h  = sqrtf(h2);

  // ---------------------------------------------------------
  // Leaf partials (CORRECTED -- Campion paper has wrong signs
  // on the x-components of these two lines)
  // ---------------------------------------------------------
  float d1x2 = -LINK_A1 * sinf(theta1);   // partial of x2 wrt theta1
  float d1y2 =  LINK_A1 * cosf(theta1);
  float d5x4 = -LINK_A4 * sinf(theta5);   // partial of x4 wrt theta5
  float d5y4 =  LINK_A4 * cosf(theta5);

  // The "other-index" partials are zero (e.g. P2 doesn't depend on theta5):
  // d5x2 = d5y2 = d1x4 = d1y4 = 0
  // (no variables, just substituted as 0 below)

  // ---------------------------------------------------------
  // Chain rule, for index i = 1 then i = 5.
  // ---------------------------------------------------------
  // We compute two columns of the Jacobian: [d1x3, d1y3] and [d5x3, d5y3].
  float d1x3, d1y3, d5x3, d5y3;

  for (int pass = 0; pass < 2; pass++) {
    // i = 1 on pass 0, i = 5 on pass 1
    float dix2, diy2, dix4, diy4;
    if (pass == 0) {
      dix2 = d1x2; diy2 = d1y2;
      dix4 = 0.0f; diy4 = 0.0f;
    } else {
      dix2 = 0.0f; diy2 = 0.0f;
      dix4 = d5x4; diy4 = d5y4;
    }

    // d_i d  = ((x4-x2)*(dix4 - dix2) + (y4-y2)*(diy4 - diy2)) / d
    float did = ((x4 - x2)*(dix4 - dix2) + (y4 - y2)*(diy4 - diy2)) / d;

    // d_i b = d_i d - d_i d * (LINK_A2^2 - LINK_A3^2 + d^2) / (2 d^2)
    float dib = did - did * (LINK_A2*LINK_A2 - LINK_A3*LINK_A3 + d*d) / (2.0f * d*d);

    // d_i h = -b/h * d_i b
    float dih = -b * dib / h;

    // d_i yh = d_i y2 + ((dib*d - did*b)/d^2)(y4-y2) + (b/d)(diy4 - diy2)
    float diyh = diy2 + ((dib*d - did*b) / (d*d)) * (y4 - y2)
                 + (b/d) * (diy4 - diy2);

    // d_i xh = d_i x2 + ((dib*d - did*b)/d^2)(x4-x2) + (b/d)(dix4 - dix2)
    float dixh = dix2 + ((dib*d - did*b) / (d*d)) * (x4 - x2)
                 + (b/d) * (dix4 - dix2);

    // d_i y3 = d_i yh - (h/d)(dix4 - dix2) - ((dih*d - did*h)/d^2)(x4-x2)
    float diy3 = diyh - (h/d)*(dix4 - dix2)
                 - ((dih*d - did*h)/(d*d))*(x4 - x2);

    // d_i x3 = d_i xh + (h/d)(diy4 - diy2) + ((dih*d - did*h)/d^2)(y4-y2)
    float dix3 = dixh + (h/d)*(diy4 - diy2)
                 + ((dih*d - did*h)/(d*d))*(y4 - y2);

    if (pass == 0) { d1x3 = dix3; d1y3 = diy3; }
    else           { d5x3 = dix3; d5y3 = diy3; }
  }

  // ---------------------------------------------------------
  // tau = J^T * F
  //   tau1 = d_x3/d_t1 * Fx + d_y3/d_t1 * Fy
  //   tau5 = d_x3/d_t5 * Fx + d_y3/d_t5 * Fy
  // ---------------------------------------------------------
  tau1 = d1x3 * Fx + d1y3 * Fy;
  tau5 = d5x3 * Fx + d5y3 * Fy;
}


// ============================================================
// MOTOR COMMAND OUTPUT
// ============================================================
// Pin numbers, calibration constant, and motor-direction polarity were
// declared at the top of the file. Modify there.

// Write one motor (torque -> duty cycle + direction).
// Input `tau` is the torque at the sector pulley (which the Jacobian
// gives us). We convert to motor-pulley torque via the capstan ratio
// before mapping to PWM, since the PWM-vs-torque relation was calibrated
// at the motor pulley in A3.
void writeMotor(int pwmPin, int dirPin, float tau_sector) {
  float tau_motor = tau_sector * CAPSTAN_RATIO;

  // Direction: positive torque = HIGH, negative = LOW (A3 convention;
  // may need to be inverted depending on motor wiring polarity).
  // [TUNE WHEN HARDWARE READY: flip HIGH/LOW if motor pushes the wrong way]
  digitalWrite(dirPin, (tau_motor >= 0.0f) ? HIGH : LOW);

  float duty = sqrtf(fabsf(tau_motor) / TORQUE_TO_DUTY_K);
  if (duty > 1.0f) duty = 1.0f;
  if (duty < 0.0f) duty = 0.0f;
  int outVal = (int)(duty * 255.0f);
  analogWrite(pwmPin, outVal);
}

void sendMotorCommand(float tau1, float tau5) {
  // Each board writes ITS OWN motor only. The other board independently
  // computes the same torque values and writes its own motor.
#ifdef IS_MOTOR_1
  writeMotor(PWM_PIN_LOCAL, DIR_PIN_LOCAL, tau1);
#else
  writeMotor(PWM_PIN_LOCAL, DIR_PIN_LOCAL, tau5);
#endif

  // Broadcast OUR theta to the partner board over the inter-board UART.
  // Protocol: "A<value>\n" with value in radians. Rate-limit to avoid
  // saturating the SoftwareSerial bandwidth (it's ~9600-19200 baud
  // reliable, much slower than the hardware UART).
  static int thetaCounter = 0;
  if (++thetaCounter >= 5) {
    linkSerial.print("A");
    linkSerial.println(readThisBoardTheta(), 5);
    thetaCounter = 0;
  }
}
