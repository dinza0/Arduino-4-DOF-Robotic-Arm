// MAE C163A/C263A Project
// Team X

// ====== CONFIG ======
const int DXL_DIR_PIN = 2;        // MAX485 DE/RE connected to pin 2
const long DXL_BAUD   = 57600;    // Bus speed for Dynamixel

const byte NUM_JOINTS = 5;
// IDs along the arm: (example mapping)
byte jointIDs[NUM_JOINTS] = {
  4,  // θ1 (base yaw)  - MX-28
  3,  // θ2 (shoulder)  - MX-28
  2,  // θ3 (elbow)     - MX-28
  1,  // θ4 (wrist)     - MX-28
  5   // θ5 (gripper)   - XL-320
};


// ====== GEOMETRY (meters) ======
const float d1 = 0.0386f;   // 38.6 mm
const float a2 = 0.14287f;  // 142.87 mm
const float a3 = 0.14287f;  // 142.87 mm
const float a4 = 0.115f;    // 115 mm

// Joint mechanical offsets (servo angle = math angle + offset)
const float JOINT_OFFSET[4] = {
  PI/18,   // joint 1 (base) – no offset
  0.0f,     // joint 2 (shoulder) – 180°
  0.0f,     // joint 3 (elbow)    – 180°
  0.0f,     // joint 4 (wrist)    – 180°
};

const float direction_offset[4] = {
  1,   // joint 1 (base) – no offset
  1,     // joint 2 (shoulder) – 180°
  -1,     // joint 3 (elbow)    – 180°
  1,     // joint 4 (wrist)    – 180°
};



// ====== POSE STRUCT ======
struct Pose {
  float px, py, pz;   // position
  float r13, r23;     // rotation matrix entries
  float r31, r32;
};

// Build the desired pickup pose in base coordinates
Pose buildPickupPose() {
  Pose P;

  // Target position in meters (from base frame)
  P.px = 0.380f;   // 127 mm
  P.py = 0.0f;
  P.pz = 0.010f;   // 10 mm

  // Base facing along +x (θ1_des = 0)
  float t1_des = 0.0f;
  P.r13 = sin(t1_des);
  P.r23 = -cos(t1_des);

  // Wrist chain orientation θ234 = 0 (roughly horizontal)
  float theta234_des = 0.0f;
  P.r31 = sin(theta234_des);
  P.r32 = cos(theta234_des);

  return P;
}

// ====== INVERSE KINEMATICS (θ1..θ4) FOR YOUR T0_5 ======
bool solveIK(const Pose &Pdes,
             float &t1, float &t2, float &t3, float &t4)
{
  float r13 = Pdes.r13;
  float r23 = Pdes.r23;
  float r31 = Pdes.r31;
  float r32 = Pdes.r32;

  float px  = Pdes.px;
  float py  = Pdes.py;
  float pz  = Pdes.pz;

  // 1) θ1 from base yaw (r13 = sin θ1, r23 = -cos θ1)
  t1 = atan2(r13, -r23);

  // 2) θ234 from wrist orientation (r31 = sin θ234, r32 = cos θ234)
  float theta234 = atan2(r31, r32);
  float c234 = cos(theta234);
  float s234 = sin(theta234);

  // 3) Project into radial plane after removing yaw
  float B = px * cos(t1) + py * sin(t1);

  // Separate a4
  float Bp = B  - a4 * c234;  // B'
  float Zp = pz - a4 * s234;  // Z'

  // 4) Two-link geometry for (a2, a3)
  float R = sqrt(Bp * Bp + Zp * Zp);
  if (R < 1e-6f) {
    return false;  // numeric issue
  }

  float gamma = atan2(Zp, Bp);
  float K     = (R*R + a2*a2 - a3*a3) / (2.0f * a2);

  float cos_val = K / R;
  if (cos_val >  1.0f) cos_val =  1.0f;
  if (cos_val < -1.0f) cos_val = -1.0f;

  float delta = acos(cos_val);

  // Elbow-up branch (use +delta or -delta to switch)
  t2 = gamma + delta;
  // Elbow-down would be: t2 = gamma + delta;

  // 5) φ = θ2 + θ3
  float num_phi_y = Zp - a2 * sin(t2);
  float num_phi_x = Bp - a2 * cos(t2);
  float phi = atan2(num_phi_y, num_phi_x);

  // 6) θ3, θ4
  t3 = phi - t2;
  t4 = theta234 - phi;

  
  return true;
}

// ====== TICKS <-> RADIANS FOR MX-28 ======
int radToPos(float theta) {
  float revs = theta / (2.0f * PI);
  long ticks = (long)roundf(revs * 4095.0f);
  ticks %= 4096;
  if (ticks < 0) ticks += 4096;
  return (int)ticks;
}

// ====== MOVE ARM TO PICKUP POSE AND CLOSE GRIPPER ======
void moveToPickup() {
  Pose Pdes = buildPickupPose();

  float t1, t2, t3, t4;
  if (!solveIK(Pdes, t1, t2, t3, t4)) {
    Serial.println("IK failed for pickup pose");
    return;
  }

  // Debug: print IK solution
  Serial.println("=== Pickup Pose IK Solution ===");
  Serial.print("Joint angles (deg): ");
  Serial.print("t1="); Serial.print(t1 * 180.0f / PI, 2); Serial.print(", ");
  Serial.print("t2="); Serial.print(t2 * 180.0f / PI, 2); Serial.print(", ");
  Serial.print("t3="); Serial.print(t3 * 180.0f / PI, 2); Serial.print(", ");
  Serial.print("t4="); Serial.print(t4 * 180.0f / PI, 2); Serial.println();
  Serial.println("================================");

  float q[4] = { t1, t2, t3, t4 };
  // Command MX-28 joints (IDs in jointIDs[0..3])
  for (int i = 0; i < 4; i++) {
    float thetaServo = q[i] * direction_offset[i] + JOINT_OFFSET[i];  // add 180° for joints 2–4
    int goalTicks = radToPos(thetaServo);
    moveServo(jointIDs[i], goalTicks);
    Serial.println(thetaServo);
    Serial.println(goalTicks);
    delay(100);

}


  // Let arm move into position
  delay(1000);


  moveServo(5,200);
  delay(500);
  moveServo(5,550);
  Serial.println("Gripper closing to fixed position.");
  delay(2000);  // time for close
}

// ====== OPEN GRIPPER (XL-320) ======
void openGripper() {
  
  moveServo(5, 200);
  Serial.println("Gripper opening.");
  delay(1000);
  
}

// ====== THROW MOTION ======
void throwDice() {
  Serial.println("=== THROW PHASE ===");

  // Simple "throw" joint angles (tune these)
  float t1_throw = 114;
  float t2_throw = 1000;
  float t3_throw = 0;
  float t4_throw = 500;

  float qThrow[4] = { t1_throw, t2_throw, t3_throw, t4_throw };

  for (int i = 0; i < 4; i++) {
    int goalTicks = qThrow[i];
    moveServo(jointIDs[i], goalTicks);
    delay(5);
  }

  // Let arm reach the throw pose briefly
  delay(400);  // 

  // Release: open gripper
  openGripper();
  delay(150);

  Serial.println("Dice released.");
}


// ====== SETUP ======
void setup() {
  Serial.begin(9600);        // debug
  Serial3.begin(DXL_BAUD);   // Dynamixel bus
  Serial.flush();
  Serial3.flush();

  pinMode(DXL_DIR_PIN, OUTPUT);
  digitalWrite(DXL_DIR_PIN, LOW);  // start in RX mode

  Serial.println("Initializing servos...");
  delay(1000);

  // Enable MX-28 joints (first 5 IDs)
  for (int i = 0; i < 5; i++) {
    enableTorqueMX(jointIDs[i]);
    delay(20);
  }
  

  Serial.println("All torques enabled.");
}

// ====== MAIN LOOP (one-shot demo) ======
void loop() {

  /*
  moveServo(1,0);
  delay(500);
  moveServo(5,500);
  
  delay(500);
  moveServo(3, 0);
  delay(500);
  moveServo(2, 0);
  delay(500);
  moveServo(1, 0);
  delay(500);
  */
  
  

  moveToPickup();
  
  delay(1000);
  throwDice();
  delay(1000);
  

}

// ====== LOW-LEVEL DYNAMIXEL HELPERS ======

void moveServo(byte servoID, int Position) {
  unsigned char CRC_L = 0;
  unsigned char CRC_H = 0;
  unsigned char Position_H = Position >> 8;  // high byte
  unsigned char Position_L = Position & 0xFF;

  // MX-28AR Protocol 2.0, Goal Position address 0x74, length 4
  unsigned char packet[] = {
    0xFF, 0xFF, 0xFD, 0x00,   // header
    servoID,                  // ID
    0x09, 0x00,               // length = 9
    0x03,                     // Instruction: WRITE
    0x74, 0x00,               // Goal Position address (low, high)
    Position_L,
    Position_H,
    0x00, 0x00,               // padding for 4-byte value (we use only 2 bytes here)
    0x00, 0x00                // CRC placeholders
  };

  unsigned short CRC = update_crc(0, packet, 14);
  CRC_L = CRC & 0xFF;
  CRC_H = (CRC >> 8) & 0xFF;
  packet[14] = CRC_L;
  packet[15] = CRC_H;

  digitalWrite(DXL_DIR_PIN, HIGH);     // TX mode
  Serial3.write(packet, sizeof(packet));
  Serial3.flush();
  digitalWrite(DXL_DIR_PIN, LOW);      // back to RX mode
}

// Enable torque for MX-28 (ID != gripper)
void enableTorqueMX(byte servoID) {
  unsigned char CRC_L = 0;
  unsigned char CRC_H = 0;
  unsigned char packet[] = {
    0xFF, 0xFF, 0xFD, 0x00,
    servoID,
    0x06, 0x00,         // length = 6
    0x03,               // WRITE
    0x40, 0x00,         // Torque Enable = 64
    0x01,               // value = 1 (ON)
    0x00, 0x00          // CRC placeholders
  };


  unsigned short CRC = update_crc(0, packet, 11);
  CRC_L = CRC & 0xFF;
  CRC_H = (CRC >> 8) & 0xFF;
  packet[11] = CRC_L;
  packet[12] = CRC_H;

  digitalWrite(DXL_DIR_PIN, HIGH);
  Serial3.write(packet, sizeof(packet));
  Serial3.flush();
  digitalWrite(DXL_DIR_PIN, LOW);
}




// CRC function (unchanged)
unsigned short update_crc(unsigned short crc_accum, unsigned char *data_blk_ptr, unsigned short data_blk_size)
{
  unsigned short i, j;
  static const unsigned short crc_table[256] = {
    0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
    0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
    0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
    0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
    0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
    0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
    0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
    0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
    0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
    0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
    0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
    0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
    0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
    0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
    0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
    0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
    0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
    0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
    0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
    0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
    0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
    0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
    0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
    0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
    0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
    0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
    0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
    0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
    0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
    0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
    0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
    0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
  };

  for (j = 0; j < data_blk_size; j++) {
    i = (unsigned short)((crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
    crc_accum = (crc_accum << 8) ^ crc_table[i];
  }
  return crc_accum;
}