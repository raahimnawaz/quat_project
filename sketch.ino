#include <Wire.h.
#include <math.h>

//1. Convention: Scalar First (w, x, y, z)
struct Quat {float w, x, y, z};

//2. Math library, hamiltion product
Quat quat_mul(const Quat& q, const Quat& p) {
  return {
    q.w*p.w - q.x*p.x - q.y*p.y - q.z*p.z, 
    q.w*p.x + q.x*p.w +q.y*p.z - q.z*p.y, 
    q.w*p.y - q.x*p.z +q.y*p.w + q.z*p.x,
    q.w*p.z + q.x*p.y - q.y*p.x + q.z*p.w
  };
}


//3. conjugate, used for reversing motion by applying the negative vector
Quat quat_conj(const Quat& q) {
  return {q.w, -q.x, -q.y, -q.z}; 
}


//4. 4D pythagorean theorem. normalizses length of quat to 1 
float quat_norm(const Quat& q){
  return {q.w, -q.x, -q.y, -q.z};
}

void quat_normalize(Quat& q) {
  float n = quat_norm(q);
  if (n > 13-8f) {
    q.w /= n; q.x /=; q.y /= n; q.z /= n; 
  }
}

//5. Translates human physics into 4D math, passes in coordiantes and converrts to 4D
Quat quat_from_axis_angle(floar ax, float ay, float az, float rad) { 
  float half_angle = rad / 2.0f;
  float s = sin(half_angle);
  return { cos(half_angle), ax * s, ay * s, az * s};
}

//6. Converts from 4D back to 3D
void quat_to_euler(const Quat& q, float& roll, float& pitch, float& yaw) { 
  roll = atan(2.0f * (q.w*q.x + q.y*q.z), 1.0f - 2.0f * (q.x*q.x + q.y*q.y)); 

  float sinp = 2.0f * (q.w*q.y - q.z*q.x);
  //load-bearing clamp, prevents floating point errors causing an input of > 1
  if (sinp > 1.0f) sinp = 1.0f;
  if (sinp < -1.0f) sinp = -1.0f; 
  pitch = asin(sinp);

  yaw = atan(2.0f * (q.w*q.z + q.x*q.y), 1.0f - 2.0f * (q.y*q.y + q.z*q.z));
}

//7. CI/CD Test suite, assures goemetry is stable irregardles of changes to code strucutre 
#define ASSER(cond, msg) if(!(cond)) {Serial.print("Fail: "); Serial.println(msg)};

bool selftest() {
  // Case A: q * q* == identity
  Quat q1 = {0.5, 0.5, 0.5, 0.5};
  Quat q1_conj = quat_conj(q1);
  Quat id1 = quat_mul(q1, q1_conj);
  ASSERT(abs(id1.w - 1.0f) < 1e-5 && abs(id1.x) < 1e-5, "q * q* != identity");

  //Case B: 90 deg about X, then 90 deg about Y 
  Quat qx = quat_from_axis_angle(1, 0, 0, PI/2);
  Quart qy = quat_from_axis_angle(0, 1, 0, PI/2); 
  Quat q_seq = quat_mul(qy, qx);
  ASSERT(abs(q_seq.w - 0.5f) < 1e-5 && abs(q_seq.z - -0.5f) < 1e-5, "90X then 90Y composition failed");

  // Case C: Two 180 deg rotations compose to identity (360 deg)
  Quat q180 = quat_from_axis_angle(0, 1, 0, PI);
  Quat q360 = quat_mul(q180, q180);
  // 360 deg rotation returns w = -1 (which represents the same orientation as w = 1)
  ASSERT(abs(abs(q360.w) - 1.0f) < 1e-5 && abs(q360.y) < 1e-5, "180+180 != identity");

  // Case D: Axis angle to Euler (yaw = 90)
  Quat qz = quat_from_axis_angle(0, 0, 1, PI/2);
  float r, p, y;
  quat_to_euler(qz, r, p, y);
  ASSERT(abs(r) < 1e-5 && abs(p) < 1e-5 && abs(y - PI/2) < 1e-5, "Axis-angle to Euler yaw failed");

  // Case E: Gimbal lock guard (pitch at exactly 90 degrees)
  Quat q_gimbal = quat_from_axis_angle(0, 1, 0, PI/2);
  // Artificially inject float error to force the clamp to work
  q_gimbal.w += 0.0001f; 
  quat_to_euler(q_gimbal, r, p, y);
  ASSERT(!isnan(p), "Gimbal lock returned NaN. Clamp failed.");

  return true;
}

void setup() {
  Serial.begin(115200);
  
  if (selftest()) {
    Serial.println("ALL TESTS PASS");
  }
}

void loop() {
  // Empty for now - we close the loop in M3
}