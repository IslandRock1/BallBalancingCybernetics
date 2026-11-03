
#include <math.h>
#include "Kinematics.hpp"

// calculates points a10, a20, b10, b20, c10, and c20
float a10[3] = { -d1, -l0, 0 };
float a20[3] = { -a10[x], a10[y], 0 };
float b10[3] = { l0*sqrt(3.0f)/2.0f + d1/2.0f, l0/2.0f - d1*sqrt(3.0f)/2.0f, 0 };
float b20[3] = { l0*sqrt(3.0f)/2.0f - d1/2.0f, l0/2.0f + d1*sqrt(3.0f)/2.0f, 0 };
float c10[3] = { -b20[x], b20[y], 0 };
float c20[3] = { -b10[x], b10[y], 0 };

// calculates vectors a_10_20, b_10_20, c_10_20,
float a_10_20[3] = { a20[x] - a10[x], a20[y] - a10[y], a20[z] - a10[z] };
float b_10_20[3] = { b20[x] - b10[x], b20[y] - b10[y], b20[z] - b10[z] };
float c_10_20[3] = { c20[x] - c10[x], c20[y] - c10[y], c20[z] - c10[z] };

//calculates normal vectors na, nb, and nc for each side of the triangle
float na[3] = { 0, -1, 0 }; // pointing out
float nb[3] = { sqrt(3.0f)/2.0f, 1/2.0f, 0 }; // pointing out
float nc[3] = { -sqrt(3.0f)/2.0f, 1/2.0f, 0 }; // pointing out

float t = lf/2.0f + m*sqrt(3.0f)/2.0f;
float w = lf*sqrt(3.0f)/2.0f - m/2.0f;

// calculated theta values a1, a2, b1, b2, c1, c2 respectively
float theta[6];
float theta_prev[6];

// compute rotations
void rotations(float* result, float hi[3], float Rz[3][3], float Ry[3][3], float Rx[3][3]) {
  float temp[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}; // Rz * Ry
  float combined[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}; // (Rz * Ry) * Rx

  // Compute Rz * Ry
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      for (int k = 0; k < 3; k++) {
        temp[i][j] += Rz[i][k] * Ry[k][j];
      }
    }
  }

  // Compute (Rz * Ry) * Rx
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      for (int k = 0; k < 3; k++) {
        combined[i][j] += temp[i][k] * Rx[k][j];
      }
    }
  }

  // Compute (Rz * Ry * Rx) * hi
  for (int i = 0; i < 3; i++) {
    result[i] = 0; // Initialize result
    for (int j = 0; j < 3; j++) {
      result[i] += combined[i][j] * hi[j];
    }
  }
}

// compute inverse kinematics
void InverseKinematics(float hx, float hy, float hz, float roll, float pitch, float yaw) {
  // calculates theta values given hx, hy, hz, roll, pitch, yaw

  //-------------------------
  // **STAGE 1 CALCULATIONS**
  // From {hx, hy, hz, roll, pitch, yaw} => get {ax, ay, az; bx, by, bz; cx, cy, cz}
  //-------------------------

  // define point h (center point of platform)
  float h[3] = { hx, hy, hz };

  // define rotation matrices for yaw, pitch and roll
  float R_z_yaw[3][3] = {{cos(yaw * toRad), -sin(yaw * toRad), 0}, {sin(yaw * toRad), cos(yaw * toRad), 0}, {0, 0, 1}}; // row, row, row
  float R_y_pitch[3][3] = {{cos(pitch * toRad), 0, sin(pitch * toRad)}, {0, 1, 0}, {-sin(pitch * toRad), 0, cos(pitch * toRad)}};
  float R_x_roll[3][3] = {{1, 0, 0}, {0, cos(roll * toRad), -sin(roll * toRad)}, {0, sin(roll * toRad), cos(roll * toRad)}};

  // define initial ha, hb, and hc
  float ha_i[3] = {0, -t, 0};
  float hb_i[3] = {sqrt(3.0f)*t/2.0f, t/2.0f, 0};
  float hc_i[3] = {-sqrt(3.0f)*t/2.0f, t/2.0f, 0};

  // initialize rotated ha, hb, hc
  float ha[3] = {0, 0, 0};
  float hb[3] = {0, 0, 0};
  float hc[3] = {0, 0, 0};

  // calculate rotated ha, hb, hc
  rotations(ha, ha_i, R_z_yaw, R_y_pitch, R_x_roll);
  rotations(hb, hb_i, R_z_yaw, R_y_pitch, R_x_roll);
  rotations(hc, hc_i, R_z_yaw, R_y_pitch, R_x_roll);

  // calculate af, bf, cf
  float af[3] = {h[0] + ha[0], h[1] + ha[1], h[2] + ha[2]};
  float bf[3] = {h[0] + hb[0], h[1] + hb[1], h[2] + hb[2]};
  float cf[3] = {h[0] + hc[0], h[1] + hc[1], h[2] + hc[2]};


  //-------------------------
  // STAGE 2 CALCULATIONS
  // From {ax, ay, az, bx, by, bz, cx, cy, cz}
  // => get {a1fx, a1fy, a1fz; a2fx, a2fy, a2fz; b1fx, b1fy, b1fz; b2fx, b2fy, b2fz; c1fx, c1fy, c1fz; c2fx, c2fy, c2fz}
  //-------------------------

  // define initial vectors af_a1f, af_a2f, bf_bf1, bf_b2f, cf_c1f, cf_c2f
  float af_a1f_i[3] = {-w, 0, 0};
  float af_a2f_i[3] = {w, 0, 0};
  float bf_b1f_i[3] = {w/2.0f, -w*sqrt(3.0f)/2.0f, 0};
  float bf_b2f_i[3] = {-w/2.0f, w*sqrt(3.0f)/2.0f, 0};
  float cf_c1f_i[3] = {w/2.0f, w*sqrt(3.0f)/2.0f, 0};
  float cf_c2f_i[3] = {-w/2.0f, -w*sqrt(3.0f)/2.0f, 0};


  // initialize rotated vectors a1f, a2f, b1f, b2f, c1f, c2f
  float af_a1f[3] = {0, 0, 0};
  float af_a2f[3] = {0, 0, 0};
  float bf_b1f[3] = {0, 0, 0};
  float bf_b2f[3] = {0, 0, 0};
  float cf_c1f[3] = {0, 0, 0};
  float cf_c2f[3] = {0, 0, 0};

  // calculate rotated vectors a1f, a2f, b1f, b2f, c1f, c2f
  rotations(af_a1f, af_a1f_i, R_z_yaw, R_y_pitch, R_x_roll);
  rotations(af_a2f, af_a2f_i, R_z_yaw, R_y_pitch, R_x_roll);
  rotations(bf_b1f, bf_b1f_i, R_z_yaw, R_y_pitch, R_x_roll);
  rotations(bf_b2f, bf_b2f_i, R_z_yaw, R_y_pitch, R_x_roll);
  rotations(cf_c1f, cf_c1f_i, R_z_yaw, R_y_pitch, R_x_roll);
  rotations(cf_c2f, cf_c2f_i, R_z_yaw, R_y_pitch, R_x_roll);

  // calculate a1f, a2f, b1f, b2f, c1f, c2f
  float a1f[3] = {af[0] + af_a1f[0], af[1] + af_a1f[1], af[2] + af_a1f[2]};
  float a2f[3] = {af[0] + af_a2f[0], af[1] + af_a2f[1], af[2] + af_a2f[2]};
  float b1f[3] = {bf[0] + bf_b1f[0], bf[1] + bf_b1f[1], bf[2] + bf_b1f[2]};
  float b2f[3] = {bf[0] + bf_b2f[0], bf[1] + bf_b2f[1], bf[2] + bf_b2f[2]};
  float c1f[3] = {cf[0] + cf_c1f[0], cf[1] + cf_c1f[1], cf[2] + cf_c1f[2]};
  float c2f[3] = {cf[0] + cf_c2f[0], cf[1] + cf_c2f[1], cf[2] + cf_c2f[2]};

  // calculate vector a1 (from a10 to a1f), a2 (from a20 to a2f), b1 (from c10 to c1f), b2 (from b20 to b2f), c1 (from c10 to c1f), c2 (from c20 to c2f)
  float a1[3] = { a1f[0] - a10[0], a1f[1] - a10[1], a1f[2] - a10[2] };
  float a2[3] = { a2f[0] - a20[0], a2f[1] - a20[1], a2f[2] - a20[2] };
  float b1[3] = { b1f[0] - b10[0], b1f[1] - b10[1], b1f[2] - b10[2] };
  float b2[3] = { b2f[0] - b20[0], b2f[1] - b20[1], b2f[2] - b20[2] };
  float c1[3] = { c1f[0] - c10[0], c1f[1] - c10[1], c1f[2] - c10[2] };
  float c2[3] = { c2f[0] - c20[0], c2f[1] - c20[1], c2f[2] - c20[2] };


  //-------------------------
  //**STAGE 3 CALCULATIONS**
  // From {a1fx, a1fy, a1fz; a2fx, a2fy, a2fz; b1fx, b1fy, b1fz; b2fx, b2fy, b2fz; c1fx, c1fy, c1fz; c2fx, c2fy, c2fz}
  // => get {theta_a10, theta_a20, theta_b10, theta_b20, theta_c10, theta_c20}
  //-------------------------

  // save previous thetas
  for (int i = 0; i < 6; i++) {
    theta_prev[i] = theta[i];
  }

  // theta_a1
  float a1s_mod = abs(dot(a1, na)); // magnitude of vector 'a1s'
  float a1s[3] = { na[0] * a1s_mod, na[1] * a1s_mod, na[2] * a1s_mod };
  float a1_proj[3] = { a1[0] + a1s[0], a1[1] + a1s[1], a1[2] + a1s[2] }; // projection of vector 'a1' onto the ac plane
  float a1_proj_mod = mag(a1_proj); // magnitude of vector 'a1' projected on the ac plane
  float p2_proj_mod = sqrt(pow(p2, 2) - pow(a1s_mod, 2)); // magnitude of link p2 projected on the ac plane
  theta[0] = acos(dot(a1_proj, a_10_20) / (2 * d1 * a1_proj_mod ) );
  theta[0] = (theta[0] - acos((pow(a1_proj_mod, 2) + pow(p1, 2) - pow(p2_proj_mod, 2)) / (2 * a1_proj_mod * p1))) * toDeg;

  // theta_a2
  float a2s_mod = abs(dot(a2, na)); // magnitude of vector 'a2s'
  float a2s[3] = { na[0] * a2s_mod, na[1] * a2s_mod, na[2] * a2s_mod };
  float a2_proj[3] = { a2[0] + a2s[0], a2[1] + a2s[1], a2[2] + a2s[2] }; // projection of vector 'a2' onto the ab plane
  float a2_proj_mod = mag(a2_proj); // magnitude of vector 'a2' projected on the ab plane
  p2_proj_mod = sqrt(pow(p2, 2) - pow(a2s_mod, 2)); // magnitude of link p2 projected on the ab plane
  theta[1] = acos(-dot(a2_proj, a_10_20) / (2 * d1 * a2_proj_mod));
  theta[1] = (theta[1] - acos((pow(a2_proj_mod, 2) + pow(p1, 2) - pow(p2_proj_mod, 2)) / (2 * a2_proj_mod * p1))) * toDeg;

  // theta_b1
  float b1s_mod = abs(dot(b1, nb)); // magnitude of vector 'b1s'
  float b1s[3] = { nb[0] * b1s_mod, nb[1] * b1s_mod, nb[2] * b1s_mod };
  float b1_proj[3] = { b1[0] + b1s[0], b1[1] + b1s[1], b1[2] + b1s[2] }; // projection of vector 'b1' onto the ab plane
  float b1_proj_mod = mag(b1_proj); // magnitude of vector 'b1' projected on the ab plane
  p2_proj_mod = sqrt(pow(p2, 2) - pow(b1s_mod, 2)); // magnitude of link p2 projected on the ab plane
  theta[2] = acos(dot(b1_proj, b_10_20) / (2 * d1 * b1_proj_mod));
  theta[2] = (theta[2] - acos((pow(b1_proj_mod, 2) + pow(p1, 2) - pow(p2_proj_mod, 2)) / (2 * b1_proj_mod * p1))) * toDeg;

  // theta_b2
  float b2s_mod = abs(dot(b2, nb)); // magnitude of vector 'b2s'
  float b2s[3] = { nb[0] * b2s_mod, nb[1] * b2s_mod, nb[2] * b2s_mod };
  float b2_proj[3] = { b2[0] + b2s[0], b2[1] + b2s[1], b2[2] + b2s[2] }; // projection of vector 'b2' onto the cb plane
  float b2_proj_mod = mag(b2_proj); // magnitude of vector 'b2' projected on the cb plane
  p2_proj_mod = sqrt(pow(p2, 2) - pow(b2s_mod, 2)); // magnitude of link p2 projected on the cb plane
  theta[3] = acos(-dot(b2_proj, b_10_20) / (2 * d1 * b2_proj_mod));
  theta[3] = (theta[3] - acos((pow(b2_proj_mod, 2) + pow(p1, 2) - pow(p2_proj_mod, 2)) / (2 * b2_proj_mod * p1))) * toDeg;

  // theta_c1
  float c1s_mod = abs(dot(c1, nc)); // magnitude of vector 'c1s'
  float c1s[3] = { nc[0] * c1s_mod, nc[1] * c1s_mod, nc[2] * c1s_mod };
  float c1_proj[3] = { c1[0] + c1s[0], c1[1] + c1s[1], c1[2] + c1s[2] }; // projection of vector 'c1' onto the cb plane
  float c1_proj_mod = mag(c1_proj); // magnitude of vector 'c1' projected on the cb plane
  p2_proj_mod = sqrt(pow(p2, 2) - pow(c1s_mod, 2)); // magnitude of link p2 projected on the cb plane
  theta[4] = acos(dot(c1_proj, c_10_20) / (2 * d1 * c1_proj_mod));
  theta[4] = (theta[4] - acos((pow(c1_proj_mod, 2) + pow(p1, 2) - pow(p2_proj_mod, 2)) / (2 * c1_proj_mod * p1))) * toDeg;

  //theta_c2
  float c2s_mod = abs(dot(c2, nc)); // magnitude of vector 'c2s'
  float c2s[3] = { nc[0] * c2s_mod, nc[1] * c2s_mod, nc[2] * c2s_mod };
  float c2_proj[3] = { c2[0] + c2s[0], c2[1] + c2s[1], c2[2] + c2s[2] }; // projection of vector 'c2' onto the ac plane
  float c2_proj_mod = mag(c2_proj); // magnitude of vector 'c2' projected on the ac plane
  p2_proj_mod = sqrt(pow(p2, 2) - pow(c2s_mod, 2)); // magnitude of link p2 projected on the ac plane
  theta[5] = acos(-dot(c2_proj, c_10_20) / (2 * d1 * c2_proj_mod));
  theta[5] = (theta[5] - acos((pow(c2_proj_mod, 2) + pow(p1, 2) - pow(p2_proj_mod, 2)) / (2 * c2_proj_mod * p1))) * toDeg;


  for (int i = 0; i < 6; i++) {

    // change theta sign (negative up)
    theta[i] = -theta[i];

    //checks theta values are between -40 and 40 degrees
    if (abs(theta[i]) > 40) {
      Serial.println("ERROR: CURRENT VALUES EXCEED ANGLE RANGE");
      Serial.println("Motor index:");
      Serial.println(i);
      Serial.println("Motor angle value:");
      Serial.println(theta[i]);
      stop();
    }

    //checks theta values are real numbers
    if (isnan(theta[i])) {
      Serial.println("ERROR: CURRENT VALUES CANNOT PHYSICALLY BE EXECUTED");
      Serial.println("Motor index:");
      Serial.println(i);
      Serial.println("Motor angle value:");
      Serial.println(theta[i]);
      stop();
    }

    //checks theta variation is smaller than 80 degrees
    if (abs(theta[i]-theta_prev[i]) > 80) {
      Serial.println("ERROR: CURRENT VALUES ARE TOO BIG FOR ONE STEP");
      Serial.println("Motor index:");
      Serial.println(i);
      Serial.println("Motor angle variation value:");
      Serial.println(theta[i]-theta_prev[i]);
      stop();
    }

  }
}
