#ifndef HELPERS_H
#define HELPERS_H

union binaryFloat {
	float f;
	unsigned char c[4];
};

union binaryInt16 {
  int16_t i;
  uint8_t c[2];
};

union anyType {
  float f;
  uint32_t ul;
  int32_t l;
  uint16_t ui[2];
  int16_t i[2];
  unsigned char c[4];
};

void setHigh(int pin);

void setLow(int pin);

void sendPulse(int pin);

void sendNegativePulse(int pin);

uint8_t convertToByte(int value, int scaleBottom, int scaleTop); //Also constrains the output between 0 - 127 :)

uint8_t maxByte(uint8_t a, uint8_t b);

uint8_t minByte(uint8_t a, uint8_t b);

#define TO_RAD(x) (x * 0.01745329252)  // *pi/180
#define TO_DEG(x) (x * 57.2957795131)  // *180/pi

inline int16_t floatToQ14( float value);
inline float Q14ToFloat( int16_t value);
inline int numberOfSetBits(uint32_t i);
inline void quat_mult(const float *q1, const float *q2, float *qProd);
inline void quat_norm(float* q);
inline void quat_conj(float* q);
inline void quat_rotate(const float* q, const float* vecIn, float* vecOut);
inline void quat_copy(const float* q, float* qOut);
inline void quat_build(const float* axis, float angle, float* qOut);
inline float quat_angle(const float* q);
inline void quat_axis(const float* q, float* vecOut);
inline void quat_eulerAngles(const float* q, float* e);
inline void vec_scale(float out[3], const float v[3], float scale);
inline void vec_add(float out[3], const float v1[3], const float v2[3]);
inline void vec_copy(const float* v, float* vOut);
inline float vec_dot(const float* v1, const float* v2);
inline void vec_norm(float* v);
inline float vec_angle(const float* v1, const float* v2);
inline float vec_length(const float* v);
inline float vec_lengthSquared(const float* v);
inline float vec_distance_squared(const float* v1, const float* v2);
inline float vec_distance(const float* v1, const float* v2);

#endif
