#ifndef SHARP_IR_H_
#define SHARP_IR_H_

// A lightweight ZSharpIR library for calibrated sensors.
class ZSharpIR {
public:
  ZSharpIR(int ir_pin, const uint32_t sensor_type,
      double m, double b, double k);
  static const uint32_t GP2Y0A21YK0F = 1080;
  static const uint32_t GP2Y0A02YK0F = 20150;
  int getIrPin() const;
  int getRaw() const;
  int distance() const;

private:
  int ir_pin_;
  uint32_t model_;
  double m_, b_, k_;
  int compute(int ir_val) const;
}

#endif