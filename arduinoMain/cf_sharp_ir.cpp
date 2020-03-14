#include "cf_sharp_ir.h"

#include <Arduino.h>

namespace {
constexpr int kNbSample = 50;
constexpr int kSRVMin = 120;
constexpr int kLRVMin = 160;
void sort(int a[], int n) {
  for (int i = 1; i < n; i++) {
    int next = a[i];
    int j;
    for (j = i - 1; j >= 0 && a[j] > next; j--) a[j + 1] = a[j];
    a[j + 1] = next;
  }
}
}  // namespace

CFSharpIR::CFSharpIR(int ir_pin, const uint32_t model, double m, double b,
                   double k)
    : ir_pin_(ir_pin), model_(model), m_(m), b_(b), k_(k) {
  // Define pin as Input
  pinMode(ir_pin_, INPUT);
}

int CFSharpIR::getIrPin() const { return ir_pin_; }

int CFSharpIR::getRaw() const {
  int ir_val[kNbSample];
  for (int i = 0; i < kNbSample; i++) ir_val[i] = analogRead(ir_pin_);
  sort(ir_val, kNbSample);

  return ir_val[kNbSample / 2];
}

int CFSharpIR::distance() const {
  int ir_val = getRaw();
  return compute(ir_val);
}

int CFSharpIR::compute(int ir_val) const {
  if (ir_val <= -b_) return -1;
  if (model_ == GP2Y0A21YK0F && ir_val < kSRVMin)
    ir_val = kSRVMin;
  else if (model_ == GP2Y0A02YK0F && ir_val < kLRVMin)
    ir_val = kLRVMin;
  double distanceCM = m_ / (ir_val + b_) - k_;
  return round(distanceCM * 10);
}
