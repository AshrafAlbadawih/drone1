// ============================================
// DUAL JOYSTICK DRONE CONTROL - SIMPLE VERSION
// ============================================

// تعريف الدبابيس
#define JOY1_X A0
#define JOY1_Y A1  
#define JOY1_Z A2
#define JOY2_X A5
#define JOY2_Y A4
#define JOY2_Z A3
#define BTN_ARM 2
#define BTN_DISARM 3
#define BTN_MODE 4

// دالة الإعداد - REQUIRED BY ARDUINO
void setup() {
  // بدء الاتصال التسلسلي
  Serial.begin(115200);
  
  // تهيئة الأزرار
  pinMode(BTN_ARM, INPUT_PULLUP);
  pinMode(BTN_DISARM, INPUT_PULLUP);
  pinMode(BTN_MODE, INPUT_PULLUP);
  
  // انتظر لتثبيت الاتصال
  delay(2000);
  
  // رسالة جاهز
  Serial.println("ARDUINO_READY");
}

// دالة الحلقة الرئيسية - REQUIRED BY ARDUINO
void loop() {
  // قراءة محاور الجويستيك الأول
  int x_val = analogRead(JOY1_X);
  int y_val = analogRead(JOY1_Y);
  int z_val = analogRead(JOY1_Z);
  
  // قراءة محاور الجويستيك الثاني
  int roll_val = analogRead(JOY2_X);
  int pitch_val = analogRead(JOY2_Y);
  int yaw_val = analogRead(JOY2_Z);
  
  // تحويل إلى نطاق 1000-2000
  x_val = map(x_val, 0, 1023, 1000, 2000);
  y_val = map(y_val, 0, 1023, 1000, 2000);
  z_val = map(z_val, 0, 1023, 1000, 2000);
  roll_val = map(roll_val, 0, 1023, 1000, 2000);
  pitch_val = map(pitch_val, 0, 1023, 1000, 2000);
  yaw_val = map(yaw_val, 0, 1023, 1000, 2000);
  
  // قراءة الأزرار
  int btn_arm = (digitalRead(BTN_ARM) == LOW) ? 1 : 0;
  int btn_disarm = (digitalRead(BTN_DISARM) == LOW) ? 1 : 0;
  int btn_mode = (digitalRead(BTN_MODE) == LOW) ? 1 : 0;
  
  // حساب CRC
  int crc = (x_val + y_val + z_val + roll_val + pitch_val + yaw_val + 
             btn_arm + btn_disarm + btn_mode) % 256;
  
  // إرسال البيانات
  Serial.print(x_val); Serial.print(",");
  Serial.print(y_val); Serial.print(",");
  Serial.print(z_val); Serial.print(",");
  Serial.print(roll_val); Serial.print(",");
  Serial.print(pitch_val); Serial.print(",");
  Serial.print(yaw_val); Serial.print(",");
  Serial.print(btn_arm); Serial.print(",");
  Serial.print(btn_disarm); Serial.print(",");
  Serial.print(btn_mode); Serial.print(",");
  Serial.println(crc);
  
  // انتظر 100ms (10Hz)
  delay(100);
}