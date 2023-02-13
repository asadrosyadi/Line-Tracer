#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>
#include <Servo.h>
#include "Adafruit_TCS34725.h"

LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address
//LiquidCrystal_I2C lcd(0x27, 16, 2);
Servo base,grip;
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

float red,green,blue;
int r, g, b, c, colorTemp, lux, sel;
char warna=" ";
int count_objek=0;
float speed_right=0.8; //set antara 0-1, semakin kecil semakin kenceng
float speed_left=0.78;  //set antara 0-1, semakin kecil semakin kenceng
const int inh=12;
const int selA=A0;
const int selB=A3;
const int selC=A2;
const int sensor_depan=A6;
const int button1=1;
const int button2=0;
const int button3=11;
const int rem_ka=8;
const int dir_ka=9;
const int ena_ka=5;  
const int rem_ki=7;
const int dir_ki=10;
const int ena_ki=6;
const int led_hijau=3;
const int obstacle=2;
int data_obstacle;
//const int INB=4;
//const int INA=3;
const int servo_base=4;
const int servo_grip=13;
unsigned char tombol1=0;
unsigned char tombol2=0;
unsigned char tombol3=0;
int data_sensor[10];
unsigned int counter=0,i,j,count=0,up=0,lintasan=0;
int xsensor,var,jumlah_indeks,jum_indeks;
int s0,s1,s2,s3,s4,s5,s6,s7,s8,s9,flag;
int ref[10],sens[10];
int sensor_now[10],maks[10];
int minimal[10]={1100,1100,1100,1100,1100,1100,1100,1100,1100,1100};
int address=0,address_jumlah_indeks=50,address_speed=15,address_kons_p=20,address_kons_d=30,address_kons_i=40;
unsigned char pwm_ka,pwm_ki,indeks=0;
int laju,kecepatan,pi,di,ai,kons_p,kons_d,kons_i;

void maju(unsigned char kiri, unsigned char kanan){
    pwm_ka=kanan;                     pwm_ki=kiri;
    pwm_ka=(255-pwm_ka)*speed_right;  pwm_ki=(255-pwm_ki)*speed_left;
    if(kanan==0){pwm_ka=255;}         if(kiri==0){pwm_ki=255;}
    analogWrite(ena_ka,pwm_ka);       analogWrite(ena_ki,pwm_ki);
    digitalWrite(dir_ka,HIGH);        digitalWrite(dir_ki,HIGH);
    digitalWrite(rem_ka,HIGH);        digitalWrite(rem_ki,HIGH);
}
void mundur(unsigned char kiri, unsigned char kanan){
    pwm_ka=kanan;                     pwm_ki=kiri;
    pwm_ka=(255-pwm_ka)*speed_right;  pwm_ki=(255-pwm_ki)*speed_left;
    if(kanan==0){pwm_ka=255;}         if(kiri==0){pwm_ki=255;}
    analogWrite(ena_ka,pwm_ka);       analogWrite(ena_ki,pwm_ki);
    digitalWrite(dir_ka,LOW);         digitalWrite(dir_ki,LOW);
    digitalWrite(rem_ka,HIGH);        digitalWrite(rem_ki,HIGH);
}
void bel_ka(unsigned char kiri,unsigned char kanan){      
    pwm_ka=kanan;                     pwm_ki=kiri;                       
    pwm_ka=(255-pwm_ka)*speed_right;  pwm_ki=(255-pwm_ki)*speed_left;   
    if(kanan==0){pwm_ka=255;}         if(kiri==0){pwm_ki=255;}
    analogWrite(ena_ka,pwm_ka);       analogWrite(ena_ki,pwm_ki);
    digitalWrite(dir_ka,LOW);         digitalWrite(dir_ki,HIGH);
    digitalWrite(rem_ka,HIGH);        digitalWrite(rem_ki,HIGH);   
}
void bel_ki(unsigned char kiri,unsigned char kanan){   
    pwm_ka=kanan;                     pwm_ki=kiri;                           
    pwm_ka=(255-pwm_ka)*speed_right;  pwm_ki=(255-pwm_ki)*speed_left;
    if(kanan==0){pwm_ka=255;}         if(kiri==0){pwm_ki=255;}
    analogWrite(ena_ka,pwm_ka);       analogWrite(ena_ki,pwm_ki);
    digitalWrite(dir_ka,HIGH);        digitalWrite(dir_ki,LOW);
    digitalWrite(rem_ka,HIGH);        digitalWrite(rem_ki,HIGH);   
}
void rem(){
  digitalWrite(rem_ka,LOW);  digitalWrite(rem_ki,LOW);
}
void stopped(){
  analogWrite(ena_ka,255); 
  analogWrite(ena_ki,255);
  digitalWrite(dir_ka,LOW);   digitalWrite(dir_ki,HIGH);
  digitalWrite(rem_ka,LOW);  digitalWrite(rem_ki,LOW); 
}
void stopped2(){
  mundur(80,80);
  delay(80);
  stopped;
}
void baca_button(){
  tombol1=digitalRead(button1);
  tombol2=digitalRead(button2);
  tombol3=digitalRead(button3);
}
void baca_color(){
  tcs.setInterrupt(false);  // turn on LED
  delay(60);
  tcs.getRGB(&red, &green, &blue); 
  tcs.setInterrupt(true);
  r=red; g=green; b=blue;
}
void identifikasi_color(){
  sel=r-b;
  if(r>g && r>=b){ 
      warna = 'M';  //merah
  }
  else if((g>b && b>=r) || (g>b && sel<7)){  //ijo
    warna = 'I';  
  }
  else if(g>b && r>=b){  //kuning
    warna = 'K';  
  }
}
void servo_on(){
  base.attach(servo_base);  
  grip.attach(servo_grip);
}
void servo_off(){
  base.detach();
  grip.detach();
}
void gripper_buka(){
  for(i=85; i>=25; i--){
    grip.write(i);  
    delay(10);
  }
}
void gripper_tutup(){
  grip.write(90);  //90
}
void gripper_angkat(){
  base.write(175); //175
}
void gripper_turun(){
  base.write(30);
}
void gripper_turun_piramid(){
  base.write(70);
}
void sensing(){
  data_sensor[0]=analogRead(A7);
  digitalWrite(inh, LOW);
  digitalWrite(selA, LOW);
  digitalWrite(selB, LOW);
  digitalWrite(selC, LOW);
  data_sensor[1]=analogRead(sensor_depan);
  digitalWrite(selA, HIGH);
  digitalWrite(selB, LOW);
  digitalWrite(selC, LOW);
  data_sensor[2]=analogRead(sensor_depan);
  digitalWrite(selA, LOW);
  digitalWrite(selB, HIGH);
  digitalWrite(selC, LOW);
  data_sensor[3]=analogRead(sensor_depan);
  digitalWrite(selA, HIGH);
  digitalWrite(selB, HIGH);
  digitalWrite(selC, LOW);
  data_sensor[4]=analogRead(sensor_depan);
  digitalWrite(selA, LOW);
  digitalWrite(selB, LOW);
  digitalWrite(selC, HIGH);
  data_sensor[5]=analogRead(sensor_depan);
  digitalWrite(selA, HIGH);
  digitalWrite(selB, LOW);
  digitalWrite(selC, HIGH);
  data_sensor[6]=analogRead(sensor_depan);
  digitalWrite(selA, LOW);
  digitalWrite(selB, HIGH);
  digitalWrite(selC, HIGH);
  data_sensor[7]=analogRead(sensor_depan);
  digitalWrite(selA, HIGH);
  digitalWrite(selB, HIGH);
  digitalWrite(selC, HIGH);
  data_sensor[8]=analogRead(sensor_depan);
  data_sensor[9]=analogRead(A1);
}
void sensing_kalibrasi(){
  sensor_now[0]=analogRead(A7);
  digitalWrite(inh, LOW);
  digitalWrite(selA, LOW);
  digitalWrite(selB, LOW);
  digitalWrite(selC, LOW);
  sensor_now[1]=analogRead(sensor_depan);
  digitalWrite(selA, HIGH);
  digitalWrite(selB, LOW);
  digitalWrite(selC, LOW);
  sensor_now[2]=analogRead(sensor_depan);
  digitalWrite(selA, LOW);
  digitalWrite(selB, HIGH);
  digitalWrite(selC, LOW);
  sensor_now[3]=analogRead(sensor_depan);
  digitalWrite(selA, HIGH);
  digitalWrite(selB, HIGH);
  digitalWrite(selC, LOW);
  sensor_now[4]=analogRead(sensor_depan);
  digitalWrite(selA, LOW);
  digitalWrite(selB, LOW);
  digitalWrite(selC, HIGH);
  sensor_now[5]=analogRead(sensor_depan);
  digitalWrite(selA, HIGH);
  digitalWrite(selB, LOW);
  digitalWrite(selC, HIGH);
  sensor_now[6]=analogRead(sensor_depan);
  digitalWrite(selA, LOW);
  digitalWrite(selB, HIGH);
  digitalWrite(selC, HIGH);
  sensor_now[7]=analogRead(sensor_depan);
  digitalWrite(selA, HIGH);
  digitalWrite(selB, HIGH);
  digitalWrite(selC, HIGH);
  sensor_now[8]=analogRead(sensor_depan);
  sensor_now[9]=analogRead(A1);
}
void read_sensor(){
  sensing();
  xsensor=255;
  xsensor&=0b11111111;
  if(data_sensor[0]<(sens[0]*4)){s0=0; }   else{s0=1;} 
  if(data_sensor[1]<(sens[1]*4)){s1=0; var=1;}   else{s1=1; var=0; flag=1;} xsensor&=~var; 
  if(data_sensor[2]<(sens[2]*4)){s2=0; var=2;}   else{s2=1; var=0;} xsensor&=~var; 
  if(data_sensor[3]<(sens[3]*4)){s3=0; var=4;}   else{s3=1; var=0;} xsensor&=~var;
  if(data_sensor[4]<(sens[4]*4)){s4=0; var=8;}   else{s4=1; var=0;} xsensor&=~var;
  if(data_sensor[5]<(sens[5]*4)){s5=0; var=16;}  else{s5=1; var=0;} xsensor&=~var;
  if(data_sensor[6]<(sens[6]*4)){s6=0; var=32;}  else{s6=1; var=0;} xsensor&=~var;
  if(data_sensor[7]<(sens[7]*4)){s7=0; var=64;}  else{s7=1; var=0;} xsensor&=~var;
  if(data_sensor[8]<(sens[8]*4)){s8=0; var=128;} else{s8=1; var=0; flag=0;} xsensor&=~var;
  if(data_sensor[9]<(sens[9]*4)){s9=0; }   else{s9=1;} 
}
void tampil_color(){
  //Serial.print("Color Temp: "); Serial.print(colorTemp, DEC); Serial.print(" K - ");
  //Serial.print("Lux: "); Serial.print(lux, DEC); //Serial.print(" - ");
  //Serial.print("R: "); Serial.print(r, DEC); Serial.print(" ");
  //Serial.print("G: "); Serial.print(g, DEC); Serial.print(" ");
  //Serial.print("B: "); Serial.print(b, DEC); Serial.print(" ");
  //Serial.print("C: "); Serial.print(c, DEC); Serial.print(" ");
  //Serial.println(" ");
  lcd.setCursor(0,0);
  lcd.print("R:");
  lcd.print(r);
  lcd.print("  ");
  lcd.setCursor(8,0);
  lcd.print("G:");
  lcd.print(g);
  lcd.print("   ");
  lcd.setCursor(0,1);
  lcd.print("B:");
  lcd.print(b);
  lcd.print("   ");
}
void tampil_all_sensor(){
  read_sensor();
  lcd.setCursor(0,0);
  lcd.print(s0);
  lcd.print(s1);
  lcd.print(s2);
  lcd.print(s3);
  lcd.print(s4);
  lcd.print(s5);
  lcd.print(s6);
  lcd.print(s7);
  lcd.print(s8);
  lcd.print(s9);
  lcd.setCursor(0,1);
  lcd.print(xsensor);
  lcd.print("   ");
}
void tampil_raw_sensor(){
  lcd.setCursor(0,0);
  lcd.print(sensor_now[1]);
  lcd.setCursor(4,0);
  lcd.print(sensor_now[2]);
  lcd.setCursor(8,0);
  lcd.print(sensor_now[3]);
  lcd.setCursor(12,0);
  lcd.print(sensor_now[4]);
  lcd.setCursor(0,1);
  lcd.print(sensor_now[5]);
  lcd.setCursor(4,1);
  lcd.print(sensor_now[6]);
  lcd.setCursor(8,1);
  lcd.print(sensor_now[7]);
  lcd.setCursor(12,1);
  lcd.print(sensor_now[8]);
}
void kalibrasi2(){
  sensing_kalibrasi();
  tampil_raw_sensor();
  for(int i=0;i<=9;i++){
    if(sensor_now[i]>maks[i]){
      maks[i]=sensor_now[i];
    }
    else if(sensor_now[i]<minimal[i]){
      minimal[i]=sensor_now[i];
    }
    ref[i]=(((maks[i]-minimal[i])/2)+minimal[i])/4;
    EEPROM.write(i,ref[i]); 
  }
}
void inverse_pid(unsigned char simpan){
  unsigned int kec_maks=200;
  unsigned int kec_min=90;
  int kp=kons_p;    //14  9.6      13.2
  int kd=kons_d;   //76  18.3     250    230
  int ki=kons_i;    //146.5              //300        1130       1730    750                                                         
  static int error,lastError,Error,LastError,SumError,right_speed,left_speed;
             
  read_sensor();                
  switch(xsensor){
  case 254 : error = -7; break;
  case 252 : error = -6; break;
  case 253 : error = -5; break;
  case 249 : error = -4; break;
  case 251 : error = -3; break;
  case 243 : error = -2; break;
  case 247 : error = -1; break;
  case 0b00011000 : error = 0; break;
  case 239 : error =  1; break;
  case 207 : error =  2; break;
  case 223 : error =  3; break;
  case 159 : error =  4; break;
  case 191 : error =  5; break;
  case 63  : error =  6; break;
  case 127 : error =  7; break;
  case 255:  
    if(flag==0){error=8;}
    else{error=-8;} 
      break;
    } 
  int SetPoint = 0;                      // Setpoint yang diinginkan
  Error = SetPoint - error; 
  int outPID = kp*0.1*Error + kd*(Error - lastError) + ki*0.1;
  lastError = Error;
  
  double motorKi = simpan - outPID;     // Motor Kiri
  double motorKa = simpan + outPID;     // Motor Kanan
  
  /*** Pembatasan kecepatan ***/
  if (motorKi > kec_maks)motorKi = kec_maks;
  if (motorKi < kec_min)motorKi = kec_min;
  if (motorKa > kec_maks)motorKa = kec_maks;
  if (motorKa < kec_min)motorKa = kec_min;
   
  if(motorKi==motorKa){  
    maju(simpan,simpan);
  }
  else if(motorKi>motorKa){
    bel_ka(motorKi,motorKa);
  }
  else if(motorKa>motorKi){
    bel_ki(motorKi,motorKa);
  }
}
void pid(unsigned char simpan) {
  unsigned int kec_maks = 180;
  unsigned int kec_min = 60;
  int kp = kons_p;  //14  9.6      13.2
  int kd = kons_d; //76  18.3     250    230
  int ki = kons_i;  //146.5              //300        1130       1730    750
  static int error, lastError, Error, LastError, SumError, right_speed, left_speed;

  read_sensor();
  switch (xsensor) {
    case 1 : error = -7; break;
    case 3 : error = -6; break;
    case 2 : error = -5; break;
    case 6 : error = -4; break;
    case 4 : error = -3; break;
    case 12 : error = -2; break;
    case 8 : error = -1; break;
    case 24 : error = 0; break;
    case 16 : error =  1; break;
    case 48 : error =  2; break;
    case 32 : error =  3; break;
    case 96 : error =  4; break;
    case 64 : error =  5; break;
    case 192 : error =  6; break;
    case 128 : error =  7; break;
    case 0:
      if (flag == 0) {
        error = 8;
      }
      else {
        error = -8;
      }
      break;
  }
  int SetPoint = 0;                      // Setpoint yang diinginkan
  Error = SetPoint - error;
  int outPID = kp * 0.1 * Error + kd * (Error - lastError) + ki * 0.1;
  lastError = Error;

  double motorKi = simpan - outPID;     // Motor Kiri
  double motorKa = simpan + outPID;     // Motor Kanan

  /*** Pembatasan kecepatan ***/
  if (motorKi > kec_maks)motorKi = kec_maks;
  if (motorKi < kec_min)motorKi = kec_min;
  if (motorKa > kec_maks)motorKa = kec_maks;
  if (motorKa < kec_min)motorKa = kec_min;

  if (motorKi == motorKa) {
    maju(simpan, simpan);
  }
  else if (motorKi > motorKa) {
    bel_ka(motorKi, motorKa);
  }
  else if (motorKa > motorKi) {
    bel_ki(motorKi, motorKa);
  }
}
void putus_putus(int a){
  for(i=0;i<=a;i++){
  while(1){
  pid(kecepatan);
  if(!s1 && !s2 && !s3 && !s4 && !s5 && !s6)
    break;
  }
  rem();delay(10);
  while(1){
    maju(100,100);
    read_sensor();
    if(s1 || s2 || s3 || s4 || s5 || s6)
    break;
  }
  }
}
void pid_timer(int waktu){
  for(i=0;i<=waktu;i++){
    for(j=0;j<=50;j++){
      pid(kecepatan);
    }
  }
  //stopped();//mundur(0,255);delay(150);//
}
void taruh_objek(){
  while(1){
  pid(kecepatan);
  if((s1 && s8) || (s3 && s4 && s5 && s6) || (s2 && s7))
    break;
  }
  mundur(100,100);delay(50);rem();delay(50);
  stopped();
  servo_on();
  gripper_turun();
  delay(600);
  gripper_buka();
  delay(600);
  gripper_angkat();
  delay(600);
}
void balik_kanan(){
  mundur(80,80);
  delay(150);
  bel_ka(85,115);
  delay(220);  
  rem();delay(50);
  while(1){
    bel_ka(75,100);
    read_sensor();
    if(s5 || s6)
    break;
  }
  rem();delay(50);
}
void finished(){
  while(1){
  pid(100);
  if(!s1 && !s2 && !s3 && !s4 && !s5 && !s6 && !s7 && !s8)
    break;
  }
  rem();delay(150);stopped(); 
}
void maju_awal(){
  while(1){
    maju(100,100);
    read_sensor();
    if(s1 || s8)break;
  }
  while(1){
    maju(100,100);
    read_sensor();
    if(s0 || s9)break;
  }
  while(1){
    bel_ki(145,120);
    read_sensor();
    if(s1 || s2)break;
  }
  rem();delay(100);
}
void per4an_lurus(int jumlah){
  for(int i=0; i<jumlah; i++){
    while(1){
      pid(kecepatan);
      if(s1 || s8)break;
    }
    //rem();delay(20);
    while(1){
      maju(100,100);
      read_sensor();
      if(s0 || s9)break;
    }  
    //rem();delay(20);//delay(100);
    //maju(100,100); delay(50);
  }
  //break;
}
void belok_kanan_objek(){
  bel_ka(70,190);
  delay(150);
  while(1){
    bel_ka(70,190);
    read_sensor();
    if(s6)break;
  }
  rem();delay(100);
}
void belok_kiri_objek(){
  bel_ki (140,60);
  delay(150);
  while(1){
    bel_ki(120,100);
    read_sensor();
    if(s4)break;
  }
  rem();delay(100);
}
void belok_kanan(){
  bel_ka(100,190);
  delay(100);
  while(1){
    bel_ka(100,190);
    read_sensor();
    if(s8)break;
  }
  rem();delay(100);
}
void belok_kiri(){
  bel_ki(190,100);
  delay(30);
  while(1){
    bel_ki(190,100);
    read_sensor();
    if(s1)break;
  }
  rem();delay(100);
}
void ambil_objek(){
  while(1){
  pid(80);
  if((s1 && s8) || (s2 && s7) || (s3 && s6))
    break;
  }
  rem();delay(50);stopped();
  maju(70,70);
  delay(140);
  stopped2();
  stopped();
  delay(100);
  servo_on();
  gripper_tutup();
  delay(400);
  while(1){
    baca_color();
    tampil_color();
    identifikasi_color();
    lcd.setCursor(8,1);
    lcd.print(warna);
    if(warna=='K'){
      break; 
    }
    else if(warna=='M'){
      break;
    }
    else if(warna=='I'){
      break;
    }
  }
}
void satu_ke_kuning(){
  while(1){
    mundur(100,100);
    read_sensor();
    if(s1 || s8)break;
  }
  //mundur(100,100);
  //delay(65);
  while(1){
    bel_ka(80,160);
    read_sensor();
    if(s9)break;
  }
  stopped2();
  while(1){
    bel_ka(80,140);
    read_sensor();
    if(s5)break;
  }
  //stopped2();
  while(1){
    pid(kecepatan);
    if(s1||s8)break;
  }
  maju(120,120);
  delay(120);
  per4an_lurus(2);
  //stopped2();
  mundur(120,120);
  delay(85);
  stopped();
  belok_kanan_objek();
  while(1){
    pid(90);
    read_sensor();
    if(s2 && s7)break;
  }
  stopped2();
  stopped();
  delay(500);
  servo_on();
  gripper_turun();
  delay(400);
  gripper_buka();
  delay(300);
  gripper_angkat();
  delay(400);
  servo_off();
  mundur(80,80);
  delay(80);
}
void satu_ke_merah(){
  while(1){
    mundur(100,100);
    read_sensor();
    if(s1 || s8)break;
  }
  //mundur(100,100);
  //delay(65);
  while(1){
    bel_ka(80,160);
    read_sensor();
    if(s9)break;
  }
  while(1){
    bel_ka(80,140);
    read_sensor();
    if(s5)break;
  }
  while(1){
    pid(kecepatan);
    if(s1||s8)break;
  }
  maju(120,120);
  delay(120);
  per4an_lurus(1);
  belok_kiri();
  per4an_lurus(1);
  while(1){
    maju(80,80);
    read_sensor();
    if(s1 || s8)break;
  }
  stopped();
  delay(1000);
  servo_on();
  gripper_turun();
  delay(700);
  gripper_buka();
  delay(300);
  gripper_angkat();
  delay(700);
  servo_off();
}
void kuning_ke_dua(){
  servo_on();
  gripper_angkat();
  delay(200);
  mundur(100,100);
  delay(50);
  while(1){
    mundur(100,100);
    read_sensor();
    if(!s8)break;
  }
  while(1){
    bel_ka(100,120);
    read_sensor();
    if(s8)break;
  }
  rem();delay(10);
  while(1){
    bel_ka(80,140);
    read_sensor();
    if(s1)break;
  }
  rem();delay(10);
  while(1){
    bel_ka(80,140);
    read_sensor();
    if(s7)break;
  }
  rem();delay(10);
  per4an_lurus(1);
  belok_kiri();
  per4an_lurus(2);
  stopped2();
  while(1){
    bel_ka(90,90);
    read_sensor();
    if(s5 || s3 || s4)break;
  }
  //delay(20);*/
  rem();delay(50);
  stopped();
  servo_on();
  gripper_turun();
  delay(400);
  servo_off();
 // pid_timer(1);
  ambil_objek();
}
void dua_ke_merah(){
  while(1){
    mundur(100,100);
    read_sensor();
    if(s1 || s8)break;
  }
  stopped2();
  while(1){
    bel_ka(80,160);
    read_sensor();
    if(s9)break;
  }
  stopped2();
  while(1){
    bel_ka(80,140);
    read_sensor();
    if(s5||s4)break;
  }
  stopped2();
  while(1){
    pid(kecepatan);
    if(s1||s8)break;
  }
  pid_timer(2);
  per4an_lurus(1);
  stopped2();
  belok_kiri_objek();
  while(1){
    pid(90);
    read_sensor();
    if(s2 && s7)break;
  }
  stopped2();
  stopped();
  delay(1000);
  servo_on();
  gripper_turun();
  delay(400);
  gripper_buka();
  delay(300);
  gripper_angkat();
  delay(400);
  servo_off();
}
void merah_ke_tiga(){
  servo_on();
  gripper_angkat();
  delay(200);
  mundur(100,100);
  delay(50);
  while(1){
    mundur(100,100);
    read_sensor();
    if(s9 || s0)break;
  }
  while(1){
    bel_ki(140,100);
    read_sensor();
    if(s1)break;
  }
  rem();delay(20);
  while(1){
    bel_ki(140,80);
    read_sensor();
    if(s4)break;
  }
  rem(); delay(20);
  per4an_lurus(1);
  belok_kanan();
  per4an_lurus(1);
  rem();delay(100);
  belok_kiri();
  rem();delay(50);
  while(1){
    bel_ka(90,90);
    read_sensor();
    if(s5 || s3 || s4)break;
  }
  stopped2();
  //stopped();
  //delay(100);
  servo_on();
  //pid_timer(1);
  maju(100,100);delay(50);bel_ka(100,100);delay(50);
  gripper_turun();
  delay(50);
  servo_off();
  ambil_objek();
}
void tiga_ke_hijau(){
  while(1){
    mundur(100,100);
    read_sensor();
    if(s1 || s8)break;
  }
  mundur(100,100);
  delay(350);
  while(1){
    bel_ka(80,160);
    read_sensor();
    if(s7)break;
  }
  rem();delay(20);
  per4an_lurus(1);
  belok_kanan();
  per4an_lurus(3);
  stopped2();
  while(1){
    pid(90);
    read_sensor();
    if(s2 && s7)break;
  }
  stopped2();
  stopped();
  delay(100);
  while(1){
    mundur(100,100);
    read_sensor();
    if(!s2 && !s7)break;
  }
  maju(100,100);
  delay(100);
  while(1){
    maju(100,100);
    read_sensor();
    if(s2 && s7)break;
  }
  stopped2();
  stopped();
  delay(100);
  servo_on();
  gripper_turun();
  delay(400);
  gripper_buka();
  delay(300);
  gripper_angkat();
  delay(400);
  servo_off();
}

//////KUMPULAN FUNGSI PIRAMID////////
//////KUMPULAN FUNGSI PIRAMID////////
//////KUMPULAN FUNGSI PIRAMID////////
void hijau_ke_tiga_piramid(){
  servo_on();
  gripper_angkat();
  delay(200);
  mundur(100,100);
  delay(50);
  while(1){
    mundur(100,100);
    read_sensor();
    if(s9 || s0)break;
  }
  while(1){
    bel_ki(160,80);
    read_sensor();
    if(s1)break;
  }
  rem();delay(20);
  while(1){
    bel_ki(160,80);
    read_sensor();
    if(s5)break;
  }
  stopped();delay(50);maju(80,80);delay(100);
  per4an_lurus(1);
  belok_kanan();
  per4an_lurus(2);
  belok_kanan();
  per4an_lurus(1);
  belok_kiri();
  rem();delay(20);
  while(1){
    bel_ka(90,90);
    read_sensor();
    if(s5 || s3 || s4)break;
  }
  rem();delay(50);
  servo_on();
  gripper_turun();
  delay(400);
  servo_off();
  ambil_objek();
  delay(700);
  
}
void tiga_ke_hijau_piramid(){
  servo_on();
  gripper_angkat();
  delay(200);
  maju(100,100); delay(80);  
  while(1){
    mundur(100,100);
    read_sensor();
    if(s1 || s8)break;
  }
  //maju(100,100);
  //delay(65);
 /* while(1){
    bel_ka(80,160);
    read_sensor();
    if(s9)break;
  }*/
  stopped2();
  while(1){
    bel_ka(80,140);
    read_sensor();
    if(s5)break;
  }
    while(1){
    bel_ki(140,80);
    read_sensor();
    if(s1||s2)break;
  }
  per4an_lurus(1);
  belok_kanan();
  per4an_lurus(1);
  belok_kiri();
  per4an_lurus(2);
  belok_kiri();
  per4an_lurus(1);
  belok_kiri_objek();
  while(1){
    pid(90);
    read_sensor();
    if(s2 && s7)break;
  }
  stopped2();
  stopped();
  delay(100);
  while(1){
    mundur(100,100);
    read_sensor();
    if(!s2 && !s7)break;
  }
  while(1){
    maju(100,100);
    read_sensor();
    if(s1 && s8)break;
  }
  stopped2();
  stopped();
  delay(100);
  servo_on();
  gripper_turun_piramid();
  delay(400);
  gripper_buka();
  delay(300);
  gripper_angkat();
  delay(400);
  servo_off();
}
void hijau_ke_dua_piramid(){
  servo_on();
  gripper_angkat();
  delay(200);
  mundur(100,100);
  delay(50);
  while(1){
    mundur(100,100);
    read_sensor();
    if(s9 || s0)break;
  }
  while(1){
    bel_ka(80,140);
    read_sensor();
    if(s8)break;
  }
  rem();delay(20);
  while(1){
    bel_ka(80,140);
    read_sensor();
    if(s5)break;
  }
  rem(); delay(20);
  per4an_lurus(1);
  belok_kiri();
  per4an_lurus(2);
  rem();delay(20);
  servo_on();
  gripper_turun();
  delay(500);
  maju(80,80);delay(15);
  servo_off();
  ambil_objek();
  delay(700);
}
void dua_ke_merah_piramid(){
  servo_on();
  gripper_angkat();
  delay(200);
  while(1){
    mundur(100,100);
    read_sensor();
    if(s1 || s8)break;
  }
  mundur(100,100);
  delay(65);
  while(1){
    bel_ka(80,160);
    read_sensor();
    if(s9)break;
  }
  stopped2();
  while(1){
    bel_ka(80,140);
    read_sensor();
    if(s5)break;
  }
  stopped2();
  //pid_timer(1);
  per4an_lurus(1);
  belok_kiri();
  per4an_lurus(1);
  belok_kanan();
  per4an_lurus(4);
  belok_kanan();
  while(1){
    pid(kecepatan);
    if(s1||s8)break;
  }
  maju(120,120);
  delay(120);
  while(1){
    pid(70);
    read_sensor();
    if(s1 && s8)break;
  }
  stopped2();
  stopped();
  delay(200);
  servo_on();
  gripper_turun_piramid();
  delay(400);
  gripper_buka();
  delay(300);
  gripper_angkat();
  delay(400);
  servo_off();
  }
void merah_ke_satu_piramid(){
  servo_on();
  gripper_angkat();
  delay(200);
  mundur(100,100);
  delay(50);
  while(1){
    mundur(100,100);
    read_sensor();
    if(s9 || s0)break;
  }
  while(1){
    bel_ka(80,140);
    read_sensor();
    if(s8)break;
  }
  rem();delay(20);
  while(1){
    bel_ka(80,140);
    read_sensor();
    if(s5)break;
  }
  rem(); delay(20);
  per4an_lurus(4);
  rem();delay(20);
  servo_on();
  gripper_turun();
  delay(400);
  servo_off();
  ambil_objek();
  delay(700);
  }
void satu_ke_kuning_piramid(){
  servo_on();
  gripper_angkat();
  delay(200);
  while(1){
    mundur(100,100);
    read_sensor();
    if(s1 || s8)break;
  }
  //mundur(100,100);
  //delay(65);
  while(1){
    bel_ka(80,160);
    read_sensor();
    if(s9)break;
  }
  stopped2();
  while(1){
    bel_ka(80,140);
    read_sensor();
    if(s5)break;
  }
  //stopped2();
  per4an_lurus(4);
  belok_kiri();
  //per4an_lurus(1);
  while(1){
    pid(kecepatan);
    if(s1||s8)break;
  }
  maju(120,120);
  delay(120);
  while(1){
    pid(70);
    read_sensor();
    if(s1 || s8)break;
  }
  stopped2();
  stopped();
  delay(200);
  servo_on();
  gripper_turun_piramid();
  delay(400);
  gripper_buka();
  delay(300);
  gripper_angkat();
  delay(400);
  servo_off();
  }
void kuning_ke_finish_piramid(){
  servo_on();
  gripper_angkat();
  delay(200);
  mundur(100,100);
  delay(50);
  while(1){
    mundur(100,100);
    read_sensor();
    if(!s8)break;
  }
  while(1){
    bel_ki(100,120);
    read_sensor();
    if(s1)break;
  }
  rem();delay(10);
  while(1){
    bel_ki(80,140);
    read_sensor();
    if(s8)break;
  }
  rem();delay(10);
  while(1){
    bel_ki(80,140);
    read_sensor();
    if(s2)break;
  }
  rem();delay(10);
  stopped2();
  belok_kiri();
  per4an_lurus(1);
  rem();delay(10);
  while(1){
    bel_ka(80,140);
    read_sensor();
    if(s4||s5)break;
  }
  rem();delay(10);
  per4an_lurus(3);
  belok_kanan();
  per4an_lurus(1);
  bel_ki(80,140); delay(180);
  maju(80,80);delay(180);
  }      
void selesai(){
  digitalWrite(led_hijau,LOW);
  gripper_turun();
  servo_off();
  finished();
  while(1){
    stopped();
  }
}

void setup() {
  //Serial.begin(9600);
  if (tcs.begin()) {
    //Serial.println("Found sensor");
  } /*else {
    //Serial.println("No TCS34725 found ... check your connections");
    while (1);
  }*/
//  lcd.begin();
  lcd.begin(16,2);
  pinMode (inh, OUTPUT);
  pinMode (selA, OUTPUT);
  pinMode (selB, OUTPUT);
  pinMode (selC, OUTPUT);
  pinMode (rem_ka, OUTPUT);
  pinMode (dir_ka, OUTPUT);
  pinMode (ena_ka, OUTPUT);
  pinMode (rem_ki, OUTPUT);
  pinMode (dir_ki, OUTPUT);
  pinMode (ena_ki, OUTPUT);
  pinMode (button1, INPUT_PULLUP); 
  pinMode (button2, INPUT_PULLUP);
  pinMode (button3, INPUT_PULLUP);
  pinMode (led_hijau, OUTPUT);  //active HIGH
  pinMode (obstacle, INPUT);
  servo_on();
  delay(500);
  gripper_buka();
  delay(1000);
  gripper_angkat();
  delay(1000);
}

void loop(){
  awal:
  lcd.setCursor(0,0);
  lcd.print("1.Setting ");
  lcd.setCursor(0,1);
  lcd.print("2.Sensor "); 
  lcd.setCursor(10,0);
  lcd.print("3.Play");
  baca_button();
  while(tombol1 && tombol2 && tombol3){baca_button();}//tampil_sensor();} 
  if(!tombol1){
    delay(200);
    lcd.clear();
    delay(200);
    while(!tombol1){
      baca_button();
      lcd.setCursor(0,0);
      lcd.print("1.Kalib ");
      lcd.setCursor(0,1);
      lcd.print("2.Speed ");
      lcd.setCursor(10,0);
      lcd.print("3.PID");
    }
    while(1){
      while(tombol1 && tombol2 && tombol3){baca_button();}
      if(!tombol1){
        lcd.clear();
        delay(200);
        while(!tombol1){
          delay(50);
          while(1){
            baca_button();
            kalibrasi2();
            if(!tombol1){
              delay(200);
              goto awal;
            }
          } 
        } 
      }
      else if(!tombol3){
        delay(100);
        kecepatan=EEPROM.read(address_speed);
        laju=kecepatan;
        while(!tombol3){baca_button();}
        lcd.clear();
        while(1){
          lcd.setCursor(0,0);
          lcd.print("Speed:");
          lcd.print(kecepatan);
          lcd.print("  ");
          baca_button();
          if(!tombol1){
            delay(200);
            laju++;
            if(laju>255){laju=0;}
            lcd.print(laju);
            lcd.print("  ");
          }
          else if(!tombol3){
            delay(200);
            laju--;
            if(laju<0){laju=255;}
            lcd.print(laju);
            lcd.print("  ");
          }
          else if(!tombol2){
            delay(200);
            lcd.clear();
            EEPROM.write(address_speed,laju);
            delay(500);
            lcd.setCursor(0,0);
            lcd.print("OK");
            delay(500);
            lcd.clear();
            goto awal;
          }
        }  
      }
      else if(!tombol2){
        delay(100);
        lcd.clear();
        delay(200);
        kons_p=EEPROM.read(address_kons_p);
        kons_d=EEPROM.read(address_kons_d);
        kons_i=EEPROM.read(address_kons_i);
        pi=kons_p;
        di=kons_d;
        ai=kons_i;
        lcd.setCursor(0,0);
        lcd.print("Kp:");
        lcd.print(pi);  
        lcd.setCursor(0,1);
        lcd.print("Kd:");
        lcd.print(di); 
        lcd.setCursor(8,0);
        lcd.print("Ki:");
        lcd.print(ai);  
        while(!tombol1){baca_button();}
        while(1){
          baca_button();
          if(!tombol1 && indeks==0){
            delay(200);
            pi++;
            if(pi>255){pi=0;}
            lcd.setCursor(3,0);
            lcd.print(pi);
            lcd.print("  ");
          }
          else if(!tombol3 && indeks==0){
            delay(200);
            pi--;
            if(pi<0){pi=255;}
            lcd.setCursor(3,0);
            lcd.print(pi);
            lcd.print("  ");
          }
          else if(!tombol2 && indeks==0){
            delay(200);
            EEPROM.write(address_kons_p,pi);
            delay(500);
            lcd.setCursor(8,1);
            lcd.print("OK");
            delay(500);
            lcd.setCursor(8,1);
            lcd.print("  ");
            indeks=1;
            delay(500);
          }
          while(indeks==1){
          baca_button();
          if(!tombol1 && indeks==1){
            delay(200);
            di++;
            if(di>255){di=0;}
            lcd.setCursor(3,1);
            lcd.print(di);
            lcd.print("  ");
          }
          else if(!tombol3 && indeks==1){
            delay(200);
            di--;
            if(di<0){di=255;}
            lcd.setCursor(3,1);
            lcd.print(di);
            lcd.print("  ");
          }
          else if(!tombol2 && indeks==1){
            delay(200);
            EEPROM.write(address_kons_d,di);
            delay(500);
            lcd.setCursor(8,1);
            lcd.print("OK");
            indeks=2;
            delay(500);
            lcd.setCursor(8,1);
            lcd.print("OK");
            delay(500);
            lcd.setCursor(8,1);
            lcd.print("  ");
          }
          }
          while(indeks==2){
          baca_button();
          if(!tombol1 && indeks==2){
            delay(200);
            ai++;
            if(ai>255){ai=0;}
            lcd.setCursor(11,0);
            lcd.print(ai);
            lcd.print("  ");
          }
          else if(!tombol3 && indeks==2){
            delay(200);
            ai--;
            if(ai<0){ai=255;}
            lcd.setCursor(11,0);
            lcd.print(ai);
            lcd.print("  ");
          }
          else if(!tombol2 && indeks==2){
            delay(200);
            EEPROM.write(address_kons_i,ai);
            delay(500);
            indeks=0;
            lcd.setCursor(8,1);
            lcd.print("OK");
            delay(500);
            lcd.setCursor(8,1);
            lcd.print("  ");
            delay(1000);
            lcd.clear();
            goto awal;
          }
          }
        }
      }
    }
  }
  else if(!tombol3){
    delay(100);
    lcd.clear();
    kons_p=EEPROM.read(address_kons_p);
    kons_d=EEPROM.read(address_kons_d);
    kons_i=EEPROM.read(address_kons_i);
    kecepatan=EEPROM.read(address_speed);
    sens[0]=EEPROM.read(0);
    sens[1]=EEPROM.read(1);
    sens[2]=EEPROM.read(2);
    sens[3]=EEPROM.read(3);
    sens[4]=EEPROM.read(4);
    sens[5]=EEPROM.read(5);
    sens[6]=EEPROM.read(6);
    sens[7]=EEPROM.read(7);
    sens[8]=EEPROM.read(8);
    sens[9]=EEPROM.read(9);
    while(1){
      tampil_all_sensor();
    }  
  }
  else if(!tombol2){
    delay(100);
    kons_p=EEPROM.read(address_kons_p);
    kons_d=EEPROM.read(address_kons_d);
    kons_i=EEPROM.read(address_kons_i);
    kecepatan=EEPROM.read(address_speed);
    sens[0]=EEPROM.read(0);
    sens[1]=EEPROM.read(1);
    sens[2]=EEPROM.read(2);
    sens[3]=EEPROM.read(3);
    sens[4]=EEPROM.read(4);
    sens[5]=EEPROM.read(5);
    sens[6]=EEPROM.read(6);
    sens[7]=EEPROM.read(7);
    sens[8]=EEPROM.read(8);
    sens[9]=EEPROM.read(9);
    lcd.clear();
    delay(100);
    while(!tombol2){
      baca_button();
      lcd.setCursor(0,0);
      lcd.print("1.Start ");
      lcd.setCursor(0,1);
      lcd.print("2.Retry ");  
      while(tombol1 && tombol3){baca_button();}
      if(!tombol1){
        lcd.clear();
        delay(200);
        while(!tombol1){
          baca_button();
          lcd.setCursor(0,0);
          lcd.print("1.Start1 ");
          lcd.setCursor(0,1);
          lcd.print("2.Cek Color ");  //START2
          while(tombol1 && tombol3){baca_button();}
          if(!tombol1){  //ISI PROGRAM START1
            lcd.clear();
            delay(200);
            while(!tombol1){
              digitalWrite(led_hijau,HIGH);
              /////TULIS PROGRAM DISINI UNTUK START1
              maju_awal();
              per4an_lurus(3);
              stopped2();
              stopped();
              belok_kanan(); 
              per4an_lurus(1);
              stopped2();
              stopped();
              belok_kiri();
              per4an_lurus(3);
              stopped2();
              stopped();
              while(1){
                bel_ka(90,90);
                read_sensor();
                if(s5 || s3 || s4)break;
              }
              stopped2();
              stopped();
              servo_on();
              gripper_turun();
              delay(400);
              servo_off();
              ambil_objek();
              if(warna=='K'||'M'||'I'){   //BLOK PERTAMA
                delay(300);
                lcd.clear();
                while(1){
                  gripper_angkat();
                  delay(400);
                  lcd.setCursor(0,0);
                  lcd.print("ke kuning");
                  satu_ke_kuning();
                  kuning_ke_dua();
                  if(warna=='M'||'K'||'I'){     //BLOK KEDUA
                    delay(300);
                    lcd.clear();
                    while(1){
                      gripper_angkat();
                      delay(400);
                      lcd.setCursor(0,0);
                      lcd.print("ke merah");
                      dua_ke_merah();
                      merah_ke_tiga();
                      if(warna=='I'||'K'||'M'){     //BLOK KETIGA
                        delay(300);
                        lcd.clear();
                        while(1){
                          gripper_angkat();
                          delay(400);
                          lcd.setCursor(0,0);
                          lcd.print("ke hijau");
                          tiga_ke_hijau();
                          goto pindah_piramid;
                          while(1){
                            stopped();
                          }
                        }
                      }
                      while(1){
                      stopped();
                    }
                    }
                    while(1){
                      stopped();
                    }
                  }
                  else if(warna=='K'){ //BLOK KEDUA
                    
                  }
                  else if(warna=='I'){
                    
                  }
                }
              }
              else if(warna=='M'||'K'||'I'){  //BLOK PERTAMA
                delay(300);
                lcd.clear();
                while(1){
                  gripper_angkat();
                  delay(400);
                  lcd.setCursor(0,0);
                  lcd.print("ke merah");
                  satu_ke_merah();
                  while(1){
                    stopped();
                  }
                }
              }
              else if(warna=='I'||'K'||'M'){ //BLOK PERTAMA
                
              }
              while(1){
                stopped();   
              }
              pindah_piramid:
              hijau_ke_tiga_piramid();
              if(warna=='I'||'K'||'M'){         // BLOK PERTAMA PIRAMID
                tiga_ke_hijau_piramid();
                hijau_ke_dua_piramid();
              if(warna=='M'||'K'||'M'){         // BLOK KEDUA PIRAMID
                dua_ke_merah_piramid();
                merah_ke_satu_piramid();
                }
                else{
                  while(1){
                  stopped();
                }
                }
              if(warna=='K'||'M'||'I'){        // BLOK KETIGA PIRAMID
                satu_ke_kuning_piramid();
                kuning_ke_finish_piramid();
                }
                  else{
                  while(1){
                  stopped();
                }
                }  
                while(1){
                  stopped();
                }
              }
              else{
                  while(1){
                  stopped();
                }
                }
            }
          }
          ///BERAKHIR DISINI
          else if(!tombol3){  //ISI PROGRAM START2
            lcd.clear();
            delay(200);
            gripper_buka();
            delay(1000);
            gripper_turun();
            while(!tombol3){
              /////TULIS PROGRAM DISINI
              data_obstacle=digitalRead(obstacle);
              baca_color(); 
              tampil_color();
              identifikasi_color();
              lcd.setCursor(8,1);
              lcd.print(warna); 
              lcd.setCursor(12,1);
              lcd.print(data_obstacle);
              ///BERAKHIR DISINI
            }
          }
        }
      }
      else if(!tombol3){
        lcd.clear();
        delay(200);
        while(!tombol3){
          baca_button();
          lcd.setCursor(0,0);
          lcd.print("1.Retry1 ");
          lcd.setCursor(0,1);
          lcd.print("2.Retry2 "); 
          while(tombol1 && tombol3){baca_button();}  
          if(!tombol1){
            lcd.clear();
            delay(200);
            while(!tombol1){
              baca_button();
              lcd.setCursor(0,0);
              lcd.print("1.CP1 ");
              lcd.setCursor(0,1);
              lcd.print("2.CP2 "); 
              lcd.setCursor(8,0);
              lcd.print("3.CP3 ");
              while(tombol1 && tombol2 && tombol3){baca_button();}  
              if(!tombol1){     //program untuk cek poin 1 start 1
                lcd.clear();
                delay(200);
                while(!tombol1){
                  /////TULIS PROGRAM DISINI              
                  //pid(kecepatan);

  per4an_lurus(2);
  belok_kanan();
  per4an_lurus(1);
  belok_kiri();
  rem();delay(20);
  while(1){
    bel_ka(90,90);
    read_sensor();
    if(s5 || s3 || s4)break;
  }
  rem();delay(50);
  servo_on();
  gripper_turun();
  delay(400);
  servo_off();
  ambil_objek();
  delay(700);
  tiga_ke_hijau_piramid();

                
                  ///BERAKHIR DISINI             
                }   
              }
              else if(!tombol3){  //program untuk cek poin 2 start 1
                lcd.clear();
                delay(200);
                while(!tombol3){  
                  /////TULIS PROGRAM DISINI
                  while(1){  
                    bel_ka(100,170);
                    read_sensor();
                    if(s9)break;
                  }
                  while(1){
                    stopped();   
                  }  
                }     
              }
              else if(!tombol2){   //program untuk cek poin 3 start 1
                lcd.clear();
                delay(200);
                while(!tombol2){  
                  /////TULIS PROGRAM DISINI
                  while(1){  
                    bel_ka(100,180);
                    read_sensor();
                    if(s9)break;
                  }
                  while(1){
                    stopped();   
                  }  
                }     
              }
            }
          }
          else if(!tombol3){
            lcd.clear();
            delay(200);
            while(!tombol3){
              baca_button();
              lcd.setCursor(0,0);
              lcd.print("1.CP1 ");
              lcd.setCursor(0,1);
              lcd.print("2.CP2 "); 
              lcd.setCursor(8,0);
              lcd.print("3.CP3 ");
              while(tombol1 && tombol2 && tombol3){baca_button();}  
              if(!tombol1){     //program untuk cek poin 1 start 2
                lcd.clear();
                delay(200);
                while(!tombol1){ 
                  /////TULIS PROGRAM DISINI    
                  maju(140,140);
                  delay(1000);
                  rem();delay(350);
                  while(1){
                    stopped();
                  }
                }   
              }
              else if(!tombol3){  //program untuk cek poin 2 start 2
                lcd.clear();
                delay(200);
                while(!tombol3){  
                  /////TULIS PROGRAM DISINI
    
                }     
              }
              else if(!tombol2){   //program untuk cek poin 3 start 2
                lcd.clear();
                delay(200);
                while(!tombol2){  
                  /////TULIS PROGRAM DISINI   
                   
                }     
              }
            }
          }
        }
      }
    }
  }
}
