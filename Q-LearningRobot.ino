/*
   contact: ajiaryad@gmail.com
*/
//parameter
float Q[145][3];
float S[301][1];
float R[301][1];
int state, next_state, a, a_maxQ;
float alpha = 0.5;
float gamma = 0.9;
float maxQ = -1000.00;
//sensor
#include <Ultrasonic.h>
int N, pin, vcc, gnd, PushB1, PushB2, k1, k2, k3, k4, s1, s2, s3, s4, s5;
int pwr[] = {30, 38, 42, 39, 31, 12, 24, 32, 33, 41, 25, 22, 45};
Ultrasonic ping1(28, 26); // (Trig PIN,Echo PIN)
Ultrasonic ping2(36, 34); // (Trig PIN,Echo PIN)
Ultrasonic ping3(40, 43); // (Trig PIN,Echo PIN)
Ultrasonic ping4(37, 35); // (Trig PIN,Echo PIN)
Ultrasonic ping5(29, 27); // (Trig PIN,Echo PIN)
#include <SharpIR.h>
#define ir1 A5
#define ir2 A4
#define ir3 A3
#define ir4 A2
#define ir5 A1
SharpIR SharpIR1(ir1, 1080);
SharpIR SharpIR2(ir2, 1080);
SharpIR SharpIR3(ir3, 1080);
SharpIR SharpIR4(ir4, 1080);
SharpIR SharpIR5(ir5, 1080);
//softmax
int i, ii;
float T = 24.0;
float P, Nrandom, acak;
float C[3];
int drvr1 = 6;
int drvr2 = 7;
int drvr3 = 8;
int drvr4 = 9;
int mspeed = 100;
int langkah = 1;
int langkah_total = 0;
int trial = 0;
int prev_state = 0;
int prev_asoft;
//reward
float r1, r2, r3, r;
int prev_a, prev_k1, prev_k2, Crash1, Crash2, total_crash;
float r_total = 0;
float r_result = 0;
//RGB LED
#include <RGBLed.h>
RGBLed led(13, 11, 10, COMMON_ANODE);
//baterai
int inBat = A7;
float bat;
//eeprom
#include <EEPROM.h>
int z;
//sdCard
#include <SPI.h>
#include <SD.h>
File myFile;
const int chipSelect = 53;
//Button
#include "ClickButton.h"
int function, tLoop;
const int buttonPin1 = 23;
ClickButton button1(buttonPin1, LOW, CLICKBTN_PULLUP);

//////////////////////////////////////////////////////
void (*resetArduino) (void) = 0;

void setup() {
  Serial.begin(9600);
  pinMode(SS, OUTPUT);
  ///////////////POWER///////////////
  for (pin = 0; pin < 13; pin++) {
    pinMode(pwr[pin], OUTPUT);
  }
  for (vcc = 0; vcc < 6; vcc++) {
    digitalWrite(pwr[vcc], HIGH);
  }
  for (gnd = 6; gnd < 13; gnd++) {
    digitalWrite(pwr[gnd], LOW);
  }

  pinMode(47, INPUT_PULLUP);//PB 2
  pinMode(drvr1, OUTPUT); // IN1 L298N
  pinMode(drvr2, OUTPUT); // IN2 L298N
  pinMode(drvr3, OUTPUT); // IN3 L298N
  pinMode(drvr4, OUTPUT); // IN4 L298N

  /////////////indikator//////////
  led.flash(RGBLed::RED, 100);
  led.flash(RGBLed::GREEN, 100);
  led.flash(RGBLed::BLUE, 100);
  PushB2 = digitalRead(47);
  if (PushB2 == 0) {
    led.setColor(RGBLed::CYAN);
  }
  else {
    led.setColor(RGBLed::MAGENTA);
  }
  //////////button////////////
  button1.debounceTime   = 20;   // Debounce timer in ms
  button1.multiclickTime = 250;  // Time limit for multi clicks
  button1.longClickTime  = 1000; // time until "held-down clicks" register
  /////////////////////////
  Serial.println("-ROBOT SIAP-");
}

void loop() {
  acak = random(10000) / 10000.0; //generate bil random 0-1 (4 angka di belakang koma)
  PushB2 = digitalRead(47); //mode sensor
  button1.Update();// Update button state
  if (button1.clicks != 0) function = button1.clicks;

  if (function == 1) { //jika Push Button ditekan
    delay(1000);
    tLoop = 0;
    while (tLoop != 300) { // jumlah trial otomatis tanpa menekan tombol
      trial++; //trial bertambah 1
      baterai(); //pembacaan baterai
      resetSP();//reset kondisi sensor
      statespace();//sensor membaca kondisi rintangan
      state = N;//memasukkan statespace ke variabel s
      while (Crash1 != 1 && Crash2 != 1 && langkah < 250) { //jalankan robot sampai menabrak/langkah maksimal
        softmax();//robot bergerak dengan fungsi softmax
        notif1();//print status
        statespace();////sensor membaca kondisi rintangan
        next_state = N;//memasukkan statespace ke variabel s'
        reward();//mengambil nilai reward
        hitungmaxQ();//menghitung nilai Q tertinggi pada matriks
        hitungQ();//menghitung nilai Q baru
        state = next_state;//nilai s' menjadi s
      }

      if (T > 0.011) T = T * 0.95; //nilai temperatur berubah
      else {
        T = 0.01;
      }
      notif3(); // print status
      while (k1 < 1 || k2 < 1) { //jauhi rintangan setelah menabrak
        if (Crash2 == 1) putarKn();
        if (Crash1 == 1) putarKr();
        statespace();
      }
      tLoop++;
    }
    berhenti();
  }
  if (function == -1) {
    led.off();
    Serial.println("Ambil Data EEPROM");
    getEEPROM();
    led.fadeOut(RGBLed::RED, 5, 100);
  }
  if (function == -2) {
    led.off();
    Serial.println("Simpan Data EEPROM");
    putEEPROM();
    led.fadeOut(RGBLed::BLUE, 5, 100);
    resetArduino();
  }
  if (function == 2) {
    led.off();
    Sdcard_Q();
  }
  if (function == -3) {
    led.off();
    Serial.println("Hapus Data EEPROM");
    clearEEPROM();
    led.fadeOut(RGBLed::WHITE, 5, 100);
    resetArduino();
  }
  function = 0;
  delay(5);
}

void hitungQ() {
  Q[state][a] = Q[state][a] + alpha * (r + gamma * maxQ - Q[state][a]); //persamaan update nilai Q
}
void hitungmaxQ() {
  for (a_maxQ = 0; a_maxQ <= 2; a_maxQ++) {
    if (maxQ <= Q[next_state][a_maxQ]) { //mencari nilai Q tertinggi pada matriks
      maxQ = Q[next_state][a_maxQ];  //nilai tertinggi diambil
    }
  }
}
void statespace() {
  N = -1;
  while (N < 0) { //jalankan program sampai nilai sensor didapat

    if (PushB2 == 1) { //pemilihan mode sensor ultrasonik/IR

      //BACA NILAI SENSOR IR
      s1 = SharpIR1.distance();
      s2 = SharpIR2.distance();
      s3 = SharpIR3.distance();
      s4 = SharpIR4.distance();
      s5 = SharpIR5.distance();
      if (s1 > 80)s1 = 80;
      if (s2 > 80)s2 = 80;
      if (s3 > 80)s3 = 80;
      if (s4 > 80)s4 = 80;
      if (s5 > 80)s5 = 80;

      //tentukan jarak
      int k_0 = 30; //k=0
      int k_1 = 50; //k=1
      int k_2 = 80; //k=2
      int k_34 = 70; // k3 k4 terbaca

      //INISIALISASI CRASH
      if (s1 <= 10 || s2 <= 10 || (s2 <= 10 && s3 <= 10)) {
        Crash1 = 1; Crash2 = 0;
      }
      else if (s4 <= 10 || s5 <= 10 || (s4 <= 10 && s3 <= 10)) {
        Crash1 = 0; Crash2 = 1;
      }

      //INISIALISASI PARAMETER k1 (ZONA KIRI)
      if (s1 < k_0 || s2 < k_0 || s3 < k_0) { //jika nilai sensor 10cm - 24cm nilai k1=0
        k1 = 0;
      }
      else if ((s1 < k_1 && s1 >= k_0) || (s2 < k_1 && s2 >= k_0)) { //jika nilai sensor 25cm - 39cm nilai k1=1
        k1 = 1;
      }
      else if ((s1 <= k_2 && s1 >= k_1) || (s2 <= k_2 && s2 >= k_1) || (s3 <= k_2 && s3 >= k_1)) { //jika nilai sensor 40cm - 54cm nilai k1=2
        k1 = 2;
      }

      //INISIALISASI PARAMETER k2 (ZONA KANAN)
      if (s3 < k_0 || s4 < k_0 || s5 < k_0) { //jika nilai sensor 10cm - 24cm nilai k2=0
        k2 = 0;
      }
      else if ((s4 < k_1 && s4 >= k_0) || (s5 < k_1 && s5 >= k_0)) { //jika nilai sensor 25cm - 39cm nilai k2=1
        k2 = 1;
      }
      else if ((s3 <= k_2 && s3 >= k_1) || (s4 <= k_2 && s4 >= k_1) || (s5 <= k_2 && s5 >= k_1)) { //jika nilai sensor 40cm - 54cm nilai k2=2
        k2 = 2;
      }

      //INISIALISASI PARAMETER k3 (SEKTOR KIRI)
      if ((s2 < k_34 || s3 < k_34) && s1 >= k_34) { //rintangan di sektor 0
        k3 = 0;
      }
      else if ((s1 < k_34 && s2 < k_34) && s3 >= k_34) { //rintangan di sektor 1
        k3 = 1;
      }
      else if (s1 < k_34 && s2 >= k_34 && s3 >= k_34) { //rintangan di sektor 2
        k3 = 2;
      }
      else if ((s1 <= k_2 && s1 >= k_1) && (s2 <= k_2 && s2 >= k_1)) { //jika nilai sensor 40cm - 54cm nilai k2=2
        k3 = 2;
      }

      //INISIALISASI PARAMETER k4 (SEKTOR KANAN)
      if ((s4 < k_34 || s3 < k_34) && s5 >= k_34) { //rintangan di sektor 0
        k4 = 0;
      }
      else if ((s5 < k_34 && s4 < k_34) && s3 >= k_34) { //rintangan di sektor 1
        k4 = 1;
      }
      else if (s5 < k_34 && s4 >= k_34 && s3 >= k_34) { //rintangan di sektor 2
        k4 = 2;
      }
      else if ((s4 <= k_2 && s4 >= k_1) && (s5 <= k_2 && s5 >= k_1)) { //jika nilai sensor 40cm - 54cm nilai k2=2
        k4 = 2;
      }
    }
    else {

      //BACA NILAI SENSOR ULTRASONIK
      s1 = ping1.Ranging(CM);
      s2 = ping2.Ranging(CM);
      s3 = ping3.Ranging(CM);
      s4 = ping4.Ranging(CM);
      s5 = ping5.Ranging(CM);

      //tentukan jarak
      int k_0 = 35; //k=0
      int k_1 = 60; //k=1
      int k_2 = 101; //k=2
      int k_34 = 85; // k3 k4 terbaca
      int k_crsh = 10;

      //INISIALISASI CRASH
      if (s1 <= k_crsh || s2 <= k_crsh || s3 <= k_crsh) {
        Crash1 = 1; Crash2 = 0;
      }
      else if (s4 <= k_crsh || s5 <= k_crsh) {
        Crash1 = 0; Crash2 = 1;
      }

      //INISIALISASI PARAMETER k1 (ZONA KIRI)
      if (s1 < k_0 || s2 < k_0 || s3 < k_0) { //jika nilai sensor 10cm - 24cm nilai k1=0
        k1 = 0;
      }
      else if ((s1 < k_1 && s1 >= k_0) || (s2 < k_1 && s2 >= k_0) || (s3 < k_1 && s3 >= k_0)) { //jika nilai sensor 25cm - 39cm nilai k1=1
        k1 = 1;
      }
      else if ((s1 < k_2 && s1 >= k_1) || (s2 < k_2 && s2 >= k_1) || (s3 < k_2 && s3 >= k_1)) { //jika nilai sensor 40cm - 54cm nilai k1=2
        k1 = 2;
      }

      //INISIALISASI PARAMETER k2 (ZONA KANAN)
      if (s3 < k_0 || s4 < k_0 || s5 < k_0) { //jika nilai sensor 10cm - 24cm nilai k2=0
        k2 = 0;
      }
      else if ((s4 < k_1 && s4 >= k_0) || (s5 < k_1 && s5 >= k_0) || (s3 < k_1 && s3 >= k_0)) { //jika nilai sensor 25cm - 39cm nilai k2=1
        k2 = 1;
      }
      else if ((s3 < k_2 && s3 >= k_1) || (s4 < k_2 && s4 >= k_1) || (s5 < k_2 && s5 >= k_1)) { //jika nilai sensor 40cm - 54cm nilai k2=2
        k2 = 2;
      }

      //INISIALISASI PARAMETER k3 (SEKTOR KIRI)
      if ((s2 < k_34 || s3 < k_34) && s1 >= k_34) { //rintangan di sektor 0
        k3 = 0;
      }
      else if ((s1 < k_34 && s2 < k_34) && s3 >= k_34) { //rintangan di sektor 1
        k3 = 1;
      }
      else if (s1 < k_34 && s2 >= k_34 && s3 >= k_34) { //rintangan di sektor 2
        k3 = 2;
      }
      else if ((s1 > k_34 && s2 > k_34 && s3 > k_34) || (s5 > k_0 && s4 < k_34 && s3 < k_34 && s2 < k_34 && s1 < k_34)) { //jika nilai sensor 40cm - 54cm nilai k2=2
        k3 = 3;
      }

      //INISIALISASI PARAMETER k4 (SEKTOR KANAN)
      if ((s4 < k_34 || s3 < k_34) && s5 >= k_34) { //rintangan di sektor 0
        k4 = 0;
      }
      else if ((s5 < k_34 && s4 < k_34) && s3 >= k_34) { //rintangan di sektor 1
        k4 = 1;
      }
      else if (s5 < k_34 && s4 >= k_34 && s3 >= k_34) { //rintangan di sektor 2
        k4 = 2;
      }
      else if (s4 > k_34 && s5 > k_34 && s3 > k_34 || (s5 < k_34 && s4 < k_34 && s3 < k_34 && s2 < k_34 && s1 > k_0)) { //jika nilai sensor 40cm - 54cm nilai k2=2
        k4 = 3;
      }
    }

    //////Memasukkan nilai diskritisasi sensor ke parameter State Matriks Q///////
    if (k1 == 0 && k2 == 0 && k3 == 0 && k4 == 0) {
      N = 1;
    }
    else if (k1 == 0 && k2 == 0 && k3 == 0 && k4 == 1) {
      N = 2;
    }
    else if (k1 == 0 && k2 == 0 && k3 == 0 && k4 == 2) {
      N = 3;
    }
    else if (k1 == 0 && k2 == 0 && k3 == 0 && k4 == 3) {
      N = 4;
    }
    else if (k1 == 0 && k2 == 0 && k3 == 1 && k4 == 0) {
      N = 5;
    }
    else if (k1 == 0 && k2 == 0 && k3 == 1 && k4 == 1) {
      N = 6;
    }
    else if (k1 == 0 && k2 == 0 && k3 == 1 && k4 == 2) {
      N = 7;
    }
    else if (k1 == 0 && k2 == 0 && k3 == 1 && k4 == 3) {
      N = 8;
    }
    else if (k1 == 0 && k2 == 0 && k3 == 2 && k4 == 0) {
      N = 9;
    }
    else if (k1 == 0 && k2 == 0 && k3 == 2 && k4 == 1) {
      N = 10;
    }
    else if (k1 == 0 && k2 == 0 && k3 == 2 && k4 == 2) {
      N = 11;
    }
    else if (k1 == 0 && k2 == 0 && k3 == 2 && k4 == 3) {
      N = 12;
    }
    else if (k1 == 0 && k2 == 0 && k3 == 3 && k4 == 0) {
      N = 13;
    }
    else if (k1 == 0 && k2 == 0 && k3 == 3 && k4 == 1) {
      N = 14;
    }
    else if (k1 == 0 && k2 == 0 && k3 == 3 && k4 == 2) {
      N = 15;
    }
    else if (k1 == 0 && k2 == 0 && k3 == 3 && k4 == 3) {
      N = 16;
    }
    else if (k1 == 0 && k2 == 1 && k3 == 0 && k4 == 0) {
      N = 17;
    }
    else if (k1 == 0 && k2 == 1 && k3 == 0 && k4 == 1) {
      N = 18;
    }
    else if (k1 == 0 && k2 == 1 && k3 == 0 && k4 == 2) {
      N = 19;
    }
    else if (k1 == 0 && k2 == 1 && k3 == 0 && k4 == 3) {
      N = 20;
    }
    else if (k1 == 0 && k2 == 1 && k3 == 1 && k4 == 0) {
      N = 21;
    }
    else if (k1 == 0 && k2 == 1 && k3 == 1 && k4 == 1) {
      N = 22;
    }
    else if (k1 == 0 && k2 == 1 && k3 == 1 && k4 == 2) {
      N = 23;
    }
    else if (k1 == 0 && k2 == 1 && k3 == 1 && k4 == 3) {
      N = 24;
    }
    else if (k1 == 0 && k2 == 1 && k3 == 2 && k4 == 0) {
      N = 25;
    }
    else if (k1 == 0 && k2 == 1 && k3 == 2 && k4 == 1) {
      N = 26;
    }
    else if (k1 == 0 && k2 == 1 && k3 == 2 && k4 == 2) {
      N = 27;
    }
    else if (k1 == 0 && k2 == 1 && k3 == 2 && k4 == 3) {
      N = 28;
    }
    else if (k1 == 0 && k2 == 1 && k3 == 3 && k4 == 0) {
      N = 29;
    }
    else if (k1 == 0 && k2 == 1 && k3 == 3 && k4 == 1) {
      N = 30;
    }
    else if (k1 == 0 && k2 == 1 && k3 == 3 && k4 == 2) {
      N = 31;
    }
    else if (k1 == 0 && k2 == 1 && k3 == 3 && k4 == 3) {
      N = 32;
    }
    else if (k1 == 0 && k2 == 2 && k3 == 0 && k4 == 0) {
      N = 33;
    }
    else if (k1 == 0 && k2 == 2 && k3 == 0 && k4 == 1) {
      N = 34;
    }
    else if (k1 == 0 && k2 == 2 && k3 == 0 && k4 == 2) {
      N = 35;
    }
    else if (k1 == 0 && k2 == 2 && k3 == 0 && k4 == 3) {
      N = 36;
    }
    else if (k1 == 0 && k2 == 2 && k3 == 1 && k4 == 0) {
      N = 37;
    }
    else if (k1 == 0 && k2 == 2 && k3 == 1 && k4 == 1) {
      N = 38;
    }
    else if (k1 == 0 && k2 == 2 && k3 == 1 && k4 == 2) {
      N = 39;
    }
    else if (k1 == 0 && k2 == 2 && k3 == 1 && k4 == 3) {
      N = 40;
    }
    else if (k1 == 0 && k2 == 2 && k3 == 2 && k4 == 0) {
      N = 41;
    }
    else if (k1 == 0 && k2 == 2 && k3 == 2 && k4 == 1) {
      N = 42;
    }
    else if (k1 == 0 && k2 == 2 && k3 == 2 && k4 == 2) {
      N = 43;
    }
    else if (k1 == 0 && k2 == 2 && k3 == 2 && k4 == 3) {
      N = 44;
    }
    else if (k1 == 0 && k2 == 2 && k3 == 3 && k4 == 0) {
      N = 45;
    }
    else if (k1 == 0 && k2 == 2 && k3 == 3 && k4 == 1) {
      N = 46;
    }
    else if (k1 == 0 && k2 == 2 && k3 == 3 && k4 == 2) {
      N = 47;
    }
    else if (k1 == 0 && k2 == 2 && k3 == 3 && k4 == 3) {
      N = 48;
    }
    else if (k1 == 1 && k2 == 0 && k3 == 0 && k4 == 0) {
      N = 49;
    }
    else if (k1 == 1 && k2 == 0 && k3 == 0 && k4 == 1) {
      N = 50;
    }
    else if (k1 == 1 && k2 == 0 && k3 == 0 && k4 == 2) {
      N = 51;
    }
    else if (k1 == 1 && k2 == 0 && k3 == 0 && k4 == 3) {
      N = 52;
    }
    else if (k1 == 1 && k2 == 0 && k3 == 1 && k4 == 0) {
      N = 53;
    }
    else if (k1 == 1 && k2 == 0 && k3 == 1 && k4 == 1) {
      N = 54;
    }
    else if (k1 == 1 && k2 == 0 && k3 == 1 && k4 == 2) {
      N = 55;
    }
    else if (k1 == 1 && k2 == 0 && k3 == 1 && k4 == 3) {
      N = 56;
    }
    else if (k1 == 1 && k2 == 0 && k3 == 2 && k4 == 0) {
      N = 57;
    }
    else if (k1 == 1 && k2 == 0 && k3 == 2 && k4 == 1) {
      N = 58;
    }
    else if (k1 == 1 && k2 == 0 && k3 == 2 && k4 == 2) {
      N = 59;
    }
    else if (k1 == 1 && k2 == 0 && k3 == 2 && k4 == 3) {
      N = 60;
    }
    else if (k1 == 1 && k2 == 0 && k3 == 3 && k4 == 0) {
      N = 61;
    }
    else if (k1 == 1 && k2 == 0 && k3 == 3 && k4 == 1) {
      N = 62;
    }
    else if (k1 == 1 && k2 == 0 && k3 == 3 && k4 == 2) {
      N = 63;
    }
    else if (k1 == 1 && k2 == 0 && k3 == 3 && k4 == 3) {
      N = 64;
    }
    else if (k1 == 1 && k2 == 1 && k3 == 0 && k4 == 0) {
      N = 65;
    }
    else if (k1 == 1 && k2 == 1 && k3 == 0 && k4 == 1) {
      N = 66;
    }
    else if (k1 == 1 && k2 == 1 && k3 == 0 && k4 == 2) {
      N = 67;
    }
    else if (k1 == 1 && k2 == 1 && k3 == 0 && k4 == 3) {
      N = 68;
    }
    else if (k1 == 1 && k2 == 1 && k3 == 1 && k4 == 0) {
      N = 69;
    }
    else if (k1 == 1 && k2 == 1 && k3 == 1 && k4 == 1) {
      N = 70;
    }
    else if (k1 == 1 && k2 == 1 && k3 == 1 && k4 == 2) {
      N = 71;
    }
    else if (k1 == 1 && k2 == 1 && k3 == 1 && k4 == 3) {
      N = 72;
    }
    else if (k1 == 1 && k2 == 1 && k3 == 2 && k4 == 0) {
      N = 73;
    }
    else if (k1 == 1 && k2 == 1 && k3 == 2 && k4 == 1) {
      N = 74;
    }
    else if (k1 == 1 && k2 == 1 && k3 == 2 && k4 == 2) {
      N = 75;
    }
    else if (k1 == 1 && k2 == 1 && k3 == 2 && k4 == 3) {
      N = 76;
    }
    else if (k1 == 1 && k2 == 1 && k3 == 3 && k4 == 0) {
      N = 77;
    }
    else if (k1 == 1 && k2 == 1 && k3 == 3 && k4 == 1) {
      N = 78;
    }
    else if (k1 == 1 && k2 == 1 && k3 == 3 && k4 == 2) {
      N = 79;
    }
    else if (k1 == 1 && k2 == 1 && k3 == 3 && k4 == 3) {
      N = 80;
    }
    else if (k1 == 1 && k2 == 2 && k3 == 0 && k4 == 0) {
      N = 81;
    }
    else if (k1 == 1 && k2 == 2 && k3 == 0 && k4 == 1) {
      N = 82;
    }
    else if (k1 == 1 && k2 == 2 && k3 == 0 && k4 == 2) {
      N = 83;
    }
    else if (k1 == 1 && k2 == 2 && k3 == 0 && k4 == 3) {
      N = 84;
    }
    else if (k1 == 1 && k2 == 2 && k3 == 1 && k4 == 0) {
      N = 85;
    }
    else if (k1 == 1 && k2 == 2 && k3 == 1 && k4 == 1) {
      N = 86;
    }
    else if (k1 == 1 && k2 == 2 && k3 == 1 && k4 == 2) {
      N = 87;
    }
    else if (k1 == 1 && k2 == 2 && k3 == 1 && k4 == 3) {
      N = 88;
    }
    else if (k1 == 1 && k2 == 2 && k3 == 2 && k4 == 0) {
      N = 89;
    }
    else if (k1 == 1 && k2 == 2 && k3 == 2 && k4 == 1) {
      N = 90;
    }
    else if (k1 == 1 && k2 == 2 && k3 == 2 && k4 == 2) {
      N = 91;
    }
    else if (k1 == 1 && k2 == 2 && k3 == 2 && k4 == 3) {
      N = 92;
    }
    else if (k1 == 1 && k2 == 2 && k3 == 3 && k4 == 0) {
      N = 93;
    }
    else if (k1 == 1 && k2 == 2 && k3 == 3 && k4 == 1) {
      N = 94;
    }
    else if (k1 == 1 && k2 == 2 && k3 == 3 && k4 == 2) {
      N = 95;
    }
    else if (k1 == 1 && k2 == 2 && k3 == 3 && k4 == 3) {
      N = 96;
    }
    else if (k1 == 2 && k2 == 0 && k3 == 0 && k4 == 0) {
      N = 97;
    }
    else if (k1 == 2 && k2 == 0 && k3 == 0 && k4 == 1) {
      N = 98;
    }
    else if (k1 == 2 && k2 == 0 && k3 == 0 && k4 == 2) {
      N = 99;
    }
    else if (k1 == 2 && k2 == 0 && k3 == 0 && k4 == 3) {
      N = 100;
    }
    else if (k1 == 2 && k2 == 0 && k3 == 1 && k4 == 0) {
      N = 101;
    }
    else if (k1 == 2 && k2 == 0 && k3 == 1 && k4 == 1) {
      N = 102;
    }
    else if (k1 == 2 && k2 == 0 && k3 == 1 && k4 == 2) {
      N = 103;
    }
    else if (k1 == 2 && k2 == 0 && k3 == 1 && k4 == 3) {
      N = 104;
    }
    else if (k1 == 2 && k2 == 0 && k3 == 2 && k4 == 0) {
      N = 105;
    }
    else if (k1 == 2 && k2 == 0 && k3 == 2 && k4 == 1) {
      N = 106;
    }
    else if (k1 == 2 && k2 == 0 && k3 == 2 && k4 == 2) {
      N = 107;
    }
    else if (k1 == 2 && k2 == 0 && k3 == 2 && k4 == 3) {
      N = 108;
    }
    else if (k1 == 2 && k2 == 0 && k3 == 3 && k4 == 0) {
      N = 109;
    }
    else if (k1 == 2 && k2 == 0 && k3 == 3 && k4 == 1) {
      N = 110;
    }
    else if (k1 == 2 && k2 == 0 && k3 == 3 && k4 == 2) {
      N = 111;
    }
    else if (k1 == 2 && k2 == 0 && k3 == 3 && k4 == 3) {
      N = 112;
    }
    else if (k1 == 2 && k2 == 1 && k3 == 0 && k4 == 0) {
      N = 113;
    }
    else if (k1 == 2 && k2 == 1 && k3 == 0 && k4 == 1) {
      N = 114;
    }
    else if (k1 == 2 && k2 == 1 && k3 == 0 && k4 == 2) {
      N = 115;
    }
    else if (k1 == 2 && k2 == 1 && k3 == 0 && k4 == 3) {
      N = 116;
    }
    else if (k1 == 2 && k2 == 1 && k3 == 1 && k4 == 0) {
      N = 117;
    }
    else if (k1 == 2 && k2 == 1 && k3 == 1 && k4 == 1) {
      N = 118;
    }
    else if (k1 == 2 && k2 == 1 && k3 == 1 && k4 == 2) {
      N = 119;
    }
    else if (k1 == 2 && k2 == 1 && k3 == 1 && k4 == 3) {
      N = 120;
    }
    else if (k1 == 2 && k2 == 1 && k3 == 2 && k4 == 0) {
      N = 121;
    }
    else if (k1 == 2 && k2 == 1 && k3 == 2 && k4 == 1) {
      N = 122;
    }
    else if (k1 == 2 && k2 == 1 && k3 == 2 && k4 == 2) {
      N = 123;
    }
    else if (k1 == 2 && k2 == 1 && k3 == 2 && k4 == 3) {
      N = 124;
    }
    else if (k1 == 2 && k2 == 1 && k3 == 3 && k4 == 0) {
      N = 125;
    }
    else if (k1 == 2 && k2 == 1 && k3 == 3 && k4 == 1) {
      N = 126;
    }
    else if (k1 == 2 && k2 == 1 && k3 == 3 && k4 == 2) {
      N = 127;
    }
    else if (k1 == 2 && k2 == 1 && k3 == 3 && k4 == 3) {
      N = 128;
    }
    else if (k1 == 2 && k2 == 2 && k3 == 0 && k4 == 0) {
      N = 129;
    }
    else if (k1 == 2 && k2 == 2 && k3 == 0 && k4 == 1) {
      N = 130;
    }
    else if (k1 == 2 && k2 == 2 && k3 == 0 && k4 == 2) {
      N = 131;
    }
    else if (k1 == 2 && k2 == 2 && k3 == 0 && k4 == 3) {
      N = 132;
    }
    else if (k1 == 2 && k2 == 2 && k3 == 1 && k4 == 0) {
      N = 133;
    }
    else if (k1 == 2 && k2 == 2 && k3 == 1 && k4 == 1) {
      N = 134;
    }
    else if (k1 == 2 && k2 == 2 && k3 == 1 && k4 == 2) {
      N = 135;
    }
    else if (k1 == 2 && k2 == 2 && k3 == 1 && k4 == 3) {
      N = 136;
    }
    else if (k1 == 2 && k2 == 2 && k3 == 2 && k4 == 0) {
      N = 137;
    }
    else if (k1 == 2 && k2 == 2 && k3 == 2 && k4 == 1) {
      N = 138;
    }
    else if (k1 == 2 && k2 == 2 && k3 == 2 && k4 == 2) {
      N = 139;
    }
    else if (k1 == 2 && k2 == 2 && k3 == 2 && k4 == 3) {
      N = 140;
    }
    else if (k1 == 2 && k2 == 2 && k3 == 3 && k4 == 0) {
      N = 141;
    }
    else if (k1 == 2 && k2 == 2 && k3 == 3 && k4 == 1) {
      N = 142;
    }
    else if (k1 == 2 && k2 == 2 && k3 == 3 && k4 == 2) {
      N = 143;
    }
    else if (k1 == 2 && k2 == 2 && k3 == 3 && k4 == 3) {
      N = 144;
    }
    else if (Crash1 == 1 || Crash2 == 1) {
      N = 0;
    }
  }
}
void softmax() {
  acak = random(10000) / 10000.0; //generate bil random 0-1 (4 angka di belakang koma)
  maxQ = -1000.0;
  for (i = 0; i <= 2; i++) {
    P = exp(Q[state][i] / T) / ((exp(Q[state][0] / T)) + (exp(Q[state][1] / T)) + (exp(Q[state][2] / T))); //persamaan boltzmann distribution
    C[i] = P;
  } //ambil nilai masing-masing perhitungan ke variabel C

  if (isnan(C[0]) || isnan(C[1]) || isnan(C[2])) { //jika perhitungan nilai P mencapai limit
    if ((Q[state][0] + Q[state][1] + Q[state][2]) == 0) { //jika nilai di tabel masih kosong
      a = random(0, 3); //generate aksi random
    }
    else { //jika terdapat nilai, ambil nilai Aksi Terbaik di tabel Q
      for (ii = 0; ii <= 2; ii++) {
        if (maxQ < Q[state][ii]) {
          maxQ = Q[state][ii];
          a = ii;
        }
      }
    }
    led.setColor(RGBLed::YELLOW);
  }
  else { //jika perhitungan tidak limit, ambil fungsi acak sampai T mendekati 0
    if (acak < C[0]) { //jika nilai acak 1-0 lebih kecil dari reward maju, eksekusi maju
      a = 0;
    }
    else if (acak >= C[0] && acak < (C[0] + C[1])) { //jika nilai acak lebih besar dari r maju dan lebih kecil dari r maju + r kiri, eksekusi kiri
      a = 1;
    }
    else if (acak >= (C[0] + C[1])) { //jika nilai acak lebih besar dari r maju + r kiri, eksekusi kanan
      a = 2;
    }
    led.setColor(RGBLed::BLUE);
  }

  switch (a) { //mengeksekusi aksi dari fungsi softmax
    case 0:
      maju();
      break;
    case 1:
      kiri();
      break;
    case 2:
      kanan();
      break;
  }
  delay(20);
  ////////print status//////////
  langkah++;
  led.off();
}
void resetSP() { //reset nilai diskritisasi sensor (sebelum eksekusi per step)
  k1 = 2;
  k2 = 2;
  k3 = 3;
  k4 = 3;
  Crash1 = 0;
  Crash2 = 0;
}
void reward() {
  ////////r1////////
  switch (a) {
    case 0:
      r1 = 0.02; //robot berjalan maju
      break;
    case 1:
      r1 = -0.01; //robot belok ke kiri
      break;
    case 2:
      r1 = -0.01; //robot belok ke kanan
      break;
  }
  ////////r2////////
  if ((k1 - prev_k1) + (k2 - prev_k2) >= 0) { //jika robot menjauhi rintangan
    r2 = 0.02;
  }
  else { //jika robot mendekati rintangan
    r2 = -0.02;
  }
  prev_k1 = k1; //ambil variabel pembacaan sensor sebelumnya
  prev_k2 = k2; //untuk perhitungan r2
  ///////r3////////
  if ((prev_a == 1 && a == 2) || (prev_a == 2 && a == 1)) { //jika robot mengambil langkah zig-zag, reward negatif
    r3 = -0.08;
  }
  else {
    r3 = 0.0;
  }
  prev_a = a; //mengambil nilai aksi sebelumnya

  ///////////Akumulasi Reward///////////////
  if (Crash1 == 1 || Crash2 == 1) { //jika robot menabrak
    r = -10;
  }
  else { //jika robot tidak menabrak
    r = r1 + r2 + r3; //akumulasi reward didapat
  }
  /////////print status///////////////
  if (langkah == 250) {
    r_total = r_total + r + 10;
  }
  else {
    r_total = r_total + r;
  }
}
void berhenti() { //robot berhenti
  analogWrite(drvr1, 0);
  analogWrite(drvr2, 0);
  analogWrite(drvr3, 0);
  analogWrite(drvr4, 0);
}
void maju() { //robot berjalan maju
  analogWrite(drvr1, mspeed * 1.2);
  analogWrite(drvr2, 0);
  analogWrite(drvr3, mspeed);
  analogWrite(drvr4, 0);
}
void putarKn() { //robot berbelok kiri
  analogWrite(drvr1, 0);
  analogWrite(drvr2, mspeed * 1.5);
  analogWrite(drvr3, mspeed * 1.5);
  analogWrite(drvr4, 0);
}
void putarKr() { //robot berbelok kanan
  analogWrite(drvr1, mspeed * 1.5);
  analogWrite(drvr2, 0);
  analogWrite(drvr3, 0);
  analogWrite(drvr4, mspeed * 1.5);
}
void kiri() { //robot berbelok kiri
  analogWrite(drvr1, 0);
  analogWrite(drvr2, 0);
  analogWrite(drvr3, mspeed);
  analogWrite(drvr4, 0);
}
void kanan() { //robot berbelok kanan
  analogWrite(drvr1, mspeed * 1.2);
  analogWrite(drvr2, 0);
  analogWrite(drvr3, 0);
  analogWrite(drvr4, 0);
}
void baterai() {
  int adc = analogRead(inBat);
  bat = adc / 65.4;
}
void notif1() {
  Serial.print("Trial: ");
  Serial.print(trial);
  Serial.print(" | Reward: ");
  Serial.print(r_result, 4);
  Serial.print(" | Step: ");
  Serial.print(langkah_total);
  Serial.print(" | Crash: ");
  Serial.print(total_crash);
  Serial.print("           Step:");
  Serial.print(langkah);
  Serial.print(" | State: ");
  Serial.print(state);
  Serial.print(" | Action: ");
  if (a == 0)Serial.println("Maju");
  else if (a == 1)Serial.println("Kiri");
  else if (a == 2)Serial.println("Kanan");
}
void notif3() {
  led.setColor(RGBLed::RED);
  r_result = r_result + r_total;
  R[trial][0] = r_result;
  r_total = 0;
  langkah_total = langkah_total + (langkah - 1);
  S[trial][0] = langkah - 1;
  langkah = 1;
  total_crash = total_crash + Crash1 + Crash2;
}
void putEEPROM() {
  z = 0;
  for (int y = 1; y < 145; y++) {
    Serial.print("Q[");
    Serial.print(y);
    Serial.print("] = ");
    for (int x = 0; x <= 2; x++) {
      z += sizeof(float);
      EEPROM.put(z, Q[y][x]);
      Serial.print(Q[y][x], 4);
      Serial.print(" ");
    }
    Serial.println("");
  }
  Serial.println("");
  z += sizeof(float);
  EEPROM.put(z, trial);
  Serial.print("Trial: ");
  Serial.println(trial);
  z += sizeof(float);
  EEPROM.put(z, T);
  Serial.print("T: ");
  Serial.println(T);
  z += sizeof(float);
  EEPROM.put(z, r_result);
  Serial.print("Reward Result: ");
  Serial.println(r_result, 4);
  z += sizeof(float);
  EEPROM.put(z, langkah_total);
  Serial.print("Step Result: ");
  Serial.println(langkah_total);
  z += sizeof(float);
  EEPROM.put(z, total_crash);
  Serial.print("Crash Result: ");
  Serial.println(total_crash);
}
void getEEPROM() { //ambil data dari eeprom
  z = 0;
  for (int y = 1; y < 145; y++) {
    Serial.print("Q[");
    Serial.print(y);
    Serial.print("] = ");
    for (int x = 0; x <= 2; x++) {
      z += sizeof(float);
      EEPROM.get(z, Q[y][x]);
      Serial.print(Q[y][x], 4);
      Serial.print(" ");
    }
    Serial.println("");
  }
  Serial.println("");
  z += sizeof(float);
  EEPROM.get(z, trial);
  Serial.print("Trial: ");
  Serial.println(trial);
  z += sizeof(float);
  EEPROM.get(z, T);
  Serial.print("T: ");
  Serial.println(T);
  z += sizeof(float);
  EEPROM.get(z, r_result);
  Serial.print("Reward Result = ");
  Serial.println(r_result, 4);
  z += sizeof(float);
  EEPROM.get(z, langkah_total);
  Serial.print("Step Result = ");
  Serial.println(langkah_total);
  z += sizeof(float);
  EEPROM.get(z, total_crash);
  Serial.print("Crash Result = ");
  Serial.println(total_crash);
}
void clearEEPROM() {
  for (int i = 0 ; i < EEPROM.length() ; i++) {
    EEPROM.write(i, 0);
  }
}

void Sdcard_Q()
{
  Serial.begin(9600);
  pinMode(SS, OUTPUT);

  if (!SD.begin(chipSelect)) {
    Serial.println("initialization failed!");
    led.fadeOut(RGBLed::RED, 5, 100);
    return;
  }
  Serial.println("initialization done.");

  myFile = SD.open("Q_Result.txt", FILE_WRITE);
  if (myFile) {
    Serial.println("Simpan Data Q");
    myFile.println("Data Tabel Q");
    z = 0;
    for (int y = 1; y < 145; y++) {
      myFile.print("{");
      for (int x = 0; x <= 2; x++) {
        z += sizeof(float);
        myFile.print(Q[y][x], 4);
        myFile.print(",");
      }
      myFile.println("},");
    }
    myFile.println("");
    z += sizeof(float);
    myFile.print("Trial: ");
    myFile.println(trial);
    z += sizeof(float);
    myFile.print("T: ");
    myFile.println(T);
    z += sizeof(float);
    myFile.print("Reward Result: ");
    myFile.println(r_result, 4);
    z += sizeof(float);
    myFile.print("Step Result: ");
    myFile.println(langkah_total);
    z += sizeof(float);
    myFile.print("Crash Result: ");
    myFile.println(total_crash);
    myFile.println("");

    Serial.println("Simpan Data Step");
    myFile.println("Data Step");
    for (int y = 1; y < 302; y++) {
      myFile.print(y);
      myFile.print(" ");
      myFile.print(S[y][0]);
      myFile.println("");
    }
    myFile.println("");

    Serial.println("Simpan Data Reward");
    myFile.println("Data Reward");
    for (int y = 1; y < 302; y++) {
      myFile.print(y);
      myFile.print(" ");
      myFile.print(R[y][0], 4);
      myFile.println("");
    }
    myFile.println("");

    myFile.close();
    Serial.println("done.");
    led.fadeOut(RGBLed::BLUE, 5, 100);
  } else {
    Serial.println("Gagal Membuka File Q.txt");
    led.fadeOut(RGBLed::RED, 5, 100);
  }
}
