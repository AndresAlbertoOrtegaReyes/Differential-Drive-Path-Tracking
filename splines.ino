int pinSTBY = 5;
int pwm_l = 0, pwm_r = 0;

float N = 572.0, R = 0.0335, L = 0.125, t0; // L está a la mitad
float x = 0, y = 0, theta = 0;
int cl_old = 0, cr_old = 0;

// Motor R
int pinPWMA = 21;
int pinAIN2 = 19;
int pinAIN1 = 18;
int interruptPinA = 32;
volatile int countA = 0;

// Motor L
int pinBIN2 = 16;
int pinBIN1 = 17;
int pinPWMB = 4;
int interruptPinB = 25;
volatile int countB = 0;

// SPLINE LINEAL (6 puntos en total)
float x0 = 0.5, y_0 = 0;      // Punto inicial (0)
float x1 = -0.5, y_1 = 0.5;     // Punto 1
float x2 = -0.5, y2 = -0.5;     // Punto 2  
float x3 = 0.5, y3 = -0.5;     // Punto 3
float x4 = 0.5, y4 = 0.5;     // Punto 4
float x5 = 0, y5 = 0;     // Punto final (5)

// Tiempos entre puntos
float t01 = 4.0;  // t0 a t1
float t12 = 5.0;  // t1 a t2
float t23 = 5.0;  // t2 a t3  
float t34 = 5.0;  // t3 a t4
float t45 = 5.0;  // t4 a t5

// Tiempos acumulados
float t1_acum = t01;
float t2_acum = t01 + t12;
float t3_acum = t01 + t12 + t23;
float t4_acum = t01 + t12 + t23 + t34;
float t5_acum = t01 + t12 + t23 + t34 + t45;

// Ganancias del controlador
float kx = 1.2, ky = 1.2;
float D = 0.1;  // Desacoplo

//
void Interrupt_A () {
  if(pwm_r>=0)
    countA++;
  else
    countA--;
}

void Interrupt_B () {
  if(pwm_l>=0)
    countB++;
  else
    countB--;
}

//
void setup() {

  pinMode(pinAIN2,OUTPUT);
  pinMode(pinAIN1,OUTPUT);
  pinMode(pinPWMA,OUTPUT);
  pinMode(pinBIN1,OUTPUT);
  pinMode(pinBIN2,OUTPUT);
  pinMode(pinPWMB,OUTPUT);
  pinMode(pinSTBY,OUTPUT);
  digitalWrite(pinSTBY, HIGH);
  
  attachInterrupt(digitalPinToInterrupt(interruptPinA), Interrupt_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(interruptPinB), Interrupt_B, CHANGE);
  
  Serial.begin(9600);
  
  delay(2000);
 // t0 = millis()/1000.0;
  t0 = millis();

  x = 0; 
  y = 0; 
  theta = 0; 
  countA = 0; 
  countB = 0;
  cl_old = 0; 
  cr_old = 0;
}

//
void loop() {
  unsigned long currentTime = millis();
  float t = (currentTime - t0) / 1000.0;

  // ODOMETRÍA
  int cr = countA, cl = countB;
  
  float delta_cl = cl-cl_old;
  cl_old = cl;
  
  float delta_cr = cr-cr_old;
  cr_old = cr;

  float dl = 2*PI*R*(delta_cl/N);
  float dr = 2*PI*R*(delta_cr/N);
  
  float delta_d = (dr+dl)/2;
  float delta_theta = (dr-dl)/(2*L);
  
  x = x + delta_d*cos(theta);
  y = y + delta_d*sin(theta);
  theta = theta + delta_theta;
  theta = atan2(sin(theta),cos(theta));
 
  // Controlador 
  float xh = x + D*cos(theta);
  float yh = y + D*sin(theta);

  // SPLINE LINEAL
  // Coeficientes
  float fx01 = (x1 - x0) / t01;
  float fy01 = (y_1 - y_0) / t01;
  float fx12 = (x2 - x1) / t12;
  float fy12 = (y2 - y_1) / t12;
  float fx23 = (x3 - x2) / t23;
  float fy23 = (y3 - y2) / t23;
  float fx34 = (x4 - x3) / t34;
  float fy34 = (y4 - y3) / t34;
  float fx45 = (x5 - x4) / t45;
  float fy45 = (y5 - y4) / t45;

  // Ecuaciones
  float xt, yt, xdt, ydt;

  if (t <= t01) {
      xt = x0 + fx01 * (t - 0);
      yt = y_0 + fy01 * (t - 0);
      xdt = fx01;
      ydt = fy01;
  }
  else if (t <= t2_acum) {
      float t1 = t - t01;
      xt = x1 + fx12 * t1;
      yt = y_1 + fy12 * t1;
      xdt = fx12;
      ydt = fy12;
  }
  else if (t <= t3_acum) {
      float t2 = t - t2_acum;
      xt = x2 + fx23 * t2;
      yt = y2 + fy23 * t2;
      xdt = fx23;
      ydt = fy23;
  }
  else if (t <= t4_acum) {
      float t3 = t - t3_acum;
      xt = x3 + fx34 * t3;
      yt = y3 + fy34 * t3;
      xdt = fx34;
      ydt = fy34;
  }
  else if (t <= t5_acum) {
      float t4 = t - t4_acum;
      xt = x4 + fx45 * t4;
      yt = y4 + fy45 * t4;
      xdt = fx45;
      ydt = fy45;
  }
  else {
      xt = x5;
      yt = y5;
      xdt = 0;
      ydt = 0;
  }

  // Controlador
  float ux = xdt + kx*(xt - xh);
  float uy = ydt + ky*(yt - yh);
 
  float v = ux*cos(theta) + uy*sin(theta);
  float w = -(ux/D)*sin(theta) + (uy/D)*cos(theta);
  w=w;

  float gc = 300, tc = -70; // Ganancias GAIN/TRIM

  // Calcula pwm 
  pwm_l = (gc + tc)*(v - w*L);
  pwm_r = (gc - tc)*(v + w*L);
  

  if(pwm_r>=0) {
    if(pwm_r>255)
      pwm_r = 255;
      
    digitalWrite(pinAIN2,HIGH);
    digitalWrite(pinAIN1,LOW);
    analogWrite(pinPWMA,pwm_r);
  } else {
    if(pwm_r<-255)
      pwm_r = -255;

    digitalWrite(pinAIN2,LOW);
    digitalWrite(pinAIN1,HIGH);
    analogWrite(pinPWMA,-pwm_r);
  }

  if(pwm_l>=0) {
     if(pwm_l>255)
      pwm_l = 255;

    digitalWrite(pinBIN2,HIGH);
    digitalWrite(pinBIN1,LOW);
    analogWrite(pinPWMB,pwm_l);
  } else {
    if(pwm_l<-255)
      pwm_l = -255;

    digitalWrite(pinBIN2,LOW);
    digitalWrite(pinBIN1,HIGH);
    analogWrite(pinPWMB,-pwm_l);
  }

  // Frenar cuando se completa la trayectoria
  if (t > t5_acum) {
    digitalWrite(pinAIN2,HIGH);
    digitalWrite(pinAIN1,LOW);
    analogWrite(pinPWMA,0);
    digitalWrite(pinBIN2,HIGH);
    digitalWrite(pinBIN1,LOW);
    analogWrite(pinPWMB,0);
  }
  
}
