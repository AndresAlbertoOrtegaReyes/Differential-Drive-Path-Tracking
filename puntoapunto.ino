//
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

// Planificación punto a punto
float xf = -2.0, yf = -1.0;   // PUNTO DESEADO (Final)
float tf = 12.0;               // Tiempo total
float ax0, ax1, ax2, ax3;
float ay0, ay1, ay2, ay3;

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
  t0 = millis()/1000.0;

  x = 0; y = 0; theta = 0; 
  countA = 0; countB = 0;
  cl_old = 0; cr_old = 0;


  // Coeficientes cúbicos
  ax0 = x;           
  ax1 = 0.0;
  ax2 = -3.0*x/(tf*tf) + 3.0*xf/(tf*tf);      
  ax3 = 2.0*x/(tf*tf*tf) - 2.0*xf/(tf*tf*tf); 

  ay0 = y;         
  ay1 = 0.0;
  ay2 = -3.0*y/(tf*tf) + 3.0*yf/(tf*tf);     
  ay3 = 2.0*y/(tf*tf*tf) - 2.0*yf/(tf*tf*tf); 

  Serial.print("Coeficientes X: ");
  Serial.print(ax0); Serial.print(", ");
  Serial.print(ax1); Serial.print(", "); 
  Serial.print(ax2); Serial.print(", ");
  Serial.println(ax3);

  Serial.print("Coeficientes Y: ");
  Serial.print(ay0); Serial.print(", ");
  Serial.print(ay1); Serial.print(", ");
  Serial.print(ay2); Serial.print(", ");
  Serial.println(ay3);
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
  float delta_theta = (dr-dl)/(2*L); // QUITA EL 2 SI NO FUNCIONA
  
  x = x + delta_d*cos(theta); // xi
  y = y + delta_d*sin(theta); // yi
  theta = theta + delta_theta;
  theta = atan2(sin(theta),cos(theta)); // thetai
 
  // Controlador
  float xh = x + D*cos(theta);
  float yh = y + D*sin(theta);

  float xt = (ax3*t*t*t) + (ax2*t*t) + (ax1*t) + ax0;
  float yt = (ay3*t*t*t) + (ay2*t*t) + (ay1*t) + ay0;

  float xdt = 3*(ax3*t*t) + 2*(ax2*t) + ax1;
  float ydt = 3*(ay3*t*t) + 2*(ay2*t) + ay1;

  float ux = xdt + kx*(xt-xh);
  float uy = ydt + ky*(yt-yh);
 
  float v = ux*cos(theta) + uy*sin(theta);
  float w = -(ux/D)*sin(theta) + (uy/D)*cos(theta);

  //float gc = 1900, tc = 25; // Ganancias GAIN/TRIM 2 m/s
  //float gc = 1470, tc = 8; // Ganancias GAIN/TRIM 1 m/s
  float gc = 300, tc = -7; // Ganancias GAIN/TRIM 2 m/s

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

  // Para frenar 
  if (t > tf) {
    digitalWrite(pinAIN2,LOW);
    digitalWrite(pinAIN1,LOW);
    analogWrite(pinPWMA,0);
    digitalWrite(pinBIN2,LOW);
    digitalWrite(pinBIN1,LOW);
    analogWrite(pinPWMB,0);
  }
  
  //for(;;);
}
