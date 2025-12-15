//
int pinSTBY = 5;
int pwm_l = 0, pwm_r = 0;

float N = 572.0, R = 0.0335, L = 0.125, t0;
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

// Parámetros de trayectoria circular
float xc = 0.8;
float yc = 0.0;
float Rc = 0.35;
float Tc = 20.0;

// Ganancias del controlador
float kx = 0.5, ky = 0.5;
float D = 0.1;  // Desacoplo

// --- INTERRUPCIONES ---
void Interrupt_A () {
  if(pwm_r>=0) countA++;
  else countA--;
}

void Interrupt_B () {
  if(pwm_l>=0) countB++;
  else countB--;
}

// --- SETUP ---
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
  t0 = millis();

  x = 0; 
  y = 0; 
  theta = 0; 
  countA = 0; 
  countB = 0;
  cl_old = 0; 
  cr_old = 0;
}

// --- LOOP PRINCIPAL ---
void loop() {

  unsigned long currentTime = millis();
  float t = (currentTime - t0) / 1000.0;

  //        ODOMETRÍA
  int cr = countA, cl = countB;
  float delta_cl = cl - cl_old;   cl_old = cl;
  float delta_cr = cr - cr_old;   cr_old = cr;

  float dl = 2*PI*R*(delta_cl/N);
  float dr = 2*PI*R*(delta_cr/N);

  float delta_d = (dr+dl)/2;
  float delta_theta = (dr-dl)/(2*L);

  x = x + delta_d*cos(theta);
  y = y + delta_d*sin(theta);
  theta = atan2(sin(theta + delta_theta), cos(theta + delta_theta));

  float xh = x + D*cos(theta);
  float yh = y + D*sin(theta);

  //1) TRAYECTORIA CIRCULAR
  /*float w_circ = 2*PI / Tc;
  float ang = w_circ * t;

  float xt = xc + Rc * cos(ang);
  float yt = yc + Rc * sin(ang);

  float xdt = -Rc * sin(ang) * w_circ;
  float ydt =  Rc * cos(ang) * w_circ;*/

 //2) TRAYECTORIA DIENTE DE SIERRA
  float Ax = 0.05;     // velocidad en X (m/s)
  float Ay = 0.6;      // amplitud de oscilación en Y (m)
  float w_s = 0.2;     // frecuencia

  // Punto deseado
  float xt = Ax * t;
  float yt = Ay * sin(w_s * t);

  // Derivadas
  float xdt = Ax;
  float ydt = Ay * w_s * cos(w_s * t);

//5) ESPIRAL DE VELOCIDAD CONSTANTE (Root-Spiral)
  /*float V_const = 0.1;      // Velocidad constante deseada (m/s)
  float Separacion = 0.1;   // Distancia entre cada vuelta de la espiral
  float b = Separacion / (2 * PI); // Coeficiente de crecimiento geometrico

  // Calculamos el angulo en funcion de la raiz del tiempo
  // Para mantener velocidad constante: theta = sqrt(2 * V * t / b)
  float ang = sqrt((2 * V_const * t) / b);
  
  // El radio depende linealmente del angulo actual
  float radio_t = b * ang;

  float xt = radio_t * cos(ang);
  float yt = radio_t * sin(ang);

  float w_inst = V_const / (radio_t + 0.0001); 
  float v_radial = b * w_inst; // Cuanto crece el radio por segundo

  float xdt = v_radial * cos(ang) - radio_t * w_inst * sin(ang);
  float ydt = v_radial * sin(ang) + radio_t * w_inst * cos(ang);*/ 

  float ux = xdt + kx*(xt - xh);
  float uy = ydt + ky*(yt - yh);

  float v = ux*cos(theta) + uy*sin(theta);
  float w = -(ux/D)*sin(theta) + (uy/D)*cos(theta);

  float gc = 300, tc = -7;

  pwm_l = (gc + tc)*(v - w*L);
  pwm_r = (gc - tc)*(v + w*L);

  // Motor R
  if(pwm_r>=0){
    pwm_r = min(pwm_r, 255);
    digitalWrite(pinAIN2, HIGH);  
    digitalWrite(pinAIN1, LOW);
    analogWrite(pinPWMA, pwm_r);
  } else {
    pwm_r = max(pwm_r, -255);
    digitalWrite(pinAIN2, LOW);   
    digitalWrite(pinAIN1, HIGH);
    analogWrite(pinPWMA, -pwm_r);
  }

  // Motor L
  if(pwm_l>=0){
    pwm_l = min(pwm_l, 255);
    digitalWrite(pinBIN2, HIGH);  
    digitalWrite(pinBIN1, LOW);
    analogWrite(pinPWMB, pwm_l);
  } else {
    pwm_l = max(pwm_l, -255);
    digitalWrite(pinBIN2, LOW);   
    digitalWrite(pinBIN1, HIGH);
    analogWrite(pinPWMB, -pwm_l);
  }
}