//Laboratorio Robótica y Animatrónica de la Universidad Nacional de Córdoba.
//Director: Ing. Hugo Pailos
//Autores: Bondaz Pablo Esteban y Torres José Ignacio.
//Contenido: Código para mover un robot planar de 3GDL en base a coordenadas X,Y,Z. con adaptaciones para nuestro caso particular

#include <Servo.h>

// === Pines de servos ===
const int pinBase  = 9;
const int pinBrazo = 10;
const int pinCodo  = 11;
const int pinGarra = 3;
Servo servoBase, servoBrazo, servoCodo, servoGarra;

// === Parámetros geométricos ===
const float l1 = 18.8;
const float l2 = 9.6;
const float l3 = 19.0;
float q1_actual = servoBase.read();
float q2_actual = servoBrazo.read();
float q3_actual = servoCodo.read();
// ----------------------------------------------------------
void setup() {
  Serial.begin(9600);

  servoBase.attach(pinBase);
  servoBrazo.attach(pinBrazo);
  servoCodo.attach(pinCodo);
  servoGarra.attach(pinGarra);

  Serial.println("Robot PUMA 3DOF listo.");
}
void Agarrar(bool agarrar){
  if (agarrar){
    for(int i=0;i<=90;i++){
    servoGarra.write(i);
    delay(20);
  }}else{
    for(int i=0;i<=90;i++){
    servoGarra.write(90-i);
    delay(20);
  }
  }
}

// === Función de cinemática inversa ===
void IK(float px, float py, float pz) {
  float pz_adj = l1 - pz; //ajustamos z para que esté al nivel de la tabla.

  
  //Calculamos q1
  float q1 = atan2(py, px); 
  float q1g = degrees(q1); //pasamos q1 a grados como q1g.
  
  //calculamos la distancia del punto al centro
  float r = sqrt(sq(px) + sq(py));
  
  //si r está muy cerca del centro no hace el movimiento.
  if (r < 3) {
    Serial.println("Error: Punto muy cercano al centro.");
    return;
  }
  
  //calculamos el coseno de q3
  float cosq3 = (sq(r) + sq(pz_adj) - sq(l2) - sq(l3)) / (2 * l2 * l3);
  cosq3 = constrain(cosq3, -1, 1); //limitamos cosq3 a [-1, 1],por cualquier error de redondeo.
  //calculamos el seno de q3
  float senq3 = sqrt(1 - sq(cosq3));

  //calculamos q3
  float q3=atan2(senq3, cosq3);
  float q3g = degrees(q3);

  //y finalmente calculamos q2
  float Beta = atan2(pz_adj, r);
  float Alfa = atan2(l3 * senq3, l2 + l3 * cosq3);
  float q2g = degrees(Beta - Alfa);

  movermotor(q1g, q2g, q3g);
}

// === Movimiento de los servos (30 pasos fijos) ===
void movermotor(float q1_target, float q2_target, float q3_target) {
  // Convertir a ángulos de servo reales,debido 
  float s1_target = q1_target;
  float s2_target = 90 + q2_target;
  float s3_target = 180 - q3_target;

  // Posiciones actuales
  float s1_actual = servoBase.read();
  float s2_actual = servoBrazo.read();
  float s3_actual = servoCodo.read();

  const int pasos = 30;

  for (int i = 0; i <= pasos; i++) {
    float t = (float)i / pasos;

    float s1 = s1_actual + (s1_target - s1_actual) * t;
    float s2 = s2_actual + (s2_target - s2_actual) * t;
    float s3 = s3_actual + (s3_target - s3_actual) * t;

    servoBase.write(s1);
    servoBrazo.write(s2);
    servoCodo.write(s3);

    delay(15); // ajustá la suavidad/velocidad
  }

  q1_actual = q1_target;
  q2_actual = q2_target;
  q3_actual = q3_target;
}

// ----------------------------------------------------------
void loop() {
  IK(25, 0, 15);
  delay(500);
  IK(25, 0, 6);
  Agarrar(true);
  IK(25, 0, 15);
  delay(1000);
  
  IK(-25, 0, 15);
  delay(500);
  IK(-25, 0, 6);
  Agarrar(false);
  IK(-25, 0, 15);
  delay(1000);
  
}
