#include <Servo.h>
#include <SoftwareSerial.h>

// Definir pines del módulo Bluetooth (RX, TX)
SoftwareSerial bluetooth(10, 11); // RX, TX

// Pines del puente H L298 para los motores traseros
int motorIzqPinA = 2; //pin a IN2
int motorIzqPinB = 4; // pin a IN1
int motorDerPinA = 7; //pin A IN4
int motorDerPinB = 5; //pin a IN3
int velocidadIzqPin =6 ;  // Pin PWM para velocidad del motor izquierdo
int velocidadDerPin = 3;  // Pin PWM para velocidad del motor derecho

int potenciaizq; //variable controladora de la potencia del motor izquierdo
int potenciader; // varable controladora de la potencia del motor derecho
int PotenciaAvance = 150; // potencia de avanze sin giro
int DiferencialGiro = 100; //potencia de avanze en caso de que haya giro hacia algun lado
// Configuración del servo
Servo direccionServo;
int servoPin = 9;
int direccionCentro = 90;  // Posición del servo centrado (90 grados)
int anguloGiroDer = 135; //Posicion del servo para girar a la derecha
int anguloGiroIzq = 45; // Posicion del servo para girar izquierda
// Variables para controlar el estado del motor y la dirección
int direccionActual = direccionCentro;
int movimientoActual = 0;  // 0 = detenido, 1 = adelante, -1 = atrás


void setup() {
  // Configuración del Bluetooth y del puerto serie
  Serial.begin(9600);
  bluetooth.begin(9600);
  Serial.println("Esperando señal Bluetooth...");

  // Configuración de los pines del puente H L298
  pinMode(motorIzqPinA, OUTPUT);
  pinMode(motorIzqPinB, OUTPUT);
  pinMode(motorDerPinA, OUTPUT);
  pinMode(motorDerPinB, OUTPUT);
  pinMode(velocidadIzqPin, OUTPUT);
  pinMode(velocidadDerPin, OUTPUT);
  
  // Configuración del servo
  direccionServo.attach(servoPin);
  direccionServo.write(direccionCentro);  // Inicializar con el servo centrado
}

void loop() {
  // Leer la señal Bluetooth
  if (bluetooth.available()) {
    char comando = bluetooth.read(); // Variable que almacena el dato que llega desde la aplicacion
    Serial.print("Comando recibido: ");
    Serial.println(comando);
    
    // Procesar el comando para controlar la dirección, el movimiento y la potencia de los motores
    procesarComando(comando);
  }
  Potencia();// configurar la potencia suministrada a cada motor trasero

  // Aplicar el movimiento actual del motor
  actualizarMovimiento();
   
    Potencia();// configurar la potencia suministrada a cada motor trasero

  // Aplicar el movimiento actual del motor

 

  // Aplicar la dirección actual
  direccionServo.write(direccionActual);
  
  delay(50); // Pequeño retardo para evitar saturación de la comunicación
}

// Función para procesar el comando recibido
void procesarComando(char comando) {
  switch (comando) {
    // Movimiento hacia adelante o detener
    case 'F':  // Adelante (botón presionado)
      movimientoActual = 1;
      potenciaizq=PotenciaAvance;
      potenciader=PotenciaAvance;
        // Movimiento adelante
      break;
    case 'f':  // Detener Adelante (botón soltado)
      movimientoActual = 0; 
      potenciaizq=PotenciaAvance;
      potenciader=PotenciaAvance;
       // Detener
      break;

    // Movimiento hacia atrás o detener
    case 'B':  // Atrás (botón presionado)
      movimientoActual = -1;
      potenciaizq=PotenciaAvance;
      potenciader=PotenciaAvance;
        // Movimiento atrás
      break;
    case 'b':  // Detener Atrás (botón soltado)
      movimientoActual = 0;
      potenciaizq=PotenciaAvance;
      potenciader=PotenciaAvance;
        // Detener
      break;

    // Control de dirección (giro a la izquierda, derecha o centrar)
    case 'L':  // Girar a la izquierda (botón presionado)
      direccionActual = anguloGiroIzq;
      potenciaizq=DiferencialGiro;
      potenciader=PotenciaAvance;  // Ajusta este valor según el ángulo deseado
       // Reducir velocidad del motor izquierdo para girar mejor
      break;
    case 'l':  // Centrar dirección (botón soltado)
      direccionActual = direccionCentro;
      potenciaizq=PotenciaAvance;
      potenciader=PotenciaAvance;
        // Igualar velocidades al centrar
      break;
    case 'R':  // Girar a la derecha (botón presionado)
      direccionActual = anguloGiroDer;
      potenciaizq=PotenciaAvance;
      potenciader=DiferencialGiro;  // Ajusta este valor según el ángulo deseado
        // Reducir velocidad del motor derecho para girar mejor
      break;
    case 'r':  // Centrar dirección (botón soltado)
      direccionActual = direccionCentro;
      potenciaizq=PotenciaAvance;
      potenciader=PotenciaAvance;
        // Igualar velocidades al centrar
      break;
  }
}

// Función para ajustar las velocidades de los motores al girar
void Potencia(){

  if(movimientoActual==1){  // en caso de que el motor avance las potencias de giro son acordes al giro
  analogWrite(velocidadIzqPin,potenciaizq); 
  analogWrite(velocidadDerPin,potenciader);
  }
  else if(movimientoActual==-1){ // cuando va hacia atras la variacion de potencias debe ser invertida
  analogWrite(velocidadIzqPin,potenciader);
  analogWrite(velocidadDerPin,potenciaizq);
  }
  else{   // si no estoy avanzando o retrocediendo mantengo la alimentacion de ambos motores
  analogWrite(velocidadIzqPin,potenciaizq);
  analogWrite(velocidadDerPin,potenciader);
  }
}

// Función para actualizar el movimiento de los motores traseros según el estado
void actualizarMovimiento() {
  switch (movimientoActual) {
    case 1:  // Adelante
      digitalWrite(motorIzqPinA, HIGH);
      digitalWrite(motorIzqPinB, LOW);
      digitalWrite(motorDerPinA, HIGH);
      digitalWrite(motorDerPinB, LOW);
      break;
    case -1:  // Atrás
      digitalWrite(motorIzqPinA, LOW);
      digitalWrite(motorIzqPinB, HIGH);
      digitalWrite(motorDerPinA, LOW);
      digitalWrite(motorDerPinB, HIGH);
      break;
    default:  // Detenido
      digitalWrite(motorIzqPinA, LOW);
      digitalWrite(motorIzqPinB, LOW);
      digitalWrite(motorDerPinA, LOW);
      digitalWrite(motorDerPinB, LOW);
      break;
  }
}
