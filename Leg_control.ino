#define MOT_A1_PIN 23
#define MOT_A2_PIN 22

void spin (int speed)
{

  digitalWrite(MOT_A1_PIN, LOW);
  analogWrite(MOT_A2_PIN, speed);
  
}

void setup(void)
{
  // Initialize the stepper driver control pins to output drive mode.
  pinMode(MOT_A1_PIN, OUTPUT);
  pinMode(MOT_A2_PIN, OUTPUT);

  // Start with drivers off, motors coasting.
  digitalWrite(MOT_A1_PIN, LOW);
  digitalWrite(MOT_A2_PIN, LOW);

  // Initialize the serial UART at 9600 bits per second.
  Serial.begin(9600);
}

void loop(void)
{
  spin(255);
}
