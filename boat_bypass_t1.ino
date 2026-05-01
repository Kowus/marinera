#include <Arduino.h>
#include <Servo.h>

Servo leftThruster;
Servo rightThruster;

const int LEFT_PIN = 9;
const int RIGHT_PIN = 10;
const int STOP = 1000;
const int HALF_FORWARD = 1500;
const int FULL_FORWARD = 2000;

// Command buffer
String commandBuffer = "";

void setup()
{
  Serial.begin(115200); // Fast USB baud rate

  leftThruster.attach(LEFT_PIN);
  rightThruster.attach(RIGHT_PIN);

  leftThruster.writeMicroseconds(STOP);
  rightThruster.writeMicroseconds(STOP);
  delay(1000);
  Serial.println("\nBoat controller ready - send commands:");
  Serial.println("M0 + PWM (e.g., M02000=forward)");
  Serial.println("M1 + PWM (e.g., M12000=backward)");
  Serial.println("T0 + PWM (e.g., T01500=turn left)");
  Serial.println("T1 + PWM (e.g., T11500=turn right)");
  Serial.println("S00000 = stop");
  delay(300);
}

void loop()
{
  // Check for incoming serial commands
  if (Serial.available())
  {
    char c = Serial.read();

    if (c == '\n' || c == '\r')
    {
      if (commandBuffer.length() > 0)
      {
        processCommand(commandBuffer);
        commandBuffer = ""; // Clear buffer
      }
    }
    else
    {
      commandBuffer += c;
    }
  }
}

void processCommand(String command)
{
  command.toUpperCase();
  command.trim();

  if (command.length() < 3)
    return;

  char cmd = command[0];
  char dir = command[1];
  String valueStr = command.substring(2);
  int value = valueStr.toInt();

  if (cmd == 'M')
  {
    if (dir == '0')
    {
      setMotorForward(value);
      Serial.print("Move Forward: ");
      Serial.println(value);
    }
    else if (dir == '1')
    {
      setMotorBackward(value);
      Serial.print("Move Backward: ");
      Serial.println(value);
    }
  }
  else if (cmd == 'T')
  {
    if (dir == '0')
    {
      turnLeft(value);
      Serial.print("Turn Left: ");
      Serial.println(value);
    }
    else if (dir == '1')
    {
      turnRight(value);
      Serial.print("Turn Right: ");
      Serial.println(value);
    }
  }
  else if (cmd == 'S')
  {
    stop();
    Serial.println("Stop");
  }
  else if (cmd == 'R')
  {
    Serial.println("Resetting...");
    boardReset();
  }
}

void setMotorForward(int pwm)
{
  pwm = constrain(pwm, STOP, FULL_FORWARD);
  leftThruster.writeMicroseconds(pwm);
  rightThruster.writeMicroseconds(pwm);
}

void setMotorBackward(int pwm)
{
  pwm = constrain(pwm, STOP, FULL_FORWARD);
  leftThruster.writeMicroseconds(STOP + (STOP - pwm)); // Reverse mapping
  rightThruster.writeMicroseconds(STOP + (STOP - pwm));
}

void turnLeft(int pwm)
{
  pwm = constrain(pwm, STOP, FULL_FORWARD);
  leftThruster.writeMicroseconds(STOP);
  rightThruster.writeMicroseconds(pwm);
}

void turnRight(int pwm)
{
  pwm = constrain(pwm, STOP, FULL_FORWARD);
  leftThruster.writeMicroseconds(pwm);
  rightThruster.writeMicroseconds(STOP);
}

void boardReset()
{
  asm volatile("jmp 0");
}
void stop()
{
  leftThruster.writeMicroseconds(STOP);
  rightThruster.writeMicroseconds(STOP);
}
