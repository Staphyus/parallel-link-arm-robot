#include <Servo.h> // Library for working with servo motors
#include <avr/wdt.h> // Library for working with watchdog timer


// Defining pins for sensors and servo motors
#define TRIGGER_PIN 6 // Trigger pin for ultrasonic sensor
#define ECHO_PIN 7 // Echo pin of the ultrasonic sensor
#define ROTATIONSERVOPIN 8 // Pin of the rotation servo motor
#define GRIPSERVOPIN 9 // Pin of the grip servo motor
#define BASESERVOPIN 10 // Base servo motor pin
#define ARM1SERVOPIN 11 // Upper arm servo motor pin
#define ARM2SERVOPIN 12 // Lower arm servo motor pin

// Analog inputs for potentiometers and buttons
const int lowerArmPotPin = A0; // Lower arm potentiometer
const int basePotPin = A1; // Base potentiometer
const int upperArmPotPin = A2; // Upper arm potentiometer
const int rotationPotPin = A3; // Rotation potentiometer
const int openButtonPin = A4; // Button to open the gripper
const int closeButtonPin = A5; // Button to close the gripper

// Creating servo motor objects
Servo arm1servo; // Upper arm servo motor
Servo arm2servo; // Lower arm servo motor
Servo baseservo; // Base servo
Servo gripservo; // Gripper servo
Servo rotationservo; // Rotation servo

// Variables for manual control
int lastRotationAngle = 80; // Last rotation angle
int lastUpperArmAngle = 125; // Last angle of the upper arm
int lastLowerArmAngle = 35; // Last angle of the lower arm
int lastBaseAngle = 90; // Last angle of the base
unsigned long previousMillis = 0; // Time of the last angle update

// Variables for coordinate control - current position of the endpoint
float currentX = 0;
float currentY = 160;
float currentZ = 160;

String inputString = “”; // String for storing commands from the serial port
bool stringComplete = false; // Flag for completing string input


/*
 * Function for manual control of the manipulator via potentiometers
 * Reads the states of buttons and potentiometers and applies the corresponding angles to the servo motors
 */
void manual_control() {
  // Reading button states to control the gripper
  int openButtonValue = analogRead(openButtonPin);
  int closeButtonValue = analogRead(closeButtonPin);
  
  // Control the gripper based on button presses
  if (openButtonValue >= 1000) {
      gripservo.write(0); // Open the gripper
  } else if (closeButtonValue >= 1000) {
      gripservo.write(67); // Close the gripper
  }
  
  // Check time to limit the frequency of readings from potentiometers
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= 10) { // Update every 10 ms
      previousMillis = currentMillis;
      
      // Read potentiometer values
      int upperArmValue = analogRead(upperArmPotPin);
      int lowerArmValue = analogRead(lowerArmPotPin);
      int baseValue = analogRead(basePotPin);
      int rotationValue = analogRead(rotationPotPin);
      
      // Update angles based on read potentiometer values
      // For each connection, check the extreme values of the potentiometer 
      // and increment or decrement the corresponding angle
      
      // Rotation
      if (rotationValue == 1023) {
          lastRotationAngle = min(lastRotationAngle + 1, 180); // Increase with a limit of 180
      } else if (rotationValue == 0) {
          lastRotationAngle = max(lastRotationAngle - 1, 0); // Decrease with a limit of 0
      }
      
      // Upper arm
      if (upperArmValue == 1023) {
          lastUpperArmAngle = min(lastUpperArmAngle + 1, 155); // Increase with a limit of 155
      } else if (upperArmValue == 0) {
          lastUpperArmAngle = max(lastUpperArmAngle - 1, 35); // Decrease with a limit of 35
      }
      
      // Lower arm
      if (lowerArmValue == 1023) {
          lastLowerArmAngle = min(lastLowerArmAngle + 1, 125); // Increase with a limit of 125
      } else if (lowerArmValue == 0) {
          lastLowerArmAngle = max(lastLowerArmAngle - 1, 35); // Reduce with a limit of 35
      }
      
      // Base
      if (baseValue == 1023) {
          lastBaseAngle = min(lastBaseAngle + 1, 180); // Increase with a limit of 180
      } else if (baseValue == 0) {
          lastBaseAngle = max(lastBaseAngle - 1, 0); // Decrease with a limit of 0
      }
      
      // Apply updated angles to servo motors
      arm1servo.write(lastUpperArmAngle);
      arm2servo.write(lastLowerArmAngle);
      baseservo.write(lastBaseAngle);
      rotationservo.write(lastRotationAngle);
  }
}


/*
 * Serial port event handling function
 * Called automatically when data arrives via the serial port
 */
void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == ‘\n’) { // If a newline character is received, the command is complete
      stringComplete = true;
    } else {
      inputString += inChar; // Add the character to the command string
    }
  }
}


/*
 * Function for processing commands from the serial port
 * Parses the received command and calls the appropriate actions
 */
void processSerialCommand() {
  // If the string is complete (a newline character has been received)
  if (stringComplete) {
    Serial.println(“Received: ” + inputString);

    // Split the string into parts by spaces
    int values[4];  // array for storing numerical command parameters
    int index = 0;
    char* token = strtok((char*)inputString.c_str(), “ ”);
    while (token != NULL && index < 4) {
      values[index] = atoi(token); // Convert text fragment to number
      token = strtok(NULL, “ ”);
      index++;
    }

    // Processing the command depending on the first number in the string (command code)
    switch (values[0]) {
      case 1: // Manual mode
        Serial.println(“Current mode: Manual”);
        while (true) {
          // Checking the condition for exiting the mode - both buttons are pressed
          if (analogRead(closeButtonPin) == 1023 && analogRead(openButtonPin) == 1023) {
            Serial.println(“Exiting manual mode...”);
            break;
          }
          manual_control(); // Perform manual control
        }
        break;
      case 2: // Scan and grab mode
        Serial.println(“Current mode: Scan and Grab”);
        while (true) {
          // Check the condition for exiting the mode - both buttons are pressed
          if (analogRead(closeButtonPin) == 1023 && analogRead(openButtonPin) == 1023) {
            Serial.println(“Exiting scan and grab mode...”);
            break;
          }
          scanAndGrab(); // Perform scanning and grabbing
        }
        break;
      case 3: // Coordinate control mode
        Serial.println(“Current mode: Coordinate”);
        // Check if the Z coordinate is within the acceptable range
        if (values[3] >= -135 && values[3] <= 160) {
          Serial.println(“Moving to x:” + String(values[1]) + “ y:” + String(values[2]) + “ z:” + String(values[3]));
          // Move to the specified coordinates using polynomial interpolation
          polynomialProfileMove(values[1], values[2], values[3]);
        } else {
          Serial.println(“The ‘z’ value cannot be reached.”);
        }
        break;
      case 4: // Gripper angle control
        gripservo.write(values[1]);
        Serial.println(“Gripper is now set to ” + String(values[1]));
        break;
      default: // Unknown command
        Serial.println(“Unknown command. Try again, please.”);
        break;
    }

    // Clear the buffer for the next command
    inputString = “”;
    stringComplete = false;
  }
}


/*
 * Function for moving to a specified point using polynomial interpolation
 * Ensures smooth movement from the current position to the target
 * xEnd - Target X coordinate
 * yEnd - Target Y coordinate
 * zEnd - Target Z coordinate
 */
void polynomialProfileMove(float xEnd, float yEnd, float zEnd) {
  // Initial coordinates - current position of the manipulator
  float xStart = currentX;
  float yStart = currentY;
  float zStart = currentZ;

  int steps = 100; // Number of steps for interpolation
  
  // Calculation of intermediate points and movement to them
  for (int i = 0; i <= steps; i++) {
    float t = (float)i / steps; // Normalized time (0..1)
    
    // 3rd degree polynomial for smooth acceleration/deceleration
    // s = 3t² - 2t³ (smooth S-shaped profile)
    float s = 3 * t * t - 2 * t * t * t;

    // Linear interpolation using an S-shaped profile
    float x = xStart + s * (xEnd - xStart);
    float y = yStart + s * (yEnd - yStart);
    float z = zStart + s * (zEnd - zStart);
      
    moveToPos(x, y, z); // Move to the calculated intermediate point
    delay(10); // Delay for smooth movement
  
    // Update current position
    currentX = x;
    currentY = y;
    currentZ = z;
  }
}


/*
 * Function for solving the inverse kinematics problem
 * Calculates the required servo motor angles to reach the specified position in Cartesian coordinates
 * x - X coordinate of the target
 * y - Y coordinate of the target
 * z - Z coordinate of the target
 */
void moveToPos(float x, float y, float z) {
  // Base rotation angle (in degrees)
  float b = abs(atan2(y, x) * (180.0 / PI));
  
  // Distance from the center of the base to the projection of the target point on the horizontal plane
  float l = sqrt(x * x + y * y);
  
  // Distance from the center of the base to the target point in 3D space
  float h = sqrt(l * l + z * z);
  
  // Check whether the point is within the working space
  if (h <= 65 || h >= 310 || l <= 92 || l >= 310) {
    return; // Point outside the working space, exit the function
  }

  // Calculate the angles of the links using trigonometry
  float alpha = atan(z / l); // Angle of inclination from the horizontal to the line from the base to the target
  float beta = acos((h / 2.0) / 160.0); // Half the angle of the arc for two identical links
  
  // Angle of the first link from the horizontal (in degrees)
  float a1 = (alpha + beta) * (180.0 / PI);
  // Calculation of the angle of the second link (in degrees)
  float q2 = alpha - beta;
  float xC = l - 200.0 * cos(q2); // X-coordinate of the link connection
  float zC = z - 200.0 * sin(q2); // Z-coordinate of the link connection
  float c = (-24000.0 + xC * xC + zC * zC) / 80.0;
  float a2 = abs(2.0 * atan((-zC - sqrt(xC * xC + zC * zC - c * c)) / (-xC - c))) * (180.0 / PI);

  // Application of calculated angles
  moveToAngle(b, a1, a2);
}


/*
 * Function for setting servo motor angles
 * Checks angle limits and converts abstract angles into real servo motor angles
 * b - Base angle
 * a1 - Angle of the first link
 * a2 - Angle of the second link
 */
void moveToAngle(float b, float a1, float a2) {
  // Checking whether the angles are within the acceptable ranges
  if (a1 >= 0 && a1 <= 120 && a2 >= 70 && a2 <= 180) {
      // Convert abstract angles to real servo motor angles
      int a11 = map(a1, 0, 120, 35, 155); // First link
      int a22 = map(a2, 90, 180, 125, 35); // Second link
      
      // Setting the angles of the servo motors
      arm1servo.write(a11);
      arm2servo.write(a22);
      baseservo.write(b); // The base angle does not need to be converted
  } else {
      // Output an error if the angles are outside the acceptable limits
      Serial.println(“Error: angles out of bounds!”);
      Serial.println(“b ” + String(b) + “ a1 ” + String(a1) + “ a2 ” + String(a2));
  }
}


/*
 * Function for measuring distance using an ultrasonic sensor
 * distance - Distance in millimeters
 */
float measureDistance() {
  // Generate ultrasonic pulse
  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(2); // Short pause for stabilization
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10); // Pulse duration
  digitalWrite(TRIGGER_PIN, LOW);
  
  // Measure the time until the reflected signal is received
  long duration = pulseIn(ECHO_PIN, HIGH);
  
  // Calculate the distance in millimeters
  // Speed of sound 0.034 cm/μs, divide by 2 (there and back)
  // Multiply by 10 to convert to millimeters
  float distance = (duration * 0.0343 / 2) * 10;
  return distance;
}


/*
 * Function for scanning and grabbing objects
 * Performs radial scanning of space using an ultrasonic sensor
 * When an object is detected, it grabs it
 */
void scanAndGrab() {
  static int currentPosition = 0; // Current scan position
  const int num_points = 30; // Number of scan points
  const float l = 100.0; // Scan radius
  const float min_radius = 92.0; // Minimum radius 

  // Move to current position
  moveToPos(currentX, currentY, currentZ);
  delay(400); // Delay to complete movement

  if (currentPosition < (num_points - 3)) {
    // Calculate angle for radial scanning (evenly distributed points)
    float theta = PI - (PI * currentPosition / (num_points - 1));

    // Calculate X and Y coordinates 
    float x = l * cos(theta);
    float y = l * sin(theta);
    float r = sqrt(x * x + y * y);

    // Correcting the position if it exceeds the minimum radius
    if (r < min_radius) {
      float correction = min_radius / r;
      x *= correction;
      y *= correction;
    }
    
    // Updating the current position for scanning
    currentX = x;
    currentY = y;
    currentZ = -80.0; // Fixed scanning height
    
    // Measure distance to object
    float distance = measureDistance();
    
    // Check if object is detected within range
    if (distance > 0 && distance <= 210) {
      // Calculate object coordinates based on measured distance
      float L = sqrt(currentX * currentX + currentY * currentY);
      float objX = currentX / L * (L + distance);
      float objY = currentY / L * (L + distance);
      float objZ = currentZ;

      // Output information about the detected object
      Serial.print(“Object detected! Moving to x: ”);
      Serial.print(objX);
      Serial.print(“ y: ”);
      Serial.println(objY);
      
      // Sequence of actions to capture the object
      polynomialProfileMove(objX, objY, objZ); // Move to the object
      delay(500);

      gripservo.write(85); // Close the gripper
      delay(1000);

      polynomialProfileMove(objX, objY, 160); // Lift the object
      delay(500);

      polynomialProfileMove(200, 0, 160); // Move to the unloading area
      delay(500);

      polynomialProfileMove(200, 0, -80); // Lower the object
      delay(500);

      gripservo.write(0); // Open gripper (release object)
      delay(1000);

      polynomialProfileMove(100, 0, -80); // Move to center
      delay(500);

      polynomialProfileMove(currentX, currentY, currentZ); // Return to scan position
      delay(1000);

      currentPosition = 0; // Reset position to start a new scan cycle
    }

    currentPosition++; // Move to the next scanning position
  } else {
    currentPosition = 0; // Reset position after completing a full cycle
  }
}


/*
 * Function for playing a ready sound signal
 * Uses digital pins to generate a sound signal
 */
void readySFX() {
  pinMode(5, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(3, OUTPUT);
  digitalWrite(5, HIGH);
  digitalWrite(3, LOW);

  // Generate a sequence of short signals
  digitalWrite(4, LOW);
  delay(80);
  digitalWrite(4, HIGH);
  delay(80);
  digitalWrite(4, LOW);
  delay(80);
  digitalWrite(4, HIGH);
  delay(80);
  digitalWrite(4, LOW);
  delay(80);
  digitalWrite(4, HIGH);
  delay(80);
}


/*
 * Function for playing the shutdown sound signal
 * Uses digital pins to generate the sound signal
 */
void offSFX() {
  pinMode(5, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(3, OUTPUT);
  digitalWrite(5, HIGH);
  digitalWrite(3, LOW);

  // Generate a sequence of long signals 
  digitalWrite(4, LOW);
  delay(180);
  digitalWrite(4, HIGH);
  delay(120);
  digitalWrite(4, LOW);
  delay(180);
  digitalWrite(4, HIGH);
}


/*
 * Function for hardware reset of the microcontroller
 * Uses a watchdog timer to perform a reset
 */
void softReset() {
  wdt_enable(WDTO_15MS); // Enable watchdog for 15 ms
  while (1) {}            // Wait for reset (looping will cause watchdog to trigger)
}


/*
 * Initialization function, called once at startup
 * Configures servo motors, sensors, and serial port
 */
void setup() {
  // Initialize servo motors with pulse configuration for precise control
  baseservo.attach(BASESERVOPIN, 700, 2250);
  arm1servo.attach(ARM1SERVOPIN, 700, 2250);
  arm2servo.attach(ARM2SERVOPIN, 700, 2250);
  gripservo.attach(GRIPSERVOPIN, 700, 2250);
  rotationservo.attach(ROTATIONSERVOPIN, 700, 2250);
  
  // Initial positioning
  gripservo.write(0); // Open grip
  rotationservo.write(lastRotationAngle); // Set initial rotation angle
  polynomialProfileMove(currentX, currentY, currentZ); // Move to initial position
  delay(100);

  // Initialize ultrasonic sensor
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Initialize the serial port
  Serial.begin(115200); // High speed for fast data exchange
  inputString.reserve(50);  // Reserve memory for the command line

  // Output information about available commands
  Serial.println(“Robotic Arm Control System”);
  Serial.println(“Commands:”);
  Serial.println(“1 - Manual mode”);
  Serial.println(“2 - Scanning mode”);
  Serial.println(“3 X Y Z - Coordinate mode”);
  Serial.println(“4 degree - Set gripper angle”);

  readySFX(); // Play readiness sound signal
}


/*
 * Main program loop, runs indefinitely
 */
void loop() {
  processSerialCommand(); // Process commands from serial port

  // Checking the button combination for restarting
  if (analogRead(closeButtonPin) == 1023 && analogRead(openButtonPin) == 1023) {
    delay(100);  // Protection against accidental activation
    offSFX(); // Sound signal for shutdown
    Serial.println(“Reset...”);
    softReset(); // Software reset of the microcontroller
  }
}
