#include <PPMReader.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// PPM input and output pin definitions
#define PPM_INPUT_PIN 34
#define PPM_OUTPUT_PIN 25

// Timing definitions for PPM signal
#define PPM_FRAME_LENGTH 22500 // Total frame length in microseconds
#define PPM_CHANNELS 8
#define PPM_PULSE_LENGTH 400   // Pulse length in microseconds

// Ultrasonic sensor pin definitions
#define FRONT_TRIGGER_PIN 14
#define FRONT_ECHO_PIN 27
#define BACK_TRIGGER_PIN 12
#define BACK_ECHO_PIN 26

// Define channels for roll and pitch
#define CHANNEL_1 1
#define CHANNEL_2 2

// Obstacle detection distance in cm (1 meter)
#define OBSTACLE_THRESHOLD 100

// Variables to store distances
volatile int frontDistance = OBSTACLE_THRESHOLD;
volatile int backDistance = OBSTACLE_THRESHOLD;

// PPM Reader with 8 channels
PPMReader ppm(PPM_INPUT_PIN, PPM_CHANNELS);

// Function declarations
int getUltrasonicDistance(int triggerPin, int echoPin);
int calculateCorrection(int distance);
void generatePPMOutput(int pin, int channels[]);
void IRAM_ATTR frontSensorISR();
void IRAM_ATTR backSensorISR();

void setup() {
    // Initialize the PPM input and output pins
    pinMode(PPM_OUTPUT_PIN, OUTPUT);

    // Initialize ultrasonic sensor trigger pins
    pinMode(FRONT_TRIGGER_PIN, OUTPUT);
    pinMode(BACK_TRIGGER_PIN, OUTPUT);

    // Initialize ultrasonic sensor echo pins
    pinMode(FRONT_ECHO_PIN, INPUT);
    pinMode(BACK_ECHO_PIN, INPUT);

    // Attach interrupts to ultrasonic sensors
    attachInterrupt(digitalPinToInterrupt(FRONT_ECHO_PIN), frontSensorISR, RISING);
    attachInterrupt(digitalPinToInterrupt(BACK_ECHO_PIN), backSensorISR, RISING);
}

void loop() {
    // Read the PPM inputs for roll (channel 1) and pitch (channel 2)
    int ch1Input = ppm.latestValidChannelValue(CHANNEL_1, 1500); // Default to center (1500us)
    int ch2Input = ppm.latestValidChannelValue(CHANNEL_2, 1500); // Default to center (1500us)

    // Apply corrections only if an obstacle is detected
    if (frontDistance < OBSTACLE_THRESHOLD || backDistance < OBSTACLE_THRESHOLD) {
        if (frontDistance < OBSTACLE_THRESHOLD) {
            ch2Input -= calculateCorrection(frontDistance); // Adjust pitch backward
        }
        if (backDistance < OBSTACLE_THRESHOLD) {
            ch2Input += calculateCorrection(backDistance); // Adjust pitch forward
        }

        // Ensure values are within the valid PPM range (1000 to 2000)
        ch1Input = constrain(ch1Input, 1000, 2000);
        ch2Input = constrain(ch2Input, 1000, 2000);

        // Prepare the channels array for PPM output
        int channels[PPM_CHANNELS] = {ch1Input, ch2Input, 1500, 1500, 1500, 1500, 1500, 1500};

        // Output the PPM signal
        generatePPMOutput(PPM_OUTPUT_PIN, channels);
    }
}

// Function to calculate the correction based on distance
int calculateCorrection(int distance) {
    return map(distance, 0, OBSTACLE_THRESHOLD, 200, 0); // Greater correction closer to obstacle
}

// Function to read distance from SRF-05 ultrasonic sensor
int getUltrasonicDistance(int triggerPin, int echoPin) {
    digitalWrite(triggerPin, LOW);
    delayMicroseconds(2);
    digitalWrite(triggerPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(triggerPin, LOW);
    long duration = pulseIn(echoPin, HIGH, 25000); // Timeout for quicker returns
    return duration == 0 ? OBSTACLE_THRESHOLD : (duration * 0.034 / 2); // Convert to cm
}

// ISR for front sensor
void IRAM_ATTR frontSensorISR() {
    frontDistance = getUltrasonicDistance(FRONT_TRIGGER_PIN, FRONT_ECHO_PIN);
}

// ISR for back sensor
void IRAM_ATTR backSensorISR() {
    backDistance = getUltrasonicDistance(BACK_TRIGGER_PIN, BACK_ECHO_PIN);
}

// Function to generate a PPM signal with corrected values
void generatePPMOutput(int pin, int channels[]) {
    int pulseLength;
    int frameLength = PPM_FRAME_LENGTH;

    // Set the initial pin state
    digitalWrite(pin, LOW);
    delayMicroseconds(PPM_PULSE_LENGTH);

    // Generate PPM signal for each channel
    for (int i = 0; i < PPM_CHANNELS; i++) {
        pulseLength = channels[i];
        digitalWrite(pin, HIGH);
        delayMicroseconds(PPM_PULSE_LENGTH);
        digitalWrite(pin, LOW);
        delayMicroseconds(pulseLength);
        frameLength -= (pulseLength + PPM_PULSE_LENGTH);
    }

    // Sync pulse
    digitalWrite(pin, HIGH);
    delayMicroseconds(frameLength);
    digitalWrite(pin, LOW);
}
