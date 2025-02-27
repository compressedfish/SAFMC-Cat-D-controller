#include <Arduino.h>
#include <Wire.h>
#include <HardwareSerial.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO08x.h>
#include <types.h>
#include <math.h>
#include <sbus.h>

#define I2C_SDA 	4
#define I2C_SCL 	5
#define PPM_TX 		8
#define THROTTLE 	A3
#define OFFSET		1
#define BASE_1_F	2
#define BASE_1_R	3
#define BASE_2_F	43
#define BASE_2_R  44
#define ARMING		7
#define DROPPING	9

#define TRAINER_MODE_PPM
#define DEBUG

//bfs::SbusRx sbus_rx(&Serial0);
/* SBUS object, writing SBUS */
// bfs::SbusTx sbus_tx(&Serial0, 7, 6, true);
// // /* SBUS data */
// bfs::SbusData data;

Adafruit_BNO08x bno08x(-1);
sh2_SensorValue_t sensorValue;
dataOutput_t output;
int counter = 0;
float formattedX = 0;
float formattedY = 0;
float formattedZ = 0;

bool isOffset = false;
float offsetX = 0;
float offsetY = 0;
float offsetZ = 0;

float throttleRaw = 0.0;
int lastT[] = {0,0,0,0};

int three_pos = -500;
int drop = -500;

struct euler_t {
  float yaw;
  float pitch;
  float roll;
} ypr;
euler_t last;

float last_yaw = 0.0;
float last_pitch = 0.0;
float last_roll = 0.0;
float yaw;
float pitch;
float roll;

int angleToRcChannel(float angle)
{
    const float value = fconstrainf(applyDeadband(angle, 5.0f), -45.0f, 45.0f); //5 deg deadband
    return (int)fscalef(value, -45.0f, 45.0f, -500, 500);
  // return map(angle, -90f, 90f, 500, 500);
}
int joystickToRcChannel(float angle)
{
    const float value = fconstrainf(applyDeadband(angle, 0.3f), -1.0f, 1.0f);
    return (int)fscalef(value, -1.0f, 1.0f, 1000, 2000);
}

int getRcChannel_wrapper(uint8_t channel)
{
  if (channel >= 0 && channel < 16)
  {
    return output.channels[channel];
  }
  else
  {
    return 1500;
  }
}

float adjustFloat(float cur, float off)
{
  float adjusted = cur - off;
  if (adjusted < 180.0)
  {
    return adjusted;
  }
  return adjusted - 360.0;
}

hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

#ifdef TRAINER_MODE_PPM

#define PPM_FRAME_LENGTH 22500
#define PPM_PULSE_LENGTH 300
#define PPM_CHANNELS 8

enum ppmState_e {
    PPM_STATE_IDLE,
    PPM_STATE_PULSE,
    PPM_STATE_FILL,
    PPM_STATE_SYNC
};


void IRAM_ATTR onPpmTimer() {

    static uint8_t ppmState = PPM_STATE_IDLE;
    static uint8_t ppmChannel = 0;
    static uint8_t ppmOutput = LOW;
    static int usedFrameLength = 0;
    int currentChannelValue;

    portENTER_CRITICAL(&timerMux);

    if (ppmState == PPM_STATE_IDLE) {
        ppmState = PPM_STATE_PULSE;
        ppmChannel = 0;
        usedFrameLength = 0;
    }

    if (ppmState == PPM_STATE_PULSE) {
        ppmOutput = HIGH;
        usedFrameLength += PPM_PULSE_LENGTH;
        ppmState = PPM_STATE_FILL;

        timerAlarmWrite(timer, PPM_PULSE_LENGTH, true);
    } else if (ppmState == PPM_STATE_FILL) {
        ppmOutput = LOW;
        currentChannelValue = getRcChannel_wrapper(ppmChannel);
        
        ppmChannel++;
        ppmState = PPM_STATE_PULSE;

        if (ppmChannel > PPM_CHANNELS) {
            ppmChannel = 0;
            timerAlarmWrite(timer, PPM_FRAME_LENGTH - usedFrameLength, true);
            usedFrameLength = 0;
        } else {
            usedFrameLength += currentChannelValue - PPM_PULSE_LENGTH;
            timerAlarmWrite(timer, currentChannelValue - PPM_PULSE_LENGTH, true);
        }
    }
    portEXIT_CRITICAL(&timerMux);
    digitalWrite(PPM_TX, ppmOutput);
}
#endif

#ifdef FAST_MODE
  // Top frequency is reported to be 1000Hz (but freq is somewhat variable)
  sh2_SensorId_t reportType = SH2_GYRO_INTEGRATED_RV;
  long reportIntervalUs = 2000;
#else
  // Top frequency is about 250Hz but this report is more accurate
  sh2_SensorId_t reportType = SH2_ARVR_STABILIZED_RV;
  long reportIntervalUs = 5000;
#endif

void setReports(sh2_SensorId_t reportType, long report_interval) {
  //Serial.println("Setting desired reports");
  if (!bno08x.enableReport(reportType, report_interval)) {
    //Serial.println("Could not enable stabilized remote vector");
		while (1);
  }
}

void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees = false) {

    float sqr = sq(qr);
    float sqi = sq(qi);
    float sqj = sq(qj);
    float sqk = sq(qk);

    ypr->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
    ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
    ypr->roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

    if (degrees) {
      ypr->yaw *= RAD_TO_DEG;
      ypr->pitch *= RAD_TO_DEG;
      ypr->roll *= RAD_TO_DEG;
    }
}

void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, euler_t* ypr, bool degrees = false) {
    quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

void quaternionToEulerGI(sh2_GyroIntegratedRV_t* rotational_vector, euler_t* ypr, bool degrees = false) {
    quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

// void onSbusTimer() {
//   sbus_tx.Write();
// }

void setup() {
  // put your setup code here, to run once:
  #ifdef DEBUG 
    // Serial.begin(115200);
  #endif
	pinMode(PPM_TX, OUTPUT);
	pinMode(THROTTLE, INPUT);
	// pinMode(OFFSET, INPUT_PULLUP);
	// pinMode(BASE_F, INPUT_PULLUP);
	// pinMode(BASE_R, INPUT_PULLUP);
	// pinMode(ARMING, INPUT_PULLUP);
	// pinMode(DROPPING, INPUT_PULLUP);
  // pinMode(BASE_1_F, INPUT_PULLUP);
  // pinMode(BASE_1_R, INPUT_PULLUP);
  // pinMode(BASE_2_F, INPUT_PULLUP);
  // pinMode(BASE_2_R, INPUT_PULLUP);
  // pinMode(DROPPING, INPUT_PULLUP);
	timer = timerBegin(0, 80, true);
	timerAttachInterrupt(timer, &onPpmTimer, true);
	timerAlarmWrite(timer, 12000, true);
	timerAlarmEnable(timer);
  //sbus_tx.Begin();
	//Serial.println("Initialising");
	if (!bno08x.begin_I2C()) {
		//Serial.println("Could not find BNO085");
		while (1);
	}
  //Serial.println("BNO085 found");
	setReports(reportType, reportIntervalUs);
	bno08x.hardwareReset();
}

void loop() {
  if (bno08x.wasReset()) {
    //Serial.print("sensor was reset ");
    setReports(reportType, reportIntervalUs);
  }

	if (bno08x.getSensorEvent(&sensorValue)) {
    // in this demo only one report type will be received depending on FAST_MODE define (above)
    switch (sensorValue.sensorId) {
      case SH2_ARVR_STABILIZED_RV:
        quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr, true);
      case SH2_GYRO_INTEGRATED_RV:
        // faster (more noise?)
        quaternionToEulerGI(&sensorValue.un.gyroIntegratedRV, &ypr, true);
        break;
    }
    //static long last = 0;
    // long now = micros();
		if (touchRead(OFFSET) > 19000) {
			last = ypr;
		}
		yaw = ypr.yaw - last.yaw;
		pitch = ypr.pitch - last.pitch;
		roll = ypr.roll - last.roll;
    // Serial.print(now - last);             Serial.print("\t");
    // last = now;
    // Serial.print(sensorValue.status);     Serial.print("\t");  // This is accuracy in the range of 0 to 3
    // Serial.print(yaw);                    Serial.print("\t");
    // Serial.print(pitch);                  Serial.print("\t");
    // Serial.print(roll);							      Serial.print("\t");
		// Serial.print(touchRead(OFFSET));      Serial.print("\t");
		// Serial.print(analogRead(THROTTLE));   Serial.print("\t");
    // Serial.print(digitalRead(BASE_1_F));
    // Serial.print(digitalRead(BASE_1_R));
    // Serial.print(digitalRead(BASE_1_F));
    // Serial.print(digitalRead(BASE_1_R));
    // Serial.println(digitalRead(DROPPING));
		output.channels[ROLL] = DEFAULT_CHANNEL_VALUE + angleToRcChannel(roll);
		output.channels[PITCH] = DEFAULT_CHANNEL_VALUE + angleToRcChannel(pitch);
    output.channels[YAW] = DEFAULT_CHANNEL_VALUE - angleToRcChannel(yaw); // why is it minus?
    output.channels[THROTTLE_T] = joystickToRcChannel(fscalef(analogRead(THROTTLE), 130.0f, 2000.0f, -1.0, 1.0));
    // data.ch[ROLL] = DEFAULT_CHANNEL_VALUE - angleToRcChannel(pitch);
    // data.ch[PITCH] = DEFAULT_CHANNEL_VALUE - angleToRcChannel(roll);
    // data.ch[YAW] = DEFAULT_CHANNEL_VALUE - angleToRcChannel(yaw);
    // data.ch[ROLL] = joystickToRcChannel(fscalef(analogRead(THROTTLE), 130.0f, 2000.0f, -1.0, 1.0));

    // output.channels[4] = 2000;
    // if (!digitalRead(BASE_1_F)) {output.channels[5] = 2000;}
    // else if (!digitalRead(BASE_1_R)) {output.channels[5] = 1000;}
    // else {output.channels[5] = 1500;}
    // if (!digitalRead(BASE_2_F)) {output.channels[6] = 2000;}
    // else if (!digitalRead(BASE_2_R)) {output.channels[6] = 1000;}
    // else {output.channels[6] = 1500;}
    // if (!digitalRead(DROPPING)) {output.channels[7] = 1750;}
    // else {output.channels[7] = 1500;}

    // Serial.print(output.channels[ROLL]);  Serial.print("\t");
    // Serial.print(output.channels[PITCH]);  Serial.print("\t");
    // Serial.print(output.channels[YAW]);  Serial.print("\t");
    // Serial.println(output.channels[THROTTLE_T]); 
  }
}