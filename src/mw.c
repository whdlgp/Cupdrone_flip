#include "board.h"
#include "mw.h"

// June 2013     V2.2-dev

flags_t f;
int16_t debug[4];
uint8_t toggleBeep = 0;
uint32_t currentTime = 0;
uint32_t previousTime = 0;
uint16_t cycleTime = 0;         // this is the number in micro second to achieve a full loop, it can differ a little and is taken into account in the PID loop
int16_t headFreeModeHold;

uint8_t vbat;                   // battery voltage in 0.1V steps
int16_t telemTemperature1;      // gyro sensor temperature

int16_t failsafeCnt = 0;
int16_t failsafeEvents = 0;
int16_t rcData[RC_CHANS];       // interval [1000;2000]
int16_t rcCommand[4];           // interval [1000;2000] for THROTTLE and [-500;+500] for ROLL/PITCH/YAW
int16_t lookupPitchRollRC[PITCH_LOOKUP_LENGTH];     // lookup table for expo & RC rate PITCH+ROLL
int16_t lookupThrottleRC[THROTTLE_LOOKUP_LENGTH];   // lookup table for expo & mid THROTTLE
uint16_t rssi;                  // range: [0;1023]
rcReadRawDataPtr rcReadRawFunc = NULL;  // receive data from default (pwm/ppm) or additional (spek/sbus/?? receiver drivers)

static void pidMultiWii(void);
static void pidRewrite(void);

static void pidGyrorate(void);

pidControllerFuncPtr pid_controller = pidMultiWii; // which pid controller are we using, defaultMultiWii

uint8_t dynP8[3], dynI8[3], dynD8[3];
uint8_t rcOptions[CHECKBOXITEMS];

int16_t axisPID[3];

// **********************
// GPS
// **********************
int32_t GPS_coord[2];
int32_t GPS_home[2];
int32_t GPS_hold[2];
uint8_t GPS_numSat;
uint16_t GPS_distanceToHome;        // distance to home point in meters
int16_t GPS_directionToHome;        // direction to home or hol point in degrees
uint16_t GPS_altitude, GPS_speed;   // altitude in 0.1m and speed in 0.1m/s
uint8_t GPS_update = 0;             // it's a binary toogle to distinct a GPS position update
int16_t GPS_angle[2] = { 0, 0 };    // it's the angles that must be applied for GPS correction
uint16_t GPS_ground_course = 0;     // degrees * 10
int16_t nav[2];
int16_t nav_rated[2];               // Adding a rate controller to the navigation to make it smoother
int8_t nav_mode = NAV_MODE_NONE;    // Navigation mode
uint8_t  GPS_numCh;                 // Number of channels
uint8_t  GPS_svinfo_chn[16];        // Channel number
uint8_t  GPS_svinfo_svid[16];       // Satellite ID
uint8_t  GPS_svinfo_quality[16];    // Bitfield Qualtity
uint8_t  GPS_svinfo_cno[16];        // Carrier to Noise Ratio (Signal Strength)

// Automatic ACC Offset Calibration
uint16_t InflightcalibratingA = 0;
int16_t AccInflightCalibrationArmed;
uint16_t AccInflightCalibrationMeasurementDone = 0;
uint16_t AccInflightCalibrationSavetoEEProm = 0;
uint16_t AccInflightCalibrationActive = 0;

// Battery monitoring stuff
uint8_t batteryCellCount = 3;       // cell count
uint16_t batteryWarningVoltage;     // annoying buzzer after this one, battery ready to be dead

void blinkLED(uint8_t num, uint8_t wait, uint8_t repeat)
{
    uint8_t i, r;

    for (r = 0; r < repeat; r++) {
        for (i = 0; i < num; i++) {
            LED0_TOGGLE;            // switch LEDPIN state
            BEEP_ON;
            delay(wait);
            BEEP_OFF;
        }
        delay(60);
    }
}

/********************Debug code********************/
#define debugCommand	0
#define debugAngle		1
#define debugMotor		2
#define debugRcdata		3
#define debugAxisPID	4
#define debugAngleRate	5

static uint8_t debugCounter = 0;
void debugPrint(uint8_t debugMode, uint8_t counterMax)
{
	switch(debugMode)
	{
	case debugCommand:
		debugCounter++;
		if(debugCounter > counterMax)
		{
			Hw_VCom_Printf("roll: %d, pitch: %d, yaw: %d, throttle: %d", rcCommand[ROLL], rcCommand[PITCH], rcCommand[YAW], rcCommand[THROTTLE]);
			debugCounter = 0;
		}
		break;

	case debugAngle:
		debugCounter++;
		if(debugCounter > counterMax)
		{
			Hw_VCom_Printf("roll: %d, pitch: %d", angle[ROLL], angle[PITCH]);
			debugCounter = 0;
		}
		break;

	case debugMotor:
		debugCounter++;
		if(debugCounter > counterMax)
		{
			Hw_VCom_Printf("motor0: %d, motor1: %d, motor2: %d, motor3: %d", motor[0], motor[1], motor[2], motor[3]);
			debugCounter = 0;
		}
		break;

	case debugRcdata:
		debugCounter++;
		if(debugCounter > counterMax)
		{
			Hw_VCom_Printf("roll: %d, pitch: %d, yaw: %d, throttle: %d", rcData[ROLL], rcData[PITCH], rcData[YAW], rcData[THROTTLE]);
			debugCounter = 0;
		}
		break;

	case debugAxisPID:
		debugCounter++;
		if(debugCounter > counterMax)
		{
			Hw_VCom_Printf("roll: %d, pitch: %d, yaw: %d", axisPID[ROLL], axisPID[PITCH], axisPID[YAW]);
			debugCounter = 0;
		}
		break;

	case debugAngleRate:
		debugCounter++;
		if(debugCounter > counterMax)
		{
			Hw_VCom_Printf("roll: %d, pitch: %d, yaw: %d", gyroData[ROLL], gyroData[PITCH], gyroData[YAW]);
			debugCounter = 0;
		}
		break;
	}
}
/**************************************************/

#define BREAKPOINT 1500

void annexCode(void)
{
    static uint32_t calibratedAccTime;
    int32_t tmp, tmp2;
    int32_t axis, prop1, prop2;
    static uint8_t buzzerFreq;  // delay between buzzer ring

    // vbat shit
    static uint8_t vbatTimer = 0;
    static uint8_t ind = 0;
    uint16_t vbatRaw = 0;
    static uint16_t vbatRawArray[8];

    int i;

    // PITCH & ROLL only dynamic PID adjustemnt,  depending on throttle value
    if (rcData[THROTTLE] < BREAKPOINT) {
        prop2 = 100;
    } else {
        if (rcData[THROTTLE] < 2000) {
            prop2 = 100 - (uint16_t) cfg.dynThrPID * (rcData[THROTTLE] - BREAKPOINT) / (2000 - BREAKPOINT);
        } else {
            prop2 = 100 - cfg.dynThrPID;
        }
    }

    for (axis = 0; axis < 3; axis++) {
        tmp = min(abs(rcData[axis] - mcfg.midrc), 500);
        if (axis != 2) {        // ROLL & PITCH
            if (cfg.deadband) {
                if (tmp > cfg.deadband) {
                    tmp -= cfg.deadband;
                } else {
                    tmp = 0;
                }
            }

            tmp2 = tmp / 100;
            rcCommand[axis] = lookupPitchRollRC[tmp2] + (tmp - tmp2 * 100) * (lookupPitchRollRC[tmp2 + 1] - lookupPitchRollRC[tmp2]) / 100;
            prop1 = 100 - (uint16_t) cfg.rollPitchRate * tmp / 500;
            prop1 = (uint16_t) prop1 *prop2 / 100;
        } else {                // YAW
            if (cfg.yawdeadband) {
                if (tmp > cfg.yawdeadband) {
                    tmp -= cfg.yawdeadband;
                } else {
                    tmp = 0;
                }
            }
            rcCommand[axis] = tmp * -mcfg.yaw_control_direction;
            prop1 = 100 - (uint16_t)cfg.yawRate * abs(tmp) / 500;
        }
        dynP8[axis] = (uint16_t)cfg.P8[axis] * prop1 / 100;
        dynI8[axis] = (uint16_t)cfg.I8[axis] * prop1 / 100;
        dynD8[axis] = (uint16_t)cfg.D8[axis] * prop1 / 100;
        if (rcData[axis] < mcfg.midrc)
            rcCommand[axis] = -rcCommand[axis];
    }

    tmp = constrain(rcData[THROTTLE], mcfg.mincheck, 2000);
    tmp = (uint32_t) (tmp - mcfg.mincheck) * 1000 / (2000 - mcfg.mincheck);       // [MINCHECK;2000] -> [0;1000]
    tmp2 = tmp / 100;
    rcCommand[THROTTLE] = lookupThrottleRC[tmp2] + (tmp - tmp2 * 100) * (lookupThrottleRC[tmp2 + 1] - lookupThrottleRC[tmp2]) / 100;    // [0;1000] -> expo -> [MINTHROTTLE;MAXTHROTTLE]

    if (f.HEADFREE_MODE) {
        float radDiff = (heading - headFreeModeHold) * M_PI / 180.0f;
        float cosDiff = cosf(radDiff);
        float sinDiff = sinf(radDiff);
        int16_t rcCommand_PITCH = rcCommand[PITCH] * cosDiff + rcCommand[ROLL] * sinDiff;
        rcCommand[ROLL] = rcCommand[ROLL] * cosDiff - rcCommand[PITCH] * sinDiff;
        rcCommand[PITCH] = rcCommand_PITCH;
    }

    if (feature(FEATURE_VBAT)) {
        if (!(++vbatTimer % VBATFREQ)) {
            vbatRawArray[(ind++) % 8] = adcGetChannel(ADC_BATTERY);
            for (i = 0; i < 8; i++)
                vbatRaw += vbatRawArray[i];
            vbat = batteryAdcToVoltage(vbatRaw / 8);
        }
        if ((vbat > batteryWarningVoltage) || (vbat < mcfg.vbatmincellvoltage)) { // VBAT ok, buzzer off
            buzzerFreq = 0;
        } else
            buzzerFreq = 4;     // low battery
    }

    buzzer(buzzerFreq);         // external buzzer routine that handles buzzer events globally now

    if ((calibratingA > 0 && sensors(SENSOR_ACC)) || (calibratingG > 0)) {      // Calibration phasis
        LED0_TOGGLE;
    } else {
        if (f.ACC_CALIBRATED)
            LED0_OFF;
        if (f.ARMED)
            LED0_ON;
        // This will switch to/from 9600 or 115200 baud depending on state. Of course, it should only do it on changes. With telemetry_softserial>0 telemetry is always enabled, also see updateTelemetryState()
        if (feature(FEATURE_TELEMETRY))
            updateTelemetryState();
    }

#ifdef LEDRING
    if (feature(FEATURE_LED_RING)) {
        static uint32_t LEDTime;
        if ((int32_t)(currentTime - LEDTime) >= 0) {
            LEDTime = currentTime + 50000;
            ledringState();
        }
    }
#endif

    if ((int32_t)(currentTime - calibratedAccTime) >= 0) {
        if (!f.SMALL_ANGLES_25) {
            f.ACC_CALIBRATED = 0; // the multi uses ACC and is not calibrated or is too much inclinated
            LED0_TOGGLE;
            //DEBUG_PRINT("LED\r\n");
            calibratedAccTime = currentTime + 500000;
        } else {
            f.ACC_CALIBRATED = 1;
        }
    }

    serialCom();

    if (sensors(SENSOR_GPS)) {
        static uint32_t GPSLEDTime;
        if ((int32_t)(currentTime - GPSLEDTime) >= 0 && (GPS_numSat >= 5)) {
            GPSLEDTime = currentTime + 150000;
            LED1_TOGGLE;
        }
    }

    // Read out gyro temperature. can use it for something somewhere. maybe get MCU temperature instead? lots of fun possibilities.
    if (gyro.temperature)
        gyro.temperature(&telemTemperature1);
    else {
        // TODO MCU temp
    }
}

uint16_t pwmReadRawRC(uint8_t chan)
{
    uint16_t data;

    data = pwmRead(mcfg.rcmap[chan]);
    if (data < 750 || data > 2250)
        data = mcfg.midrc;

    return data;
}

void computeRC(void)
{
    uint8_t chan;

    if (feature(FEATURE_SERIALRX)) {
        for (chan = 0; chan < 8; chan++)
            rcData[chan] = rcReadRawFunc(chan);
    } else {
        static int16_t rcData4Values[8][4], rcDataMean[8];
        static uint8_t rc4ValuesIndex = 0;
        uint8_t a;

        rc4ValuesIndex++;
        for (chan = 0; chan < 8; chan++) {
            rcData4Values[chan][rc4ValuesIndex % 4] = rcReadRawFunc(chan);
            rcDataMean[chan] = 0;
            for (a = 0; a < 4; a++)
                rcDataMean[chan] += rcData4Values[chan][a];

            rcDataMean[chan] = (rcDataMean[chan] + 2) / 4;
            if (rcDataMean[chan] < rcData[chan] - 3)
                rcData[chan] = rcDataMean[chan] + 2;
            if (rcDataMean[chan] > rcData[chan] + 3)
                rcData[chan] = rcDataMean[chan] - 2;
        }
    }
}

//static void mwArm(void)
void mwArm(void)
{
    if (calibratingG == 0 && f.ACC_CALIBRATED) {
        // TODO: feature(FEATURE_FAILSAFE) && failsafeCnt < 2
        // TODO: && ( !feature || ( feature && ( failsafecnt > 2) )
        if (!f.ARMED) {         // arm now!
            f.ARMED = 1;
            headFreeModeHold = heading;
        }
    } else if (!f.ARMED) {
        blinkLED(2, 255, 1);
    }
}

//static void mwDisarm(void)
void mwDisarm(void)
{
    if (f.ARMED)
        f.ARMED = 0;
}

static void mwVario(void)
{

}

static int32_t errorGyroI[3] = { 0, 0, 0 };
static int32_t errorAngleI[2] = { 0, 0 };

static void pidMultiWii(void)
{
    int axis, prop;
    int32_t error, errorAngle;
    int32_t PTerm, ITerm, PTermACC = 0, ITermACC = 0, PTermGYRO = 0, ITermGYRO = 0, DTerm;
    static int16_t lastGyro[3] = { 0, 0, 0 };
    static int32_t delta1[3], delta2[3];
    int32_t deltaSum;
    int32_t delta;

    // **** PITCH & ROLL & YAW PID ****
    prop = max(abs(rcCommand[PITCH]), abs(rcCommand[ROLL])); // range [0;500]
    for (axis = 0; axis < 3; axis++) {
        if ((f.ANGLE_MODE || f.HORIZON_MODE) && axis < 2) { // MODE relying on ACC
            // 50 degrees max inclination
            errorAngle = constrain(2 * rcCommand[axis] + GPS_angle[axis], -((int)mcfg.max_angle_inclination), +mcfg.max_angle_inclination) - angle[axis] + cfg.angleTrim[axis];
            PTermACC = errorAngle * cfg.P8[PIDLEVEL] / 100; // 32 bits is needed for calculation: errorAngle*P8[PIDLEVEL] could exceed 32768   16 bits is ok for result
            PTermACC = constrain(PTermACC, -cfg.D8[PIDLEVEL] * 5, +cfg.D8[PIDLEVEL] * 5);

            errorAngleI[axis] = constrain(errorAngleI[axis] + errorAngle, -10000, +10000); // WindUp
            ITermACC = (errorAngleI[axis] * cfg.I8[PIDLEVEL]) >> 12;
        }
        if (!f.ANGLE_MODE || f.HORIZON_MODE || axis == 2) { // MODE relying on GYRO or YAW axis
            error = (int32_t)rcCommand[axis] * 10 * 8 / cfg.P8[axis];
            error -= gyroData[axis];

            PTermGYRO = rcCommand[axis];

            errorGyroI[axis] = constrain(errorGyroI[axis] + error, -16000, +16000); // WindUp
            if (abs(gyroData[axis]) > 640)
                errorGyroI[axis] = 0;
            ITermGYRO = (errorGyroI[axis] / 125 * cfg.I8[axis]) >> 6;
        }
        if (f.HORIZON_MODE && axis < 2) {
            PTerm = (PTermACC * (500 - prop) + PTermGYRO * prop) / 500;
            ITerm = (ITermACC * (500 - prop) + ITermGYRO * prop) / 500;
        } else {
            if (f.ANGLE_MODE && axis < 2) {
                PTerm = PTermACC;
                ITerm = ITermACC;
            } else {
                PTerm = PTermGYRO;
                ITerm = ITermGYRO;
            }
        }

        PTerm -= (int32_t)gyroData[axis] * dynP8[axis] / 10 / 8; // 32 bits is needed for calculation
        delta = gyroData[axis] - lastGyro[axis];
        lastGyro[axis] = gyroData[axis];
        deltaSum = delta1[axis] + delta2[axis] + delta;
        delta2[axis] = delta1[axis];
        delta1[axis] = delta;
        DTerm = (deltaSum * dynD8[axis]) / 32;
        axisPID[axis] = PTerm + ITerm - DTerm;
    }
}

#define GYRO_I_MAX 256

static void pidRewrite(void)
{
    int32_t errorAngle = 0;
    int axis;
    int32_t delta, deltaSum;
    static int32_t delta1[3], delta2[3];
    int32_t PTerm, ITerm, DTerm;
    static int32_t lastError[3] = { 0, 0, 0 };
    int32_t AngleRateTmp, RateError;

    // ----------PID controller----------
    for (axis = 0; axis < 3; axis++) {
        // -----Get the desired angle rate depending on flight mode
        if ((f.ANGLE_MODE || f.HORIZON_MODE) && axis < 2 ) { // MODE relying on ACC
            // calculate error and limit the angle to max configured inclination
            errorAngle = constrain((rcCommand[axis] << 1) + GPS_angle[axis], -((int)mcfg.max_angle_inclination), +mcfg.max_angle_inclination) - angle[axis] + cfg.angleTrim[axis]; // 16 bits is ok here
        }
        if (axis == 2) { // YAW is always gyro-controlled (MAG correction is applied to rcCommand)
            AngleRateTmp = (((int32_t)(cfg.yawRate + 27) * rcCommand[2]) >> 5);
         } else {
            if (!f.ANGLE_MODE) { //control is GYRO based (ACRO and HORIZON - direct sticks control is applied to rate PID
                AngleRateTmp = ((int32_t) (cfg.rollPitchRate + 27) * rcCommand[axis]) >> 4;
                if (f.HORIZON_MODE) {
                    // mix up angle error to desired AngleRateTmp to add a little auto-level feel
                    AngleRateTmp += (errorAngle * cfg.I8[PIDLEVEL]) >> 8;
                }
            } else { // it's the ANGLE mode - control is angle based, so control loop is needed
                AngleRateTmp = (errorAngle * cfg.P8[PIDLEVEL]) >> 4;
            }
        }

        // --------low-level gyro-based PID. ----------
        // Used in stand-alone mode for ACRO, controlled by higher level regulators in other modes
        // -----calculate scaled error.AngleRates
        // multiplication of rcCommand corresponds to changing the sticks scaling here
        RateError = AngleRateTmp - gyroData[axis];

        // -----calculate P component
        PTerm = (RateError * cfg.P8[axis]) >> 7;
        // -----calculate I component
        // there should be no division before accumulating the error to integrator, because the precision would be reduced.
        // Precision is critical, as I prevents from long-time drift. Thus, 32 bits integrator is used.
        // Time correction (to avoid different I scaling for different builds based on average cycle time)
        // is normalized to cycle time = 2048.
        errorGyroI[axis] = errorGyroI[axis] + ((RateError * cycleTime) >> 11) * cfg.I8[axis];

        // limit maximum integrator value to prevent WindUp - accumulating extreme values when system is saturated.
        // I coefficient (I8) moved before integration to make limiting independent from PID settings
        errorGyroI[axis] = constrain(errorGyroI[axis], (int32_t)-GYRO_I_MAX << 13, (int32_t)+GYRO_I_MAX << 13);
        ITerm = errorGyroI[axis] >> 13;

        //-----calculate D-term
        delta = RateError - lastError[axis];  // 16 bits is ok here, the dif between 2 consecutive gyro reads is limited to 800
        lastError[axis] = RateError;

        // Correct difference by cycle time. Cycle time is jittery (can be different 2 times), so calculated difference
        // would be scaled by different dt each time. Division by dT fixes that.
        delta = (delta * ((uint16_t)0xFFFF / (cycleTime >> 4))) >> 6;
        // add moving average here to reduce noise
        deltaSum = delta1[axis] + delta2[axis] + delta;
        delta2[axis] = delta1[axis];
        delta1[axis] = delta;
        DTerm = (deltaSum * cfg.D8[axis]) >> 8;

        // -----calculate total PID output
        axisPID[axis] = PTerm + ITerm + DTerm;
    }
}

// **********************
// FlipMode Area Start
// **********************

// **********************
// Steps of flip maneuver
// 1. pidControl	-	Stabilize with PID controller
// 2. acceleration	-	Accelerate up at near-maximum collective acceleration(throttle)
// 3. startRotate	-	start rotating
// 4. coast			-	Accelerate down at near-minimum collective acceleration(throttle)
// 5. stopRotate	-	For stopping, accelerate up in opposite direction of rotating, similar to 'brake' of car
// 6. recovery		- 	recovery attitude, throttle
//
// in pidControl steps, We use manual flight mode.
// **********************

// **********************
// 플립 동작의 순서
// 1. pidControl	-	PID 컨트롤러를 이용하여 자세제어
// 2. acceleration	-	최고치에 근접하게 총 가속도(throttle)를 증가
// 3. startRotate	-	회전시작
// 4. coast			-	최저치에 근접하게 총 가속도(throttle)를 감소
// 5. stopRotate	-	회전을 멈추기 위해 도는방향에 반대방향으로 가속도를 증가, 자동차의 '제동걸기'와 유사
// 6. recovery		- 	자세와 총 가속도(Throttle) 회복
//
// pidControl 단계에서 수동 제어 모드(일반 모드)를 이용합니다.
// **********************

enum
{
	acceleration = 0,
	startRotate		,
	coast			,
	stopRotate		,
	recovery		,

	pidControl		,//연산의 편의상
};

/*************PID Controller for Flip**************/

// wrapper function for use pid_controller pointer as constant in flipState table
// flipState 테이블의 초기화 시 pid_controller 함수포인터로 초기화 하기 위한 래퍼 함수
void pidPrimary(void)
{
	pid_controller();
}

// fake PID controller. just fix axisPID array. (Ex : in coast step)
// 가짜 PID 컨트롤러, axisPID 배열을 강제로 고정할 뿐 (Ex : coast 스탭에서)
void fixedPID(void)
{
	axisPID[ROLL] = 0;
	axisPID[PITCH] = 0;
	axisPID[YAW] = 0;
}

static void pidGyrorate(void)
{
	int16_t gyroRate[] 		= {0,0};
	int16_t rcCommandRate[] = {0,0};

	int16_t errorRate[] 	= {0,0};
	//int16_t preErrorRate[]		= {0,0};
	int16_t PTerm[] 		= {0,0};
	//int16_t ITerm[] 		= {0,0};
	//int16_t DTerm[]			= {0,0};

	//ROLL, PITCH
	uint8_t axis;
	for(axis = 0; axis < 2; axis++)
	{
		//Gyroscope's output data : -8192~8192(-2000~2000 deg/s).
		//So, gyroRate[axis] means almost -2000~2000 deg/s
		gyroRate[axis] = gyroData[axis] >> 2;
		//rcCommand's output data : -500~500
		//So, rcCommandRate[axis] means (rcCommand[axis] * 4), almost same range with gyroRate[axis]'s
		rcCommandRate[axis] = rcCommand[axis] << 2;

		errorRate[axis] = rcCommandRate[axis] - gyroRate[axis];

		PTerm[axis] = (errorRate[axis] * 68) >> 7 ;

		/*
		ITerm[axis] += errorRate[axis] >> 7;
		ITerm[axis] = constrain(ITerm[axis], -1000, 1000); // Wind up

		DTerm[axis] = errorRate[axis] - preErrorRate[axis];
		DTerm[axis] = (DTerm[axis]) >> 2;
		preErrorRate[axis] = errorRate[axis];
		*/

		axisPID[axis] = PTerm[axis];// + ITerm[axis] + DTerm[axis];
	}
}

/**************************************************/

/***************Flip State Machine*****************/

//TODO 	: THROTTLE을 최소(1000)로 하여도 기울일 경우 모터에 파워가 인가되는것을 확인
//fix	: THROTTLE_RAW 항목(rcData[THROTTLE])을 강제로 변경하도록 flipState와 동작함수 변경하여 강제로 모터파워오프
//		: rcData[THROTTLE] 이 mcfg.mincheck 미만일 경우 모터가 강제로 최저속도가 되도록 하는 코드 이용,
//		: mixer.c 의 mixerTable 함수 내부 if ((rcData[THROTTLE]) < mcfg.mincheck) 구문 참조
// ROLL 			: rcCommand[ROLL] 		, for PID Controller
// PITCH 			: rcCommand[PITCH]		, for PID Controller
// YAW 				: rcCommand[YAW]		, for PID Controller
// THROTTLE 		: rcCommand[THROTTLE]	, for PID Controller
// PID Controller 	: PID Controller (Ex : pidMultiWii, pidRewrite)
// THROTTLE_RAW 	: rcData[THROTTLE]		, for Stopping Motor(Ex : coast step)

typedef struct flipSet
{
	int16_t 				roll;
	int16_t 				pitch;
	int16_t 				yaw;
	int16_t 				throttle;
	pidControllerFuncPtr 	pid;

	int16_t					throttleRaw;
} flipSet;

flipSet flipState[] =
{
		//ROLL	 PITCH	 YAW	 THROTTLE  PID Controller	THROTTLE_RAW
		{0		, 0		, 0		, 2000	, pidPrimary		, 2000		}, 	// acceleration
		{-200	, 0		, 0		, 1500	, pidGyrorate		, 1500		},	// startRotate
		{0		, 0		, 0		, 1150	, fixedPID			, 1150		}, 	// coast
		{-160	, 0		, 0		, 1500	, pidGyrorate		, 1500		},	// stopRotate
		{0		, 0		, 0		, 2000	, pidPrimary		, 2000		}   // recovery
};

// 동작함수, rcCommand, rcData에 넣을 값과 PID컨트롤러를 인자로 받는 함수
// 받은 인자를 통해 axisPID(PID를 통해 계산된 값)을 계산한다.
void axisPIDset(flipSet* flip)
{
	rcCommand[ROLL] 	= flip->roll;
	rcCommand[PITCH]	= flip->pitch;
	rcCommand[YAW] 		= flip->yaw;
	rcCommand[THROTTLE] = flip->throttle;

	flip->pid();

	rcData[THROTTLE] = flip->throttleRaw;
}

void flipStateMachine(uint8_t step)
{
	if(step == pidControl)
	{
		pidPrimary();
	}
	else
	{
		axisPIDset(&flipState[step]);
	}
}

/**************************************************/

/***************Flip Step Scheduler****************/

enum
{
	clockWise = 0,
	counterClockWIse,
};

uint8_t flipStateFlag = 0;

void flipFlagControl()
{
	flipStateFlag = 1;
}

int16_t angleAdjust(int16_t angle, uint8_t direction)
{
	if(direction == clockWise)
		return -angle;
	else
		return angle;
}

uint8_t gotoFlip()
{
	//if((rcCommand[THROTTLE] > 1400) && (rcCommand[ROLL] < -100))
	if(flipStateFlag)
	{
		flipStateFlag = 0;
		return acceleration;
	}
	else
		return pidControl;
}

uint8_t timePassed(uint8_t currentStep, uint8_t nextStep, uint16_t targetTime)
{
	static uint8_t timeTic = 0;
	// 1 timeTic means 3.5ms

	timeTic++;
	if(timeTic > targetTime)
	{
		timeTic = 0;
		return nextStep;
	}
	else
	{
		return currentStep;
	}
}

uint8_t anglePassed(uint8_t currentStep, uint8_t nextStep, int16_t currentAngle, int16_t targetAngle)
{
	if(targetAngle <0)
	{
		if((currentAngle < 0) && (currentAngle > targetAngle))
			return nextStep;
		else
			return currentStep;
	}
	else
	{
		if((currentAngle >= 0) && (currentAngle > targetAngle))
			return nextStep;
		else
			return currentStep;
	}
}

uint8_t flipStepScheduler()
{
	static uint8_t step = pidControl;
	static int16_t currentAngle;
	currentAngle = angleAdjust(angle[ROLL], clockWise);

	switch(step)
	{
	case pidControl:
		step = gotoFlip();
		break;

	case acceleration:
		step = timePassed(acceleration	, startRotate	, 143					);
		break;

	case startRotate:
		step = anglePassed(startRotate	, coast			, currentAngle	, 600	);
		break;

	case coast:
		step = anglePassed(coast		, stopRotate	, currentAngle	, -1500	);
		break;

	case stopRotate:
		step = anglePassed(stopRotate	, recovery		, currentAngle	, -200	);
		break;

	case recovery:
		step = timePassed(recovery		, pidControl	, 143					);
		break;
	}

	return step;
}

/**************************************************/

/*************for logging in flip******************/
uint8_t logFlag = 0;

void logFlagControl()
{
	logFlag = 1;
}

typedef struct logData
{
	uint8_t step;
	int16_t data;
} logData;

int16_t angle2(uint8_t step, int16_t angle)
{
	int16_t returnval = 0;

	switch(step)
	{
	case acceleration:
		returnval = -angle;
		break;
	case startRotate:
		returnval = -angle;
		break;
	case coast:
		if(angle < 0)
			returnval = -angle;
		else
			returnval = 3600-angle;
		break;
	case stopRotate:
		returnval = 3600-angle;
		break;
	case recovery:
		returnval = 3600-angle;
		break;
	}
	return returnval;
}

void logging(uint8_t step)
{
	//for logging
	static uint16_t lineNum = 0;
	static uint16_t lineMax = 0;
	static logData log[600];

	static uint8_t startLogging = 0;
	static uint8_t preStep = pidControl;

	//for logging
	if(preStep != step)
	{
		if(preStep == pidControl) //Start Logging in dump
			startLogging = 1;
		else if(step == pidControl) //Stop Logging in dump
			startLogging = 0;
	}
	preStep = step;

	if(startLogging)
	{
		static uint8_t timeTic = 0; // 1tic means 3.5ms
		timeTic++;
		if(timeTic > 1)
		{
			timeTic = 0;

			log[lineNum].step = step;
			log[lineNum].data = angle2(step,angle[ROLL]);
			//log[lineNum].data = angle[ROLL];
			//log[lineNum].data = gyroData[ROLL] >> 2;
			lineNum++;

			lineMax = lineNum;
		}
	}
	else
		lineNum = 0;


	//if(serialTotalBytesWaiting(core.debugport) > 0)
	if(logFlag)
	{
		static uint8_t timeTic = 0;
		static uint16_t lineCount = 0;
		uint8_t sendData[3];

		timeTic++;
		if(timeTic > 13)
		{
			timeTic = 0;

			sendData[0] = log[lineCount].step;
			sendData[1] = log[lineCount].data >> 8;
			sendData[2] = log[lineCount].data & 0xff;

			serialWrite(core.mainport, '$');
			serialWrite(core.mainport, 'M');
			serialWrite(core.mainport, sendData[0]);
			serialWrite(core.mainport, sendData[1]);
			serialWrite(core.mainport, sendData[2]);
			lineCount++;

			if(lineCount == lineMax)
			{
				lineCount = 0;
				logFlag = 0;
			}
		}
	}
}
/**************************************************/

static void pidFlip(void)
{
	static uint8_t flipStep;

	flipStep = flipStepScheduler();

	//logging(flipStep);

	flipStateMachine(flipStep);
}

// **********************
// FlipMode Area End
// **********************

void setPIDController(int type)
{
    switch (type) {
        case 0:
        default:
            pid_controller = pidMultiWii;
            break;
        case 1:
            pid_controller = pidRewrite;
            break;
    }
}

void loop(void)
{
    static uint8_t rcDelayCommand;      // this indicates the number of time (multiple of RC measurement at 50Hz) the sticks must be maintained to run or switch off motors
    static uint8_t rcSticks;            // this hold sticks position for command combos
    uint8_t stTmp = 0;
    int i;
    static uint32_t rcTime = 0;
#ifdef BARO
    static int16_t initialThrottleHold;
#endif
    static uint32_t loopTime;
    uint16_t auxState = 0;
    static uint8_t GPSNavReset = 1;
    bool isThrottleLow = false;
    bool rcReady = false;

    // calculate rc stuff from serial-based receivers (spek/sbus)
    if (feature(FEATURE_SERIALRX)) {
        switch (mcfg.serialrx_type) {
            case SERIALRX_SPEKTRUM1024:
            case SERIALRX_SPEKTRUM2048:
                rcReady = spektrumFrameComplete();
                break;
            case SERIALRX_SBUS:
                rcReady = sbusFrameComplete();
                break;
            case SERIALRX_SUMD:
                rcReady = sumdFrameComplete();
                break;
            #if defined(SKYROVER)
            case SERIALRX_HEXAIRBOT:
                rcReady = hexairbotFrameComplete();

                if( rcReady )
                {
                    LED1_TOGGLE;
                }
                break;                
            #endif
        }
    }

    if (((int32_t)(currentTime - rcTime) >= 0) || rcReady) { // 50Hz or data driven
        rcReady = false;
        rcTime = currentTime + 20000;
        computeRC();

        // in 3D mode, we need to be able to disarm by switch at any time
        if (feature(FEATURE_3D)) {
            if (!rcOptions[BOXARM])
                mwDisarm();
        }

        // Read value of AUX channel as rssi
        // 0 is disable, 1-4 is AUX{1..4}
        if (mcfg.rssi_aux_channel > 0) {
            const int16_t rssiChannelData = rcData[AUX1 + mcfg.rssi_aux_channel - 1];
            // Range of rssiChannelData is [1000;2000]. rssi should be in [0;1023];
            rssi = (uint16_t)((constrain(rssiChannelData - 1000, 0, 1000) / 1000.0f) * 1023.0f);
        }

        // Failsafe routine
        if (feature(FEATURE_FAILSAFE)) {
            if (failsafeCnt > (5 * cfg.failsafe_delay) && f.ARMED) { // Stabilize, and set Throttle to specified level
                for (i = 0; i < 3; i++)
                    rcData[i] = mcfg.midrc;      // after specified guard time after RC signal is lost (in 0.1sec)
                rcData[THROTTLE] = cfg.failsafe_throttle;
                if (failsafeCnt > 5 * (cfg.failsafe_delay + cfg.failsafe_off_delay)) {  // Turn OFF motors after specified Time (in 0.1sec)
                    mwDisarm();             // This will prevent the copter to automatically rearm if failsafe shuts it down and prevents
                    f.OK_TO_ARM = 0;        // to restart accidentely by just reconnect to the tx - you will have to switch off first to rearm
                }
                failsafeEvents++;
            }
            if (failsafeCnt > (5 * cfg.failsafe_delay) && !f.ARMED) {  // Turn off "Ok To arm to prevent the motors from spinning after repowering the RX with low throttle and aux to arm
                mwDisarm();         // This will prevent the copter to automatically rearm if failsafe shuts it down and prevents
                f.OK_TO_ARM = 0;    // to restart accidentely by just reconnect to the tx - you will have to switch off first to rearm
            }
            failsafeCnt++;
        }
        // end of failsafe routine - next change is made with RcOptions setting

        // ------------------ STICKS COMMAND HANDLER --------------------
        // checking sticks positions
        for (i = 0; i < 4; i++) {
            stTmp >>= 2;
            if (rcData[i] > mcfg.mincheck)
                stTmp |= 0x80;  // check for MIN
            if (rcData[i] < mcfg.maxcheck)
                stTmp |= 0x40;  // check for MAX
        }
        if (stTmp == rcSticks) {
            if (rcDelayCommand < 250)
                rcDelayCommand++;
        } else
            rcDelayCommand = 0;
        rcSticks = stTmp;

        // perform actions
        if (feature(FEATURE_3D) && (rcData[THROTTLE] > (mcfg.midrc - mcfg.deadband3d_throttle) && rcData[THROTTLE] < (mcfg.midrc + mcfg.deadband3d_throttle)))
            isThrottleLow = true;
        else if (!feature(FEATURE_3D) && (rcData[THROTTLE] < mcfg.mincheck))
            isThrottleLow = true;
        if (isThrottleLow) {
            errorGyroI[ROLL] = 0;
            errorGyroI[PITCH] = 0;
            errorGyroI[YAW] = 0;
            errorAngleI[ROLL] = 0;
            errorAngleI[PITCH] = 0;
            if (cfg.activate[BOXARM] > 0) { // Arming/Disarming via ARM BOX
                if (rcOptions[BOXARM] && f.OK_TO_ARM)
                    mwArm();
                else if (f.ARMED)
                    mwDisarm();
            }
        }

        if (rcDelayCommand == 20) {
            if (f.ARMED) {      // actions during armed
                // Disarm on throttle down + yaw
                if (cfg.activate[BOXARM] == 0 && (rcSticks == THR_LO + YAW_LO + PIT_CE + ROL_CE))
                    mwDisarm();
                // Disarm on roll (only when retarded_arm is enabled)
                if (mcfg.retarded_arm && cfg.activate[BOXARM] == 0 && (rcSticks == THR_LO + YAW_CE + PIT_CE + ROL_LO))
                    mwDisarm();
            } else {            // actions during not armed
                i = 0;
                // GYRO calibration
                if (rcSticks == THR_LO + YAW_LO + PIT_LO + ROL_CE) {
                    calibratingG = CALIBRATING_GYRO_CYCLES;
                    if (feature(FEATURE_GPS))
                        GPS_reset_home_position();
                    if (sensors(SENSOR_BARO))
                        calibratingB = 10; // calibrate baro to new ground level (10 * 25 ms = ~250 ms non blocking)
                    if (!sensors(SENSOR_MAG))
                        heading = 0; // reset heading to zero after gyro calibration
                // Inflight ACC Calibration
                } else if (feature(FEATURE_INFLIGHT_ACC_CAL) && (rcSticks == THR_LO + YAW_LO + PIT_HI + ROL_HI)) {
                    if (AccInflightCalibrationMeasurementDone) {        // trigger saving into eeprom after landing
                        AccInflightCalibrationMeasurementDone = 0;
                        AccInflightCalibrationSavetoEEProm = 1;
                    } else {
                        AccInflightCalibrationArmed = !AccInflightCalibrationArmed;
                        if (AccInflightCalibrationArmed) {
                            toggleBeep = 2;
                        } else {
                            toggleBeep = 3;
                        }
                    }
                }

                // Multiple configuration profiles
                if (rcSticks == THR_LO + YAW_LO + PIT_CE + ROL_LO)          // ROLL left  -> Profile 1
                    i = 1;
                else if (rcSticks == THR_LO + YAW_LO + PIT_HI + ROL_CE)     // PITCH up   -> Profile 2
                    i = 2;
                else if (rcSticks == THR_LO + YAW_LO + PIT_CE + ROL_HI)     // ROLL right -> Profile 3
                    i = 3;
                if (i) {
                    mcfg.current_profile = i - 1;
                    writeEEPROM(0, false);
                    blinkLED(2, 40, i);
                    // TODO alarmArray[0] = i;
                }

                // Arm via YAW
                if (cfg.activate[BOXARM] == 0 && (rcSticks == THR_LO + YAW_HI + PIT_CE + ROL_CE))
                    mwArm();
                // Arm via ROLL
                else if (mcfg.retarded_arm && cfg.activate[BOXARM] == 0 && (rcSticks == THR_LO + YAW_CE + PIT_CE + ROL_HI))
                    mwArm();
                // Calibrating Acc
                else if (rcSticks == THR_HI + YAW_LO + PIT_LO + ROL_CE)
                    calibratingA = CALIBRATING_ACC_CYCLES;
                // Calibrating Mag
                else if (rcSticks == THR_HI + YAW_HI + PIT_LO + ROL_CE)
                    f.CALIBRATE_MAG = 1;
                i = 0;
                // Acc Trim
                if (rcSticks == THR_HI + YAW_CE + PIT_HI + ROL_CE) {
                    cfg.angleTrim[PITCH] += 2;
                    i = 1;
                } else if (rcSticks == THR_HI + YAW_CE + PIT_LO + ROL_CE) {
                    cfg.angleTrim[PITCH] -= 2;
                    i = 1;
                } else if (rcSticks == THR_HI + YAW_CE + PIT_CE + ROL_HI) {
                    cfg.angleTrim[ROLL] += 2;
                    i = 1;
                } else if (rcSticks == THR_HI + YAW_CE + PIT_CE + ROL_LO) {
                    cfg.angleTrim[ROLL] -= 2;
                    i = 1;
                }
                if (i) {
                    writeEEPROM(1, true);
                    rcDelayCommand = 0; // allow autorepetition
                }
            }
        }

        if (feature(FEATURE_INFLIGHT_ACC_CAL)) {
            if (AccInflightCalibrationArmed && f.ARMED && rcData[THROTTLE] > mcfg.mincheck && !rcOptions[BOXARM]) {   // Copter is airborne and you are turning it off via boxarm : start measurement
                InflightcalibratingA = 50;
                AccInflightCalibrationArmed = 0;
            }
            if (rcOptions[BOXCALIB]) {      // Use the Calib Option to activate : Calib = TRUE Meausrement started, Land and Calib = 0 measurement stored
                if (!AccInflightCalibrationActive && !AccInflightCalibrationMeasurementDone)
                    InflightcalibratingA = 50;
            } else if (AccInflightCalibrationMeasurementDone && !f.ARMED) {
                AccInflightCalibrationMeasurementDone = 0;
                AccInflightCalibrationSavetoEEProm = 1;
            }
        }

        // Check AUX switches
        for (i = 0; i < 4; i++)
            auxState |= (rcData[AUX1 + i] < 1300) << (3 * i) | (1300 < rcData[AUX1 + i] && rcData[AUX1 + i] < 1700) << (3 * i + 1) | (rcData[AUX1 + i] > 1700) << (3 * i + 2);
        for (i = 0; i < CHECKBOXITEMS; i++)
            rcOptions[i] = (auxState & cfg.activate[i]) > 0;

        // note: if FAILSAFE is disable, failsafeCnt > 5 * FAILSAVE_DELAY is always false
        if ((rcOptions[BOXANGLE] || (failsafeCnt > 5 * cfg.failsafe_delay)) && (sensors(SENSOR_ACC))) {
            // bumpless transfer to Level mode
            if (!f.ANGLE_MODE) {
                errorAngleI[ROLL] = 0;
                errorAngleI[PITCH] = 0;
                f.ANGLE_MODE = 1;
            }
        } else {
            f.ANGLE_MODE = 0;        // failsave support
        }

        if (rcOptions[BOXHORIZON]) {
            f.ANGLE_MODE = 0;
            if (!f.HORIZON_MODE) {
                errorAngleI[ROLL] = 0;
                errorAngleI[PITCH] = 0;
                f.HORIZON_MODE = 1;
            }
        } else {
            f.HORIZON_MODE = 0;
        }

        if ((rcOptions[BOXARM]) == 0)
            f.OK_TO_ARM = 1;
        if (f.ANGLE_MODE || f.HORIZON_MODE) {
            LED1_ON;
        } else {
            LED1_OFF;
        }

#ifdef BARO
        if (sensors(SENSOR_BARO)) {
            // Baro alt hold activate
            if (rcOptions[BOXBARO]) {
                if (!f.BARO_MODE) {
                    f.BARO_MODE = 1;
                    AltHold = EstAlt;
                    initialThrottleHold = rcCommand[THROTTLE];
                    errorAltitudeI = 0;
                    BaroPID = 0;
                }
            } else {
                f.BARO_MODE = 0;
            }
            // Vario signalling activate
            if (feature(FEATURE_VARIO)) {
                if (rcOptions[BOXVARIO]) {
                    if (!f.VARIO_MODE) {
                        f.VARIO_MODE = 1;
                    }
                } else {
                    f.VARIO_MODE = 0;
                }
            }
        }
#endif

#ifdef  MAG
        if (sensors(SENSOR_ACC) || sensors(SENSOR_MAG)) {
            if (rcOptions[BOXMAG]) {
                if (!f.MAG_MODE) {
                    f.MAG_MODE = 1;
                    magHold = heading;
                }
            } else {
                f.MAG_MODE = 0;
            }
            if (rcOptions[BOXHEADFREE]) {
                if (!f.HEADFREE_MODE) {
                    f.HEADFREE_MODE = 1;
                }
            } else {
                f.HEADFREE_MODE = 0;
            }
            if (rcOptions[BOXHEADADJ]) {
                headFreeModeHold = heading; // acquire new heading
            }
        }
#endif


#ifdef ACC_AS_MAG
        if (sensors(SENSOR_ACC))
        {
            if (rcOptions[BOXHEADFREE])
            {
                if (!f.HEADFREE_MODE)
                {
                    f.HEADFREE_MODE = 1;
                }
            }
            else
            {
                f.HEADFREE_MODE = 0;
            }
            if (rcOptions[BOXHEADADJ])
            {
                headFreeModeHold = heading; // acquire new heading
            }
        }
#endif


        if (sensors(SENSOR_GPS)) {
            if (f.GPS_FIX && GPS_numSat >= 5) {
                // if both GPS_HOME & GPS_HOLD are checked => GPS_HOME is the priority
                if (rcOptions[BOXGPSHOME]) {
                    if (!f.GPS_HOME_MODE) {
                        f.GPS_HOME_MODE = 1;
                        f.GPS_HOLD_MODE = 0;
                        GPSNavReset = 0;
                        GPS_set_next_wp(&GPS_home[LAT], &GPS_home[LON]);
                        nav_mode = NAV_MODE_WP;
                    }
                } else {
                    f.GPS_HOME_MODE = 0;
                    if (rcOptions[BOXGPSHOLD] && abs(rcCommand[ROLL]) < cfg.ap_mode && abs(rcCommand[PITCH]) < cfg.ap_mode) {
                        if (!f.GPS_HOLD_MODE) {
                            f.GPS_HOLD_MODE = 1;
                            GPSNavReset = 0;
                            GPS_hold[LAT] = GPS_coord[LAT];
                            GPS_hold[LON] = GPS_coord[LON];
                            GPS_set_next_wp(&GPS_hold[LAT], &GPS_hold[LON]);
                            nav_mode = NAV_MODE_POSHOLD;
                        }
                    } else {
                        f.GPS_HOLD_MODE = 0;
                        // both boxes are unselected here, nav is reset if not already done
                        if (GPSNavReset == 0) {
                            GPSNavReset = 1;
                            GPS_reset_nav();
                        }
                    }
                }
            } else {
                f.GPS_HOME_MODE = 0;
                f.GPS_HOLD_MODE = 0;
                nav_mode = NAV_MODE_NONE;
            }
        }

        if (rcOptions[BOXPASSTHRU]) {
            f.PASSTHRU_MODE = 1;
        } else {
            f.PASSTHRU_MODE = 0;
        }

        if (mcfg.mixerConfiguration == MULTITYPE_FLYING_WING || mcfg.mixerConfiguration == MULTITYPE_AIRPLANE) {
            f.HEADFREE_MODE = 0;
        }
    } else {                    // not in rc loop
        static int taskOrder = 0;    // never call all function in the same loop, to avoid high delay spikes
        if (taskOrder > 4)
            taskOrder -= 5;
        switch (taskOrder) {
        case 0:
            taskOrder++;
#ifdef MAG
            if (sensors(SENSOR_MAG) && Mag_getADC())
                break;
#endif
        case 1:
            taskOrder++;
#ifdef BARO
            if (sensors(SENSOR_BARO) && Baro_update())
                break;
#endif
        case 2:
            taskOrder++;
#ifdef BARO
            if (sensors(SENSOR_BARO) && getEstimatedAltitude())
                break;
#endif
        case 3:
            // if GPS feature is enabled, gpsThread() will be called at some intervals to check for stuck
            // hardware, wrong baud rates, init GPS if needed, etc. Don't use SENSOR_GPS here as gpsThread() can and will
            // change this based on available hardware
            taskOrder++;
            if (feature(FEATURE_GPS)) {
                gpsThread();
                break;
            }
        case 4:
            taskOrder++;
#ifdef SONAR
            if (sensors(SENSOR_SONAR)) {
                Sonar_update();
            }
#endif
            if (feature(FEATURE_VARIO) && f.VARIO_MODE)
                mwVario();
            break;
        }
    }

    currentTime = micros();
    if (mcfg.looptime == 0 || (int32_t)(currentTime - loopTime) >= 0) {
        loopTime = currentTime + mcfg.looptime;

        computeIMU();
        annexCode();
        // Measure loop rate just afer reading the sensors
        currentTime = micros();
        cycleTime = (int32_t)(currentTime - previousTime);
        previousTime = currentTime;

#ifdef MAG
        if (sensors(SENSOR_MAG)) {
            if (abs(rcCommand[YAW]) < 70 && f.MAG_MODE) {
                int16_t dif = heading - magHold;
                if (dif <= -180)
                    dif += 360;
                if (dif >= +180)
                    dif -= 360;
                dif *= -mcfg.yaw_control_direction;
                if (f.SMALL_ANGLES_25)
                    rcCommand[YAW] -= dif * cfg.P8[PIDMAG] / 30;    // 18 deg
            } else
                magHold = heading;
        }
#endif

#ifdef BARO
        if (sensors(SENSOR_BARO)) {
            if (f.BARO_MODE) {
                static uint8_t isAltHoldChanged = 0;
                static int16_t AltHoldCorr = 0;
                if (!f.FIXED_WING) {
                    // multirotor alt hold
                    if (cfg.alt_hold_fast_change) {
                        // rapid alt changes
                        if (abs(rcCommand[THROTTLE] - initialThrottleHold) > cfg.alt_hold_throttle_neutral) {
                            errorAltitudeI = 0;
                            isAltHoldChanged = 1;
                            rcCommand[THROTTLE] += (rcCommand[THROTTLE] > initialThrottleHold) ? -cfg.alt_hold_throttle_neutral : cfg.alt_hold_throttle_neutral;
                        } else {
                            if (isAltHoldChanged) {
                                AltHold = EstAlt;
                                isAltHoldChanged = 0;
                            }
                            rcCommand[THROTTLE] = initialThrottleHold + BaroPID;
                        }
                    } else {
                        // slow alt changes for apfags
                        if (abs(rcCommand[THROTTLE] - initialThrottleHold) > cfg.alt_hold_throttle_neutral) {
                            // Slowly increase/decrease AltHold proportional to stick movement ( +100 throttle gives ~ +50 cm in 1 second with cycle time about 3-4ms)
                            AltHoldCorr += rcCommand[THROTTLE] - initialThrottleHold;
                            AltHold += AltHoldCorr / 2000;
                            AltHoldCorr %= 2000;
                            isAltHoldChanged = 1;
                        } else if (isAltHoldChanged) {
                            AltHold = EstAlt;
                            AltHoldCorr = 0;
                            isAltHoldChanged = 0;
                        }
                        rcCommand[THROTTLE] = initialThrottleHold + BaroPID;
                        rcCommand[THROTTLE] = constrain(rcCommand[THROTTLE], mcfg.minthrottle + 150, mcfg.maxthrottle);
                    }
                } else {
                    // handle fixedwing-related althold. UNTESTED! and probably wrong
                    // most likely need to check changes on pitch channel and 'reset' althold similar to
                    // how throttle does it on multirotor
                    rcCommand[PITCH] += BaroPID * mcfg.fixedwing_althold_dir;
                }
            }
        }
#endif

        if (cfg.throttle_angle_correction && (f.ANGLE_MODE || f.HORIZON_MODE)) {
            rcCommand[THROTTLE] += throttleAngleCorrection;
        }

        if (sensors(SENSOR_GPS)) {
            if ((f.GPS_HOME_MODE || f.GPS_HOLD_MODE) && f.GPS_FIX_HOME) {
                float sin_yaw_y = sinf(heading * 0.0174532925f);
                float cos_yaw_x = cosf(heading * 0.0174532925f);
                if (cfg.nav_slew_rate) {
                    nav_rated[LON] += constrain(wrap_18000(nav[LON] - nav_rated[LON]), -cfg.nav_slew_rate, cfg.nav_slew_rate); // TODO check this on uint8
                    nav_rated[LAT] += constrain(wrap_18000(nav[LAT] - nav_rated[LAT]), -cfg.nav_slew_rate, cfg.nav_slew_rate);
                    GPS_angle[ROLL] = (nav_rated[LON] * cos_yaw_x - nav_rated[LAT] * sin_yaw_y) / 10;
                    GPS_angle[PITCH] = (nav_rated[LON] * sin_yaw_y + nav_rated[LAT] * cos_yaw_x) / 10;
                } else {
                    GPS_angle[ROLL] = (nav[LON] * cos_yaw_x - nav[LAT] * sin_yaw_y) / 10;
                    GPS_angle[PITCH] = (nav[LON] * sin_yaw_y + nav[LAT] * cos_yaw_x) / 10;
                }
            }
        }

        // PID - note this is function pointer set by setPIDController()
        //pid_controller(); //original code for PID controller select
        pidFlip(); //code for flip. but in normal state, use 'pid_controller' pointer
        /*
         * pidFlip모드 사용시 주의사항
         * Flip모드 사용중에 임의로 rcCommand 배열의 ROLL, PITCH, YAW, THROTTLE 값을 바꿉니다.
         * 조종기로 ROLL, PITCH, YAW, THROTTLE 명령을 주어 플립을 직접 하는것처럼
         * 가상으로 rcCommand 값을 이용하여 자동으로 플립을 구현하였습니다.
         * 사용시 rcCommand 배열의 ROLL, PITCH, YAW, THROTTLE 관련 작업시 주의해주세요.
         * 추가변경사항 : rcData 배열의 THROTTLE 또한 일시적으로 변경합니다(모터를 강제로 멈추기 위함)
         * 				: PID까지 거친 다음에 변경하므로 모터를 멈추는 동작에만 관여할 것으로 보입니다.
         *
         * Attention for pidFlip Mode
         * Flip mode changes 'rcCommand' array(ROLL, PITCH, YAW, THROTTLE).
         * Like Manual Flip, It use rcCommand for implementation of Flip
         * So, If use rcCommand(ROLL, PITCH, YAW, THROTTLE) to another work, be careful
         * Additional Change 	: It changes rcData array(THROTTLE), because of Stopping motors
         * 						: because PID control use rcCommand, Changing rcData array only relevant to Stopping motor
         */

        mixTable();

        /* For seeing some data of drone
         * debugCommand	: 'rcCommand' array
         * debugAngle	: estimated Angles, range [-1800:1800]
         * debugMotor	: distributed motor power '1000' means 'no power'
         * debugRcdata	: 'rcData' array
         * debugAxisPID	: 'axisPID' array, output data of PID controller
         *
         * '20' means 20*3.5 millisecond(printing period)
         * Use ONE debugPrint.
         */
        //debugPrint(debugCommand, 20);
        //debugPrint(debugAngle, 20);
        //debugPrint(debugMotor, 20);
        //debugPrint(debugRcdata, 20);
		//debugPrint(debugAxisPID, 20);
		//debugPrint(debugAngleRate, 25);

        //writeServos(); //No need to use writeServos, We only use motors
        writeMotors(); //write PWM to motors. If you don't want to enable motor, just use '//'
    }
}
