#include "main.h"

//#include "lib7842/api.hpp"
//dead - 11,12,9,14,19,20,16.17
//using namespace lib7842;
using namespace okapi;

// ***** Drive Definitions Below *****
#define BACK_RIGHT_MOTOR_PORT 5
#define FRONT_RIGHT_MOTOR_PORT 21
#define BACK_LEFT_MOTOR_PORT 8
#define FRONT_LEFT_MOTOR_PORT 1
#define LEFT_DRIVE_REVERSE_STATE false
#define RIGHT_DRIVE_REVERSE_STATE true
#define DRIVE_MOTOR_GEARSET AbstractMotor::gearset::green
#define DRIVE_MOTOR_POLLING_RATE imev5GreenTPR
// ***** Drive Definitions Above *****
// ***** Transition *****
// ***** Intake Definitions Below *****
#define INTAKE_LEFT_MOTOR_PORT 2
#define INTAKE_RIGHT_MOTOR_PORT 18
#define INTAKE_MOTOR_GEARSET AbstractMotor::gearset::green
// ***** Intake Definitions Above *****
// ***** Transition *****
// ***** Arm Definitions Below *****
#define ARM_MOTOR_PORT 13
#define ARM_MOTOR_GEARSET AbstractMotor::gearset::red
// ***** Arm Definitions Above *****
// ***** Transition *****
// ***** Tray Definitions Below *****
#define TRAY_MOTOR_PORT 15
#define TRAY_MOTOR_GEARSET AbstractMotor::gearset::red
// ***** Tray Definitions Above *****
// ***** Transition *****
// ***** Chassis Definitions Below *****
#define WHEEL_DIAMETER 4_in
#define WHEEL_TRACK 12.6_in
#define MAX_VELOCITY_SLOW 0.4
#define MAX_ACCELERATION_SLOW 0.5
#define MAX_JERK_SLOW 1.0
#define MAX_VELOCITY_FAST 0.6
#define MAX_ACCELERATION_FAST 1.0
#define MAX_JERK_FAST 5.0
#define ERROR_TIME_UTIL 50
#define DERIVATIVE_TIME_UTIL 5
#define TIME_TIME_UTIL 250_ms
#define DRIVE_GEAR_RATIO 1.0
// ***** Chassis Definitions Above *****


// ***** Transition *****


// ***** Auton Definitions Below *****

#define GLOBAL_DELAY 10	//In milliseconds

// ***** Auton Definitions Above *****


// ***** Transition *****


// ***** Generic Objects/Variables Below *****

Controller master;

// ***** Generic Objects/Variables Below *****


// ***** Transition *****


// ***** Drive Objects/Variables Below *****

Motor backLeftMotor
(
	BACK_LEFT_MOTOR_PORT,
	LEFT_DRIVE_REVERSE_STATE,
	DRIVE_MOTOR_GEARSET,
	AbstractMotor::encoderUnits::degrees
);

Motor frontLeftMotor
(
	FRONT_LEFT_MOTOR_PORT,
	LEFT_DRIVE_REVERSE_STATE,
	DRIVE_MOTOR_GEARSET,
	AbstractMotor::encoderUnits::degrees
);

Motor backRightMotor
(
	BACK_RIGHT_MOTOR_PORT,
	RIGHT_DRIVE_REVERSE_STATE,
	DRIVE_MOTOR_GEARSET,
	AbstractMotor::encoderUnits::degrees
);

Motor frontRightMotor
(
	FRONT_RIGHT_MOTOR_PORT,
	RIGHT_DRIVE_REVERSE_STATE,
	DRIVE_MOTOR_GEARSET,
	AbstractMotor::encoderUnits::degrees
);

MotorGroup leftDriveMotors ( { backLeftMotor, frontLeftMotor } );
MotorGroup rightDriveMotors ( { backRightMotor, frontRightMotor } );

// ***** Drive Objects/Variables Above *****


// ***** Transition *****


// ***** Intake Objects/Variables Below *****

Motor intakeLeft
(
	INTAKE_LEFT_MOTOR_PORT,
	false,
	INTAKE_MOTOR_GEARSET,
	AbstractMotor::encoderUnits::degrees
);

Motor intakeRight
(
	INTAKE_RIGHT_MOTOR_PORT,
	true,
	INTAKE_MOTOR_GEARSET,
	AbstractMotor::encoderUnits::degrees
);

MotorGroup intakeMotors ( { intakeLeft, intakeRight } );

// ***** Intake Objects/Variables Above *****


// ***** Transition *****


// ***** Arm Objects/Variables Below *****

Motor armMotor
(
	ARM_MOTOR_PORT,
	false,
	ARM_MOTOR_GEARSET,
	AbstractMotor::encoderUnits::degrees
);

// ***** Arm Objects/Variables Above *****


// ***** Transition *****


// ***** Tray Objects/Variables Below *****

Motor trayMotor
(
	TRAY_MOTOR_PORT,
	false,
	TRAY_MOTOR_GEARSET,
	AbstractMotor::encoderUnits::degrees
);

// ***** Tray Objects/Variables Above *****


// ***** Transition *****


// ***** Controller Objects/Variables Below *****

std::shared_ptr < ChassisController > chassis;

std::shared_ptr < AsyncPositionController < double, double > > armController;

std::shared_ptr < AsyncPositionController < double, double > > trayController;

// ***** Controller Objects/Variables Above *****


// ***** Transition *****


// ***** Auton Objects/Variables Below *****

pros::ADIDigitalIn autonComponentOne ( 1 );		//Out = Blue, In = Red
pros::ADIDigitalIn autonComponentTwo ( 2 );		//Out = Small Goal, In = Large Goal
pros::ADIDigitalIn autonComponentThree ( 3 );	//Out = Type 1, In = Type 2
pros::ADIDigitalIn autonComponentFour ( 4 );

int autonVariable;

bool intakeLock = false;

int activeIntakeVoltage = 12000;
int passiveIntakeVoltage = 1200;

//bool autonColor = false;
//bool autonSide = false;
//bool autonType = false;

//bool autonLockIn = false;

// ***** Auton Objects/Variables Above *****


// ***** Transition *****


void initialize ()
{
	//autonSelector ();

	//backLeftMotor.setBrakeMode ( AbstractMotor::brakeMode::coast );
	//frontLeftMotor.setBrakeMode ( AbstractMotor::brakeMode::coast );

	//backRightMotor.setBrakeMode ( AbstractMotor::brakeMode::coast );
	//frontRightMotor.setBrakeMode ( AbstractMotor::brakeMode::coast );

	pros::lcd::initialize ();

	pros::delay ( 500 );

	chassis = ChassisControllerBuilder ()
		.withMotors ( leftDriveMotors, rightDriveMotors )
		.withSensors ( IntegratedEncoder ( backLeftMotor ), IntegratedEncoder ( backRightMotor ) )
		.withDimensions ( DRIVE_MOTOR_GEARSET, { { WHEEL_DIAMETER, WHEEL_TRACK }, DRIVE_MOTOR_POLLING_RATE } )
		.withClosedLoopControllerTimeUtil ( ERROR_TIME_UTIL, DERIVATIVE_TIME_UTIL, TIME_TIME_UTIL )
		.build ();

	armController = AsyncPosControllerBuilder ()
		.withMotor ( armMotor )
		.withGains ( { 0.0050, 0.0, 0.000010 } )
		.withGearset ( { ARM_MOTOR_GEARSET, 5.0 } )
		.withTimeUtilFactory ( TimeUtilFactory () )
		.build ();

	trayController = AsyncPosControllerBuilder ()
		.withMotor ( trayMotor )
		.withGains ( { 0.0020, 0.0, 0.0 } )
		.withGearset ( { TRAY_MOTOR_GEARSET, 7.0 } )
		.withTimeUtilFactory ( TimeUtilFactory () )
		.build ();

	pros::delay ( 1000 );

	autonVariable = ( 1 * autonComponentOne.get_value () ) + ( 2 * autonComponentTwo.get_value () ) + ( 4 * autonComponentThree.get_value () ) + ( 8 * autonComponentFour.get_value () );

	pros::delay ( 500 );

	pros::lcd::set_text ( 0, std::to_string ( autonVariable ) );

	pros::delay ( 500 );

	master.rumble ( "-" );
}

void disabled ()
{

}

void competition_initialize ()
{

}

void autonomous ()
{
	switch ( autonVariable )
	{
		case 0://blue small
		{
			chassis -> setMaxVelocity ( 55 );
			prepareDriveMotors ();
			pros::delay ( 100 );
			chassis -> moveDistanceAsync ( 3.5_ft ); //pick up 5
			pros::delay(500);
			autonSetMotorValues ( 2, 12000 );
			chassis -> waitUntilSettled ();
			autonSetMotorValues ( 2, 1200 );
			chassis -> turnAngle ( 20_deg );//turn towards 6th
			autonSetMotorValues ( 2, 10500 );
			chassis -> moveDistance ( 0.6_ft ); //pick up last
			autonSetMotorValues ( 2, 1200 );
			chassis -> moveDistance( -0.3_ft);
			chassis -> waitUntilSettled ();
			chassis -> setMaxVelocity ( 92 );
			chassis -> turnAngle ( -175_deg ); //turn to goal
			chassis -> waitUntilSettled ();
			chassis -> setMaxVelocity ( 170 );
			chassis -> moveDistanceAsync ( 2.8_ft ); //go t stack
			chassis -> waitUntilSettled ();
			pros::delay(300);
			intakeMotors.moveRelative ( -370, -127 );
			trayController -> setTarget ( 91.0 ); //stack
			trayController -> waitUntilSettled ();
			pros::delay ( 2000 );
			chassis -> setMaxVelocity ( 100 );
			intakeMotors.moveRelative ( -1000, -120 );
			pros::delay(300);
			chassis -> moveDistanceAsync ( -1_ft ); //back away
			chassis -> waitUntilSettled ();
			break;
		}
		case 1:{//red small
			chassis -> setMaxVelocity ( 55 );
			prepareDriveMotors ();
			pros::delay ( 100 );
			chassis -> moveDistanceAsync ( 3.5_ft ); //pick up 5
			pros::delay(500);
			autonSetMotorValues ( 2, 12000 );
			chassis -> waitUntilSettled ();
			autonSetMotorValues ( 2, 1200 );
			chassis -> turnAngle ( -20_deg );//turn towards 6th
			autonSetMotorValues ( 2, 10500 );
			chassis -> moveDistance ( 0.6_ft ); //pick up last
			autonSetMotorValues ( 2, 1200 );
			chassis -> moveDistance( -0.3_ft);
			chassis -> waitUntilSettled ();
			chassis -> setMaxVelocity ( 90 );
			chassis -> turnAngle ( 173_deg ); //turn to goal
			chassis -> waitUntilSettled ();
			chassis -> setMaxVelocity ( 170 );
			chassis -> moveDistanceAsync ( 2.8_ft ); //go t stack
			chassis -> waitUntilSettled ();
			intakeMotors.moveRelative ( -350, -127 );
			trayController -> setTarget ( 90.0 ); //stack
			trayController -> waitUntilSettled ();
			pros::delay ( 2000 );
			chassis -> setMaxVelocity ( 100 );
			intakeMotors.moveRelative ( -1000, -80 );
			pros::delay(300);
			chassis -> moveDistanceAsync ( -1_ft ); //back away
			chassis -> waitUntilSettled ();
			break;
		}
		case 2:{//blue Large
			chassis -> setMaxVelocity ( 75 );
			prepareDriveMotors ();
			pros::delay ( 100 );
			autonSetMotorValues ( 2, 12000 );
			chassis -> moveDistanceAsync ( 1.7_ft ); //pick up 2nd
			chassis -> waitUntilSettled ();
			//autonSetMotorValues ( 2, 1200 );
			chassis -> turnAngleAsync ( -103_deg ); //turn to 3rd and 4th
			chassis -> waitUntilSettled ();
			chassis -> setMaxVelocity ( 80 );
			autonSetMotorValues ( 2, 12000 );
			chassis -> moveDistance ( 3.0_ft ); //pick up 4th
			chassis -> waitUntilSettled ();
			pros::delay(500);
			autonSetMotorValues ( 2, 1200 );
			chassis -> setMaxVelocity ( 120 );
			chassis -> moveDistance ( -2.0_ft ); //go to scoring zone
			chassis -> waitUntilSettled ();
			chassis -> turnAngle ( -125_deg ); //turn to stack
			chassis -> waitUntilSettled ();
			chassis -> moveDistance ( 1.3_ft ); //go to stack
			chassis -> waitUntilSettled ();
			intakeMotors.moveRelative ( -300, -127 );
			trayController -> setTarget ( 90.0 ); //stack
			trayController -> waitUntilSettled ();
			pros::delay ( 2000 );
			chassis -> setMaxVelocity ( 100 );
			intakeMotors.moveRelative ( -1000, -100 );
			pros::delay(300);
			chassis -> moveDistanceAsync ( -1_ft ); //back away
			chassis -> waitUntilSettled ();
			break;
		}
		case 3:{//red Large
			chassis -> setMaxVelocity ( 75 );
			prepareDriveMotors ();
			pros::delay ( 100 );
			autonSetMotorValues ( 2, 12000 );
			chassis -> moveDistanceAsync ( 1.7_ft ); //pick up 2nd
			pros::delay( 1300 );
			autonSetMotorValues ( 2, 1200 );
			chassis -> waitUntilSettled ();
			chassis -> turnAngleAsync ( 103_deg ); //turn to 3rd and 4th
			chassis -> waitUntilSettled ();
			chassis -> setMaxVelocity ( 80 );
			autonSetMotorValues ( 2, 11500 );
			chassis -> moveDistance ( 3.2_ft ); //pick up 4th
			chassis -> waitUntilSettled ();
			pros::delay(500);
			autonSetMotorValues ( 2, 1200 );
			chassis -> setMaxVelocity ( 120 );
			chassis -> moveDistance ( -2.5_ft ); //go to scoring zone
			chassis -> waitUntilSettled ();
			chassis -> turnAngleAsync ( 120_deg ); //turn to stack
			chassis -> waitUntilSettled ();
			chassis -> moveDistance ( 0.93_ft ); //go to stack
			chassis -> waitUntilSettled ();
			intakeMotors.moveRelative ( -150, -127 );
			trayController -> setTarget ( 90.0 ); //stack
			trayController -> waitUntilSettled ();
			pros::delay ( 2000 );
			chassis -> setMaxVelocity ( 100 );
			intakeMotors.moveRelative ( -1000, -120 );
			pros::delay(400);
			chassis -> moveDistanceAsync ( -1_ft ); //back away
			chassis -> waitUntilSettled ();
			break;
		}
		case 4:{//push auton
			chassis -> setMaxVelocity ( 100 );
			prepareDriveMotors ();
			pros::delay ( 200 );
			chassis -> moveDistance ( -0.6_ft ); //push cube in
			chassis -> waitUntilSettled ();
			chassis -> setMaxVelocity ( 100 );
			prepareDriveMotors ();
			pros::delay ( 200 );
			chassis -> moveDistance ( 1.0_ft ); //get away
			chassis -> waitUntilSettled ();
			break;
		}
		case 5:{//prog skills
			chassis -> setMaxVelocity ( 50 );
			prepareDriveMotors ();
			pros::delay ( 100 );
			chassis -> moveDistanceAsync ( 2.0_ft ); //pick up 5
			pros::delay(300);
			autonSetMotorValues ( 2, 10000 );
			chassis -> waitUntilSettled ();//wait for deploy
			pros::delay(1000);
			chassis -> moveDistance(1.5_ft);//pick up rest
			chassis -> turnAngle ( -20_deg );//turn to tower
			chassis -> moveDistance ( 0.8_ft );//pick up 6th
			chassis -> moveDistance (-0.4_ft);
			pros::delay(500);
			autonSetMotorValues ( 2, 1200 );
			intakeMotors.moveRelative ( -170, -127 );//outtake for cube lock
			armController -> setTarget ( 90.0 ); //lift cube
			armController -> waitUntilSettled ();
			pros::delay(1500);
			intakeMotors.moveRelative ( -1300, -127 ); //tower cube
			pros::delay(700);
			autonSetMotorValues ( 2, 12000 );
			armController -> setTarget (-2);//lower arm
			armController -> waitUntilSettled ();
			pros::delay(1000);
			chassis -> moveDistance (-0.5_ft);//clear tower
			chassis -> waitUntilSettled ();
			chassis -> turnAngle ( 25_deg ); //turn round tower
			chassis -> moveDistance (2.0_ft);//around tower
			chassis -> waitUntilSettled ();
			chassis -> turnAngle ( -15_deg ); //turn to 4 line
			chassis -> moveDistance (0.7_ft);//around tower
			chassis -> turnAngle ( 10_deg );//line up
			chassis -> moveDistance (1.8_ft);//get 10
			autonSetMotorValues ( 2, 1200 );
			chassis -> moveDistance(1.5_ft);
			chassis -> turnAngle ( 95_deg );//line up for tower2
			chassis -> waitUntilSettled ();
			chassis -> moveDistance(-0.8_ft);//back up for tower
			autonSetMotorValues ( 2, 12000 );
			pros::delay(2000);//overflow
			autonSetMotorValues ( 2, 1200 );
			chassis -> turnAngle ( 15_deg );//go for 3rd tower
			chassis -> moveDistance(1.3_ft);
			intakeMotors.moveRelative ( -170, -127 );//outtake for cube lock
			armController -> setTarget ( 90.0 ); //lift cube
			armController -> waitUntilSettled ();
			pros::delay(1500);
			intakeMotors.moveRelative ( -1300, -127 ); //tower cube
			pros::delay(700);
			autonSetMotorValues ( 2, 12000 );
			armController -> setTarget (-2);//lower arm
			armController -> waitUntilSettled ();
			pros::delay(1000);
			autonSetMotorValues ( 2, 12000 );
			chassis -> moveDistance(0.3_ft);//pick up possible drop out
			autonSetMotorValues ( 2, 1200 );
			chassis -> moveDistance(-1.0_ft);
			chassis -> turnAngle ( -50_deg );//line up to score
			chassis -> moveDistance(1.4_ft);//line up
			intakeMotors.moveRelative ( -400, -127 );
			trayController -> setTarget ( 90.0 ); //stack
			trayController -> waitUntilSettled ();
			pros::delay ( 2000 );
			chassis -> setMaxVelocity ( 20 );
			chassis -> moveDistance(0.15_ft);//push stack
			chassis -> setMaxVelocity ( 100 );
			intakeMotors.moveRelative ( -1000, -120 );
			pros::delay(300);
			chassis -> moveDistanceAsync ( -1_ft ); //back away
			chassis -> waitUntilSettled ();
			break;
		}
	}
}

void opcontrol ()
{
	pros::Task taskArm ( opArm, nullptr, "Drive" );
	pros::Task taskTray ( opTray, nullptr, "Tray" );
	pros::Task taskIntake ( opIntake, nullptr, "Intake" );

	chassis -> setMaxVelocity ( 500 );

	std::uint32_t timeValue = pros::millis ();

	while ( true )
	{
		chassis -> getModel () -> arcade ( master.getAnalog ( ControllerAnalog::rightY ), master.getAnalog ( ControllerAnalog::rightX ) );

		pros::Task::delay_until ( &timeValue, GLOBAL_DELAY );
	}
}

void autonSetMotorValues ( int type, int value )
{
  switch ( type )
  {
    case 0:
    {

    }
    case 1:
    {

    }
    case 2:
    {
      intakeMotors.moveVoltage ( value );
    }
  }
}

void opArm ( void * )
{
	double armTarget = 0.0;
	bool extended = false;

	while ( true )
	{
		switch ( ( 1 * master.getDigital ( ControllerDigital::L1 ) ) + ( 2 * master.getDigital(ControllerDigital::L2 ) ) + ( 4 * master.getDigital ( ControllerDigital::down ) ) )
		{
			case 1:
			{
				if ( extended == true )
				{
					armTarget = 0.0;

					passiveIntakeVoltage = 1000;

					activeIntakeVoltage = 12000;

					extended = false;

					while ( master.getDigital ( ControllerDigital::L1 ) == true )
					{
						pros::delay ( GLOBAL_DELAY );
					}

					break;
				}

				intakeLock = true;

				armTarget = 80.0;

				armController -> setTarget ( armTarget );

				pros::delay ( 100 );

				activeIntakeVoltage = 7500;

				passiveIntakeVoltage = 0;

				intakeMotors.moveRelative(-230,-127);

				pros::delay(1000);

				extended = true;

				intakeLock = false;

				while ( master.getDigital ( ControllerDigital::L1 ) == true )
				{
					pros::delay ( GLOBAL_DELAY );
				}

				break;
			}
			case 2:
			{
				intakeLock = true;

				armTarget = 110.0;

				armController -> setTarget ( armTarget );

				pros::delay ( 100 );

				passiveIntakeVoltage = 0;

				intakeMotors.moveRelative(-230,-127);

				pros::delay(1000);

				extended = true;

				while ( master.getDigital ( ControllerDigital::L2 ) == true )
				{
					pros::delay ( GLOBAL_DELAY );
				}

				intakeLock = false;

				break;
			}
			case 5:
			{
				armTarget += 1.0;

				activeIntakeVoltage = 12000;

				break;
			}
			case 6:
			{
				armTarget -= 1.0;

				activeIntakeVoltage = 12000;

				break;
			}
		}

		armController -> setTarget ( armTarget );

		pros::delay ( GLOBAL_DELAY );
	}
}

void opTray ( void * )
{
	double trayTarget = 0.0;
	bool extended = false;

	while ( true )
	{
		switch ( ( 1 * master.getDigital ( ControllerDigital::up ) ) + ( 2 * master.getDigital ( ControllerDigital::right ) ) + ( 4 * master.getDigital(ControllerDigital::left ) ) )
		{
			case 1:
			{
				if ( extended == true )
				{
					trayTarget = 0.0;

					extended = false;

					while ( master.getDigital ( ControllerDigital::up ) == true )
					{
						pros::delay ( GLOBAL_DELAY );
					}

					break;
				}

				trayTarget = 95.0;

				extended = true;

				while ( master.getDigital ( ControllerDigital::up ) == true )
				{
					pros::delay ( GLOBAL_DELAY );
				}

				break;
			}
			case 2:
			{
				trayTarget -= 0.25;

				while ( master.getDigital ( ControllerDigital::up ) == true )
				{
					pros::delay ( GLOBAL_DELAY );
				}

				break;
			}
			case 4:
			{
				trayTarget += 0.25;

				while ( master.getDigital ( ControllerDigital::up ) == true )
				{
					pros::delay ( GLOBAL_DELAY );
				}

				break;
			}
		}

		trayController -> setTarget ( trayTarget );

		pros::delay ( GLOBAL_DELAY );
	}
}

void opIntake ( void * )
{
	while ( true )
	{
		if ( intakeLock == true )
		{
			pros::delay ( GLOBAL_DELAY );

			continue;
		}

		switch ( ( 1 * master.getDigital ( ControllerDigital::R1 ) ) + ( 2 * master.getDigital ( ControllerDigital::R2 ) ) )
		{
			case 1:
			{
				intakeMotors.moveVoltage ( activeIntakeVoltage );

				break;
			}
			case 2:
			{
				intakeMotors.moveVoltage ( activeIntakeVoltage * -1 );

				break;
			}
			default:
			{
				intakeMotors.moveVoltage ( passiveIntakeVoltage );

				break;
			}
		}

		pros::delay ( GLOBAL_DELAY );
	}
}

void prepareDriveMotors ()
{
	backLeftMotor.moveVoltage ( 2200 );
	frontRightMotor.moveVoltage ( 2200 );

	backRightMotor.moveVoltage ( 2200 );
	frontRightMotor.moveVoltage ( 2200 );
}
