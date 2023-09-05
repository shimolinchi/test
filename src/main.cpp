#include "main.h"

void initialize() {
	pros::lcd::initialize();
	RopoDevice::DeviceInit();
}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {
	pros::Controller MasterController(pros::E_CONTROLLER_MASTER);
	RopoController::ButtonTaskLine ButtonDetectLine(MasterController);
	RopoApi::FloatType VelocityMax = 6;
	RopoApi::FloatType RopoWcLimit = 10;
	bool ChassisMove = false;
	RopoController::AxisValueCast XVelocityInput(MasterController,pros::E_CONTROLLER_ANALOG_LEFT_Y,RopoController::Exp);
	RopoController::AxisValueCast WVelocityInput(MasterController,pros::E_CONTROLLER_ANALOG_RIGHT_X,RopoController::Linear);
	RopoMath::Matrix<RopoApi::FloatType> Parameter(3,3);
	RopoMath::Vector<RopoApi::FloatType> Velocity(RopoMath::ColumnVector,3),ResVelocity;
	MasterController.clear();
	while (true) {
		RopoApi::FloatType XInput =  XVelocityInput.GetAxisValue();
		RopoApi::FloatType WInput = -WVelocityInput.GetAxisValue();
		if(fabs(XInput) <= 0.05 && fabs(WInput) <= 0.05 ){
			Velocity[1] = Velocity[2] = 0;
			if(ChassisMove)
				RopoDevice::Chassis.MoveVelocity(Velocity);
			ChassisMove = false;
		}
		else{
			Velocity[1] = XInput * VelocityMax;
			Velocity[2] = WInput * RopoWcLimit;
			RopoDevice::Chassis.MoveVelocity(Velocity);
			ChassisMove = true;
		}
		pros::delay(20);
	}
}
