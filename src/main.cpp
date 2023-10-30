#include "main.h"
#include "RopoDevice.hpp"
#include "RopoPosition.hpp"
#include "pros/rtos.hpp"

 namespace ControlFunction{
	void Throw(){
		RopoDevice::Thrower.Throw();
	}
	void ChangeThrowerStatus(){
		if(RopoDevice::Thrower.GetThrowerStatus() == RopoThrower::HIDE)RopoDevice::Thrower.Wait();
		else if(RopoDevice::Thrower.GetThrowerStatus() == RopoThrower::WAIT)RopoDevice::Thrower.Hide();
	}
} 

void initialize() {
	pros::lcd::initialize();
	RopoDevice::DeviceInit();
	RopoPosition::StartPosition();
	
}
void disabled() {}

void competition_initialize() {}

void autonomous() {
	// RopoDevice::Chassis.MoveVelocity(2,0);
	// pros::delay(1000);
	// RopoDevice::Chassis.MoveVelocity(0,0);
}

void opcontrol() {
	pros::Controller MasterController(pros::E_CONTROLLER_MASTER);
	RopoController::ButtonTaskLine ButtonDetectLine(MasterController);
	RopoApi::FloatType VelocityMax = 2;
	RopoApi::FloatType RopoWcLimit = 4;
	bool ChassisMove = false;
	RopoController::AxisValueCast XVelocityInput(MasterController,pros::E_CONTROLLER_ANALOG_LEFT_Y,RopoController::Exp);
	RopoController::AxisValueCast WVelocityInput(MasterController,pros::E_CONTROLLER_ANALOG_RIGHT_X,RopoController::Linear);
	RopoMath::Vector<RopoApi::FloatType> Velocity(RopoMath::ColumnVector,2),ResVelocity;
	MasterController.clear();
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_DOWN  , RopoController::Rising,  autonomous);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_R1, RopoController::Pressing, ControlFunction::Throw);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_R2, RopoController::Falling, ControlFunction::ChangeThrowerStatus);
	ButtonDetectLine.Enable();

	while (true) {
		RopoWcLimit = 3;
		RopoApi::FloatType XInput =  XVelocityInput.GetAxisValue();
		RopoApi::FloatType WInput = -WVelocityInput.GetAxisValue();
		if(fabs(XInput) <= 0.04 && fabs(WInput) <= 0.04 ){
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
		pros::lcd::print(1,"Ready!!! V:%.1f %.1f",XInput,WInput);
		pros::delay(4);
		MasterController.print(0,1,"V!!! %.1f %.1f",XInput,WInput);
		pros::delay(4);
		MasterController.print(1,1,"P!!!! %.1lf %.1lf",RopoPosition::Get_X()*1000.0,RopoPosition::Get_Y() *1000.0);
		pros::delay(4); 
		MasterController.print(2,1,"P!!!! %.1lf",-RopoDevice::Sensors::Inertial.get_yaw());
		pros::delay(4); 
		
	}
}
