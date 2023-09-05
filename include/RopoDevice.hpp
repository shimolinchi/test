// Code : UTF - 8
#ifndef ROPO_DEVICE_HPP
#define ROPO_DEVICE_HPP

#include "pros/imu.hpp"
#include "pros/adi.hpp"
#include "pros/motors.hpp"
#include "RopoMath/Vector.hpp"
#include "RopoSensor/EncodingDisk.hpp"
#include "RopoSensor/OpenMv.hpp"
#include "RopoControl/Regulator.hpp"
#include "RopoApi.hpp"
#include "RopoChassis.hpp"


namespace RopoDevice{

	// Api
	typedef RopoApi::FloatType FloatType;

	namespace ThreeWire{
		const char LiftPneumaticPort  = 'C';
		pros::ADIDigitalOut LiftPneumatic(LiftPneumaticPort,false);

		const char ExternPneumaticPort  = 'A';
		pros::ADIDigitalOut ExternPneumatic(ExternPneumaticPort,false);
	}

	// Code 
	namespace Motors{

		const int LeftMotor1Port  	= 11;
		const int LeftMotor2Port  	= 12;
		const int LeftMotor3Port  	= 13;
		const int RightMotor1Port	= 17;
		const int RightMotor2Port	= 18;
		const int RightMotor3Port	= 19;
		

		const pros::motor_gearset_e_t ChassisGearset = pros::E_MOTOR_GEAR_BLUE;

		pros::Motor LeftMotor1 ( LeftMotor1Port , 	ChassisGearset, true );
		pros::Motor LeftMotor2 ( LeftMotor2Port , 	ChassisGearset, true );
		pros::Motor LeftMotor3 ( LeftMotor3Port , 	ChassisGearset, true );
		pros::Motor RightMotor1( RightMotor1Port,	ChassisGearset, false);
		pros::Motor RightMotor2( RightMotor2Port,	ChassisGearset, false);
		pros::Motor RightMotor3( RightMotor3Port,	ChassisGearset, false);


		const FloatType ChassisRatio = 2.0 / 2.0;

		void LeftWheelMove	(FloatType Velocity){
			LeftMotor1.move_velocity(Velocity * ChassisRatio);
			LeftMotor2.move_velocity(Velocity * ChassisRatio);
			LeftMotor3.move_velocity(Velocity * ChassisRatio);
		}

		void RightWheelMove (FloatType Velocity){
			RightMotor1.move_velocity(Velocity * ChassisRatio);
			RightMotor2.move_velocity(Velocity * ChassisRatio);
			RightMotor3.move_velocity(Velocity * ChassisRatio);
		}


	}

	namespace Sensors{

		const int InertialPort = 10;
		pros::IMU Inertial(InertialPort);

	}				

	FloatType GetHeading(){
		return -RopoDevice::Sensors::Inertial.get_yaw();
	}

	RopoMath::Vector<FloatType> GetPosition(){
		RopoMath::Vector<FloatType> PositionVector(RopoMath::ColumnVector,3);
		PositionVector[1] =  0 / 1000.0;
		PositionVector[2] =  0 / 1000.0;
		PositionVector[3] =  GetHeading();

		return PositionVector;
	}

	RopoChassis::TankChassis Chassis( Motors::RightWheelMove , Motors::LeftWheelMove , GetPosition , 12 );

	void DeviceInit(){
		RopoDevice::Chassis.SetVelocityLimits(600);
		Sensors::Inertial.reset(true);
		//while(Sensors::Inertial.is_calibrating())pros::delay(20);
		pros::delay(200);
	}

}

#endif
