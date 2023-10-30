#include "pros/motors.h"
#include "pros/motors.hpp"
#include "RopoMath/Vector.hpp"
#ifndef _ROPOTHROWER_HPP_
#define _ROPOTHROWER_HPP_

namespace RopoThrower{

    const double ThrownPosition = 120.0;
    const double WaitingPosition = 210.0;
    const double HidingPosition = 0.0;
    const double ThrowerRatio = 1.0 / 18.0 / 3.0 ;
    const int FullSpeedVoltage = 12000;
    const int Deltatime = 3;

    typedef enum{
        HIDDEN = 0,      //when thrower is hidden in the robot
        WAITING = 1,    //when thrower have streched and waiting to shoot
        //THROWING = 2      //when thrower haven't been hiden
    }State;

    typedef enum{
        HIDE = 0,
        THROW = 1,
        WAIT = 2
    }AimState;

    class ThrowerModule{
        private:
        double ThrowerPosition;
        State ThrowerState;
        AimState ThrowerAimState;
        pros::MotorGroup *Motors; 

        public:
        static void ThrowerBackGroundFunction(void *Parameter);
        ThrowerModule(pros::MotorGroup *Mtrs);
        void Throw();
        void Hide();
        void Wait();
        State GetThrowerStatus();

    };
    ThrowerModule::ThrowerModule(pros::MotorGroup *Mtrs) {
        ThrowerState = HIDDEN;
        ThrowerAimState = HIDE;
        Motors = Mtrs;
        ThrowerPosition = 0;
        new pros::Task(ThrowerModule::ThrowerBackGroundFunction);
    }
    void ThrowerModule::ThrowerBackGroundFunction(void *Parameter) {
        if(Parameter = nullptr)return;
        ThrowerModule *This = static_cast<ThrowerModule *>(Parameter);
        This -> Motors -> set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
        This -> Motors -> tare_position();
        std::vector<double> PositionVector ;
        while(true) {
            PositionVector = This -> Motors -> get_positions() ;
            for(double i : PositionVector) This -> ThrowerPosition += i;
            This -> ThrowerPosition / 2.0 * ThrowerRatio;
            if(This -> ThrowerPosition >=  360)This -> ThrowerPosition -= 360;
            if(This -> ThrowerPosition <= -360)This -> ThrowerPosition += 360;

            switch(This -> ThrowerState){

                case HIDDEN:
                if(This -> ThrowerAimState == HIDE) This -> Motors -> move_voltage(0);
                else if(This -> ThrowerAimState == WAIT || This -> ThrowerAimState == THROW) {
                    if(This -> ThrowerPosition < WaitingPosition){
                        This -> Motors -> move_voltage(FullSpeedVoltage);
                    }
                    This -> ThrowerState = WAITING;
                }

                case WAITING:
                if(This -> ThrowerAimState == HIDE) {
                    if(This -> ThrowerPosition > HidingPosition){
                        This -> Motors -> move_velocity(-FullSpeedVoltage);
                    }
                    This -> ThrowerState = HIDDEN;
                } else if(This -> ThrowerAimState == WAIT) {
                    This -> Motors -> brake();
                } else if(This -> ThrowerAimState == THROW) {
                    This -> Motors -> move_velocity(FullSpeedVoltage);
                }
            }
            pros::delay(Deltatime);
        }
    }
    void ThrowerModule::Throw() { ThrowerAimState = THROW; }
    void ThrowerModule::Hide()  { ThrowerAimState = HIDE; }
    void ThrowerModule::Wait()  { ThrowerAimState = WAIT; }
    State ThrowerModule::GetThrowerStatus() { return ThrowerState;}
}
#endif //_ROPOTHROWER_HPP_