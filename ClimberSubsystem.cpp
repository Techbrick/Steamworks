#include "ClimberSubsystem.h"

ClimberSubsystem::ClimberSubsystem(int climberChannel) :
	climber(climberChannel) //initialize climber
{
	//climber.SetControlMode(CANTalon::ControlMode::kSpeed); //speed control mode - RPM
	climber.ClearStickyFaults();
}

void ClimberSubsystem::enable() {
	climber.Enable();
}

void ClimberSubsystem::disable() {
	climber.Disable();
}

void ClimberSubsystem::setSpeed(float speed) {
	//climber.Set(speed * Constants::climberMaxSpeed);
	climber.Set(speed);
}
