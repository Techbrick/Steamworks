/*
 * Accumulator.cpp
 *
 *  Created on: Feb 7, 2017
 *      Author: FRC3941
 */

#include "Accumulator.h"

DoubleDouble Accumulator::drive(bool hasBeenGoing, bool encoderPos, bool powerPos, bool goingPos, float target, float targetAngle, float targetError, float robotAngle)
{


	float usAverage = (leftProx->GetRangeInches()+rightProx->GetRangeInches())/2;//This is the average ultrasonic value
	float visualDistance = aimer->GetDistanceToGear();//This is the distamce based on the cameras

	SmartDashboard::PutNumber("Joe visual distance", visualDistance );

	SmartDashboard::PutNumber("Joe us average", usAverage);
	float currentAngleError = targetAngle - robotAngle + lastCameraAngle;//The error of the new angle
	if (!hasBeenGoing)//If its the first time
	{
		encoderY->Reset();
		float intitialAngleError = targetAngle - robotAngle;
		reset(aimer->GetXDistanceToGear(usAverage, intitialAngleError), usAverage, robotAngle);//Resets the initial values in the accumulator
		SmartDashboard::PutNumber("Joe start y position", getCurrentPosition().y);
		lastLeftProxValue = leftProx->GetRangeInches();
		lastRightProxValue = rightProx->GetRangeInches();
		float cameraAngle = aimer->GetAngleToGear(usAverage);
		if(fabs(cameraAngle)< 30){
			lastCameraAngle = aimer->GetAngleToGear(usAverage);
		}else{

			lastCameraAngle = targetAngle;
		}
		SmartDashboard::PutNumber("Joe returned angle from aimer", lastCameraAngle);
		updateUS(leftProx->GetRangeInches(), 0);
		lastLeftProxValue = leftProx->GetRangeInches();
		float adjustedAngle = targetAngle - robotAngle + lastCameraAngle;
		float currentAngleError2 = aimer->ConvertToPlusMinus180(adjustedAngle);
		if (fabs(robotAngle - targetAngle) <= 2)
			updateX(robotAngle - targetAngle);
		lastCameraAngle = aimer->GetAngleToGear(history.back().y);
	}

	if(fabs(lastLeftProxValue - leftProx->GetRangeInches()) > 1){//Checks to be able to update the left ultrasonic value and y value
		SmartDashboard::PutString("Joe Status", "new left ultrasonic");
		updateUS(leftProx->GetRangeInches(), 0);
		lastLeftProxValue = leftProx->GetRangeInches();
	}

	if(fabs(lastRightProxValue - rightProx->GetRangeInches()) > 1){//Same thing for right
		SmartDashboard::PutString("Joe Status", "new right ultrasoinic");
		updateUS(rightProx->GetRangeInches(), 1);
		lastRightProxValue = rightProx->GetRangeInches();
	}

	if (lastCameraAngle != aimer->GetAngleToGear(history.back().y) && fabs(getCurrentPosition().y) >= 24)//Checks to update the X value
	{
		float gearAngle = aimer-> GetAngleToGear(history.back().y);
		if(fabs(gearAngle) < 30){
			SmartDashboard::PutNumber("Joe returned angle from aimer", gearAngle);
			currentAngleError = aimer->ConvertToZeroTo360(gearAngle - robotAngle);
			SmartDashboard::PutNumber("Joe angleError", currentAngleError);
			SmartDashboard::PutString("Joe Status", "new camera angle");
			updateX(robotAngle - targetAngle);
			lastCameraAngle = aimer->GetAngleToGear(history.back().y);
		}

	}
	currentAngleError = targetAngle - robotAngle + lastCameraAngle;
	SmartDashboard::PutNumber("Joe angleError", currentAngleError);
	SmartDashboard::PutNumber("joe test last camera angle", lastCameraAngle);
	SmartDashboard::PutNumber("joe test get angle to gear", aimer->GetAngleToGear(history.back().y));

	float robotAngleError = fabs(targetAngle - robotAngle);
	float driveX = 0.0;
	float driveY = 0.0;
	float driveZ = 0.0;
	float powerX = 0.0;
	float power = 0.0;
	if(true){//I dunno
		float basicXP = SmartDashboard::GetNumber("Joe XP", Constants::accumulatorXp);
		float tempXPower = -basicXP * (getCurrentPosition().x + SmartDashboard::GetNumber("Joe x offset", 0)); //This is the power calculator for the x movement TODO: JOE HAD THIS AS PUT NUMBER. FIND OUT WHY

			 //Fixing the power
			if (tempXPower > 1)
				powerX = 1.0;
			else if (tempXPower < Constants::minStrafePower && tempXPower > 0.05)
				powerX = Constants::minStrafePower;
			else if (tempXPower > -Constants::minStrafePower && tempXPower < 0.05)
				powerX = -Constants::minStrafePower;
			else if (tempXPower < -1)
				powerX = -1.0;
			else
				powerX = tempXPower;

			if (fabs(getCurrentPosition().x + SmartDashboard::GetNumber("Joe x offset", 0)) < 1 || fabs(getCurrentPosition().y) < 14) //TODO: JOE HAD THIS AS PUT NUMBER. FIND OUT WHY
				powerX = 0.0;

			float yscaleFactor = (fabs(getCurrentPosition().x + SmartDashboard::GetNumber("Joe x offset", 0) )<4)? 1: (fabs(getCurrentPosition().y )<= 30 && fabs(getCurrentPosition().y) >= 24)? 0 : .5; //This scales the y depending on how bad the x is

			float distance = encoderY->GetDistance();
			float lastDistanceTraveled = distance - lastEncoderDistance;
			float currentY = getCurrentPosition().y;
			float distanceError = currentY + (lastDistanceTraveled * ((goingPos) ? -1 : 1) * ((encoderPos) ? 1 : -1)) - target;//Calculates how far away from the wall the robot is
			float joeP = SmartDashboard::GetNumber("Joe P-value", (Constants::accumulatorPower));
			float tempPower = -fabs(joeP) * ((powerPos) ? 1 : -1) * distanceError * yscaleFactor;//Calculates the y power

			if (tempPower > 1)
				power = 1;
			else if (tempPower < Constants::minForwardPower && tempPower > 0)
				power = Constants::minForwardPower;
			else if (tempPower > -Constants::minForwardPower && tempPower < 0)
				power = -.13;
			else if (tempPower < -1)
				power = -1;
			else
				power = tempPower;

			SmartDashboard::PutNumber("Joe distance error", distanceError);
			SmartDashboard::PutNumber("Joe target error", targetError);
			if (fabs(distanceError) < targetError)
				power = 0;


			update(lastDistanceTraveled, power, powerX, robotAngle);//Update all of the powers with the accumulating
			//robotDrive.MecanumDrive_Cartesian(0,power,0);
			SmartDashboard::PutNumber("Joe Motor Power", power);
			SmartDashboard::PutNumber("Joe Distance", distance);
			SmartDashboard::PutNumber("Joe last Distance", lastDistanceTraveled);
			SmartDashboard::PutNumber("Joe Error", error);
			SmartDashboard::PutNumber("Joe X error", errorX);
			SmartDashboard::PutNumber("Joe current y position", currentY);
			SmartDashboard::PutNumber("Joe current x position", getCurrentPosition().x + SmartDashboard::GetNumber("Joe x offset", 0)); //TODO: JOE HAD THIS AS PUT NUMBER. FIND OUT WHY

	}
		SmartDashboard::PutBoolean("Joe in loop", true);

//	float gearAngle = aimer->TwoCameraAngleFilter() + robotAngle; //TODO: get angle via function (closest angle of the 3 options (0, 60, -60, find which is closest to current angle)?)
//	gearAngle = gearAngle < 360 ? gearAngle : gearAngle - 360; //scaling
//	gearAngle = gearAngle > 0 ? gearAngle : gearAngle + 360;
//	pid->resetPIDAngle(); //if loop is done reset values
	robotAngle = robotAngle < 0 ? 360 + robotAngle : robotAngle;
	float angleOutput = pid->PIDAngle(robotAngle, targetAngle);//Calculate the angle changle
	driveX = (fabs(robotAngle - targetAngle) < 2 && fabs(robotAngle - targetAngle - 360) < 2) ? powerX : 0.0; //TODO: JOE HAD THESE FLIPPED. FIND OUT WHY
	driveY = (fabs(robotAngle - targetAngle) < 2 && fabs(robotAngle - targetAngle - 360) < 2) ? power : 0.0; //TODO: JOE HAD THESE FLIPPED. FIND OUT WHY
	driveZ = angleOutput;
	lastEncoderDistance = encoderY->GetDistance();
	SmartDashboard::PutNumber("Joe robotAngle - targetAngle", fabs(robotAngle - targetAngle));
	SmartDashboard::PutNumber("Joe robotAngle - targetAngle - 360", fabs(robotAngle - targetAngle - 360));
	//SmartDashboard::PutString("Joe Status", "in try loop");
	//return DoubleDouble(0, 0, 0);
	return DoubleDouble(driveX, driveY, driveZ);
	//return DoubleDouble(driveX, 0, driveZ);

}


//PLEASE IGNORE THIS CRUD
DoubleDouble Accumulator::driveToTower(bool hasBeenGoing, bool encoderPos, bool powerPos, bool goingPos, float target, float targetAngle, float targetError, float robotAngle)
{

	if (!(aimer->GetBoilerAngle() > 500) && !(aimer->GetBoilerDistance() > 500))
	{
		float boilerAngle = aimer->GetBoilerAngle();
		float boilerDistance = aimer->GetBoilerDistance();
		if (!hasBeenGoing)
			{
				encoderY->Reset();
				encoderX->Reset();
				float intitialAngleError = targetAngle - robotAngle;
				reset(0, boilerDistance, boilerAngle);

				lastRightProxValue = rightProx->GetRangeInches();
				if(fabs(boilerAngle)< 30){
					lastCameraAngle = boilerAngle;
				}else{

					lastCameraAngle = targetAngle;
				}
				SmartDashboard::PutNumber("Joe returned angle from aimer", lastCameraAngle);
				//updateUS(leftProx->GetRangeInches(), 0);
				//lastLeftProxValue = leftProx->GetRangeInches();
				float adjustedAngle = targetAngle - robotAngle + lastCameraAngle;
				float currentAngleError2 = aimer->ConvertToPlusMinus180(adjustedAngle);
				updateX(robotAngle - targetAngle);
				lastCameraAngle = aimer->GetAngleToGear(history.back().y);
			}
		return DoubleDouble(0,0,0);

	}
	else
		return DoubleDouble(0,0,0);

}

void Accumulator::update(float encoder, float power, float powerX, float angle)//Update accumulating values
{
//	history.push(DoubleDouble(0,average + history.back().y + error,0));
//	prevPredicted = predict(power);

	//
	//
	float lastY = history.back().y;
	float lastDistancePower = Constants::yDistancePerSecond * power * Constants::teleopLoopTime;//Add the accumulating y to the current y
	SmartDashboard::PutNumber("Joe last power distance accum", lastDistancePower);
	float averageDistance = (encoder + lastDistancePower)/2;
	SmartDashboard::PutNumber("Joe average distance accum", averageDistance);
	float lastX = history.back().x;
	float xPowerDistance = Constants::xDistancePerSecond * powerX * Constants::teleopLoopTime;//Increments the accumulator for the x
	SmartDashboard::PutNumber("joe X predicted Entry", xPowerDistance + lastX);
	history.push_back(DoubleDouble(xPowerDistance + lastX, lastY + averageDistance, angle));
}

void Accumulator::updateUS(float ultrasonic, int id)//This basically compares the values back in time with the recently returned values to adjust the accumulator because the sensor lag is real
{
	if (ultrasonic < 120 && ultrasonic != 0)//Makes sure there are no bogus values
	{
		float lastY = history[(history.size() + lastUpdated[id]) / 2].y;
		error = ultrasonic - lastY;
		SmartDashboard::PutNumber("Joe last us y accum", lastY);
		//SmartDashboard::PutNumber("history size", history.size());
		int intermediateLastUpdate = lastUpdated[id];
		int historyLastIndex = history.size()-1;

				if(intermediateLastUpdate < historyLastIndex  && intermediateLastUpdate <= (history.size() -1)){
					float yValueInHistory = history[intermediateLastUpdate].y;
					float errorAtHistory = ultrasonic - yValueInHistory;
					SmartDashboard::PutNumber("joe y history error", errorAtHistory);
					//SmartDashboard::PutString("Joe updatedVis status", "updating");
					float lasty = history.back().y;
					for (int i = intermediateLastUpdate; i < history.size(); i++){
						float yPosit = history[i].y;
						history[i].y = yPosit + errorAtHistory;
					}
					float latestYValue = history.back().y;
					SmartDashboard::PutNumber("joe lasty in ydistance", latestYValue);
					//SmartDashboard::PutString("Joe updatedVis status", "updated History");
				}else if(history.size() == 1){
					history.back().y = ultrasonic;
					//SmartDashboard::PutNumber("joe lastx in xdistance", history.back().x);
				}


	//	}
		lastUpdated[id] = history.size() - 1;
	}
}

void Accumulator::updateX(float currentAngleError)//updates the x based on the camera angles
{

	float lastX = history[lastUpdated.back()].x;
	float dist = aimer->GetXDistanceToGear(history.back().y, currentAngleError);
	SmartDashboard::PutNumber("Joe y in xUpdate", history.back().y);
	SmartDashboard::PutNumber("Joe x update from last x", lastX);


	if(dist < 140)
	{
		SmartDashboard::PutNumber("Joe x distance from gear", dist);
		errorX = dist;// - lastX;
		lastUpdated.back() = history.size() - 1;
		int lastUpdatedIndex = lastUpdated.back();
		int historyLastIndex = history.size()-1;
		if(lastUpdatedIndex < historyLastIndex  && lastUpdatedIndex <= (history.size() -1)){
			float xValueInHistory = history[lastUpdatedIndex].x;
			float errorAtHistory = dist - xValueInHistory;
			SmartDashboard::PutNumber("joe x history error", errorAtHistory);
			SmartDashboard::PutString("Joe updatedVis status", "updating");
			float lastX = history.back().x;
			for (int i = lastUpdatedIndex; i < history.size(); i++){
				float xPosit = history[i].x;
				history[i].x = xPosit + errorAtHistory;
			}
			float latestXValue = history.back().x + SmartDashboard::GetNumber("Joe x offset", 0);
			SmartDashboard::PutNumber("joe lastx in xdistance", latestXValue);
			SmartDashboard::PutString("Joe updatedVis status", "updated History");
		}else if(history.size() == 1){
			history.back().x = dist;
			SmartDashboard::PutNumber("joe lastx in xdistance", history.back().x);
		}

	}
}
float Accumulator::GetXHistoryOffset(float targetAngle){
	float result = 0;

	return result;

}

void Accumulator::reset(float x, float y, float z)
{
	history.clear();
	prevPredicted = error = 0.0;
	history.push_back(DoubleDouble((x > 100) ? 0 : x,y,z));
}
