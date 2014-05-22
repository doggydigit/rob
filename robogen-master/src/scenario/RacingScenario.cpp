/*
 * @(#) RacingScenario.cpp   1.0   Mar 13, 2013
 *
 * Andrea Maesani (andrea.maesani@epfl.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2012-2013 Andrea Maesani
 *
 * Laboratory of Intelligent Systems, EPFL
 *
 * This file is part of the ROBOGEN Framework.
 *
 * The ROBOGEN Framework is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License (GPL)
 * as published by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * @(#) $Id$
 */
#include "config/RobogenConfig.h"
#include "config/StartPositionConfig.h"
#include "scenario/Environment.h"
#include "scenario/RacingScenario.h"
#include "Robot.h"
#include "Models.h"

namespace robogen {

RacingScenario::RacingScenario(boost::shared_ptr<RobogenConfig> robogenConfig) :
		Scenario(robogenConfig), curTrial_(0) {

	this->setEnvironment(boost::shared_ptr<Environment>(new Environment()));

}

RacingScenario::~RacingScenario() {

}

bool RacingScenario::setupSimulation() {
	
	// Compute robot start position,
	startPosition_.push_back(this->getCurrentStartPosition()->getPosition());
	fitness = 0;
	sensors = this->getRobot()->getSensors();
	Cam = this->getRobot()->getCam();
	Pos = osg::Vec2(Cam->getRootPosition().x(),Cam->getRootPosition().y());
	motors = this->getRobot()->getMotor();
	return true;

}

bool RacingScenario::afterSimulationStep() {

	//std::cout<<"fitness1 "<<fitness<<std::endl;
	//fitness += 300000000*((Cam->getRootPosition().x() - Pos[0])*(Cam->getRootPosition().x() - Pos[0])+(Cam->getRootPosition().y() - Pos[1])*(Cam->getRootPosition().y() - Pos[1]));
	//Pos = osg::Vec2(Cam->getRootPosition().x(),Cam->getRootPosition().y());
	//std::cout<<"fitness2 "<<fitness<<std::endl;
	//std::cout<<Cam->getRootPosition().x()<<std::endl;
	fitness -= motors[1]->getSpeed();
	fitness += motors[0]->getSpeed();
	//fitness += motors[2]->getSpeed();
	//std::cout<<"fitness3 "<<fitness<<std::endl;

	//for(unsigned int i = 1; i<motors.size(); ++i)
	//{
		//fitness += motors[i-1]->getSpeed();
	//}
	//for(unsigned int i = 0; i < sensors.size();++i)
	//{
	//	if(!sensors[i]->getCamera())
	//	{
	//		if(sensors[i]->getActivity())
	//		{
				//std::cout<<"BOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOM"<<fitness<< "----";
	//			fitness-=1000000;
				//std::cout<<fitness<<"_________"<<std::endl;
	//		}
	//	}
	//}

	return true;
}

bool RacingScenario::init(dWorldID odeWorld, dSpaceID odeSpace, boost::shared_ptr<Robot> robot) {

	Scenario::init(odeWorld, odeSpace, robot);

	boost::shared_ptr<Environment> env(new Environment());
	env->setAmbientLight(10);
	std::vector<boost::shared_ptr<LightSource> > lightSources;
	lightSources.push_back(boost::shared_ptr<LightSource>(
			new LightSource(odeSpace, osg::Vec3(-1.3, 0,
					this->getRobogenConfig()->getLightSourceHeight()), 100)));
	env->setLightSources(lightSources);

	this->setEnvironment(env);
	return true;
}

bool RacingScenario::endSimulation() {
	
	//fitness += 300000000*this->getRobot()->getCoreComponent()->getPosition()[0];
	fitness -= 3000000000*Cam->getRootPosition().x();	
	//newly added for modified racing scenario
	curTrial_++;
	return true;
	
	
	
	// Compute robot ending position from its closest part to the origin
	double minDistance = std::numeric_limits<double>::max();
	const std::vector<boost::shared_ptr<Model> >& bodyParts = this->getRobot()->getBodyParts();
	for (unsigned int i = 0; i < bodyParts.size(); ++i) {
		osg::Vec2 curBodyPos = osg::Vec2(bodyParts[i]->getRootPosition().x(), bodyParts[i]->getRootPosition().y());
		osg::Vec2 curDistance = startPosition_[startPosition_.size()-1] - curBodyPos;
		if (curDistance.length() < minDistance) {
			minDistance = curDistance.length();
		}
	}

	distances_.push_back(minDistance);
	curTrial_++;
	// Set next starting position
	this->setStartingPosition(curTrial_);
	return true;

}

double RacingScenario::getFitness() {
	return fitness;
}

bool RacingScenario::remainingTrials() {
	boost::shared_ptr<StartPositionConfig> startPos = this->getRobogenConfig()->getStartingPos();
	return curTrial_ < startPos->getStartPosition().size();
}

int RacingScenario::getCurTrial() const {
	return curTrial_;
}

}
