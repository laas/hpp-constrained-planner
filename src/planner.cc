// Copyright (C) 2011 by Sebastien Dalibard.
//
// This file is part of the hpp-constrained-planner.
//
// hpp-constrained-planner is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// hpp-constrained-planner is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with hpp-constrained-planner.  If not, see <http://www.gnu.org/licenses/>.

#include <jrl/mal/matrixabstractlayer.hh>

#include <hpp/gik/constraint/transformation-constraint.hh>
#include <hpp/gik/constraint/com-constraint.hh>

#include <hpp/model/humanoid-robot.hh>

#include <hpp/constrained/roadmap-builder.hh>
#include <hpp/constrained/config-projector.hh>
#include <hpp/constrained/kws-constraint.hh>

#include <hpp/constrained/planner.hh>



namespace hpp {
  namespace constrained {
    Planner::Planner():
      goalConfigGenerator_(NULL),
      configurationExtendor_(NULL)
    {
    }
    
    Planner::~Planner()
    {
    }

    void
    Planner::buildDoubleSupportStaticStabilityConstraints(CkwsConfigShPtr i_config,
							  std::vector<CjrlGikStateConstraint*> & o_soc)
    {
     hpp::model::HumanoidRobotShPtr robot = 
	KIT_DYNAMIC_PTR_CAST(hpp::model::HumanoidRobot,i_config->device());

      if (!robot) {
	return;
      }

      robot->hppSetCurrentConfig(*i_config);
 
      CjrlJoint * rightAnkle = robot->rightAnkle();
      matrix4d rightAnkleT = rightAnkle->currentTransformation();
  
      CjrlJoint * leftAnkle = robot->leftAnkle();
      matrix4d leftAnkleT = leftAnkle->currentTransformation();

      ChppGikTransformationConstraint * rightAnkleConstraint =
	new ChppGikTransformationConstraint(*robot,*rightAnkle,vector3d(0,0,0),rightAnkleT);

      ChppGikTransformationConstraint * leftAnkleConstraint =
	new ChppGikTransformationConstraint(*robot,*leftAnkle,vector3d(0,0,0),leftAnkleT);
  
      vector3d comPos = robot->positionCenterOfMass();
      ChppGikComConstraint * comConstraint = 
	new ChppGikComConstraint(*robot,comPos[0],comPos[1]);

      o_soc.clear();
      o_soc.push_back(rightAnkleConstraint);
      o_soc.push_back(leftAnkleConstraint);
      o_soc.push_back(comConstraint);
    }

    void
    Planner::buildSingleSupportStaticStabilityConstraints(CkwsConfigShPtr i_config,
							  bool rightFootSupporting,
							  std::vector<CjrlGikStateConstraint*> & o_soc)
    {
      hpp::model::HumanoidRobotShPtr robot = 
	KIT_DYNAMIC_PTR_CAST(hpp::model::HumanoidRobot,i_config->device());

      if (!robot) {
	return;
      }

      robot->hppSetCurrentConfig(*i_config);

      CjrlJoint * ankle = rightFootSupporting ? robot->rightAnkle() : robot->leftAnkle();
      matrix4d ankleT = ankle->currentTransformation();

      ChppGikTransformationConstraint * ankleConstraint =
	new ChppGikTransformationConstraint(*robot,*ankle,vector3d(0,0,0),ankleT);

      vector3d comPos = robot->positionCenterOfMass();
      ChppGikComConstraint * comConstraint = 
	new ChppGikComConstraint(*robot,comPos[0],comPos[1]);

      o_soc.clear();
      o_soc.push_back(ankleConstraint);
      o_soc.push_back(comConstraint);    
    }

    ktStatus
    Planner::initializeProblem()
    {
      if(!goalConfigGenerator_) {
	return KD_ERROR;
      }

      if(!configurationExtendor_){
	return KD_ERROR;
      }
      
      hpp::model::HumanoidRobotShPtr robot =  
	KIT_DYNAMIC_PTR_CAST(hpp::model::HumanoidRobot, robotIthProblem (0));

      if (!robot) {
	return KD_ERROR;
      }

      std::string property,value;
      property="ComputeZMP"; value="false";robot->setProperty ( property,value );
      property="TimeStep"; value="0.005";robot->setProperty ( property,value );
      property="ComputeAccelerationCoM"; value="false";robot->setProperty ( property,value );
      property="ComputeBackwardDynamics"; value="false";robot->setProperty ( property,value );
      property="ComputeMomentum"; value="false";robot->setProperty ( property,value );
      property="ComputeAcceleration"; value="false";robot->setProperty ( property,value );
      property="ComputeVelocity"; value="false";robot->setProperty ( property,value );
      property="ComputeSkewCom"; value="false";robot->setProperty ( property,value );
      property="ComputeCoM"; value="true";robot->setProperty ( property,value );

      CkwsRoadmapShPtr roadmap = CkwsRoadmap::create(robot);
      CkwsDiffusingRdmBuilderShPtr rdmBuilder = DiffusingRoadmapBuilder::create(roadmap,configurationExtendor_);
  
      rdmBuilder->diffuseFromProblemStart(true);
      rdmBuilder->diffuseFromProblemGoal(true);

      roadmapBuilderIthProblem ( 0, rdmBuilder);

      steeringMethodIthProblem(0, CkwsSMLinear::create ());

      CkwsConfigShPtr initConfig;
      robot->getCurrentConfig(initConfig);
      initConfIthProblem(0,initConfig);

      if (generateGoalConfigurations(0,2) != KD_OK) {
	return KD_ERROR;
      }

      KwsConstraintShPtr constraint = KwsConstraint::create("Whole-Body Constraint", configurationExtendor_);
      robot->userConstraints()->add(constraint);

      CkwsLoopOptimizerShPtr optimizer = 
	CkwsRandomOptimizer::create();
      optimizer->penetration (hppProblem (0)->penetration ());
      
      pathOptimizerIthProblem(0,optimizer);
      
      hppProblem (0)->alwaysOptimize (true);

      return KD_OK;
    }

    ktStatus
    Planner::generateGoalConfigurations(unsigned int rank, unsigned int nb_configs)
    {
      if (!goalConfigGenerator_) {
	return KD_ERROR;
      }
      CkppDeviceComponentShPtr robot = robotIthProblem(rank);
      CkwsRoadmapBuilderShPtr rdmBuilder = roadmapBuilderIthProblem(rank);

      unsigned int nb_validConfigs=0;
      unsigned int nb_try=0;
      unsigned int nb_maxTry=10000;
      while ( (nb_try < nb_maxTry) 
	      && (nb_validConfigs < nb_configs) ) { //Shoot config

	CkwsConfigShPtr randomConfig = CkwsConfig::create (robot);
	randomConfig->randomize();

	if (goalConfigGenerator_->project(*randomConfig) == KD_OK) { //Projection worked
	  robot->setCurrentConfig(*randomConfig);

	  if(!robot->collisionTest()) { //Configuration is collision free

	    CkwsNodeShPtr newNode(rdmBuilder->roadmapNode(*randomConfig));
	    goalConfIthProblem(rank,randomConfig);
	    if (rdmBuilder->addGoalNode(newNode) == KD_OK) {
	      nb_validConfigs++;
	    }
	  }
	}

	nb_try++;	
      }
      if (nb_validConfigs < nb_configs) {
	return KD_ERROR;
      }
      return KD_OK;	
    }

    void
    Planner::setGoalConfigGenerator(ConfigProjector * i_goalConfigGenerator)
    {
      goalConfigGenerator_ = i_goalConfigGenerator;
    }

    ConfigProjector *
    Planner::getGoalConfigGenerator()
    {
      return goalConfigGenerator_;
    }

    void
    Planner::setConfigurationExtendor(ConfigExtendor * i_configExtendor)
    {
      configurationExtendor_ = i_configExtendor;
    }

    ConfigExtendor *
    Planner::getConfigurationExtendor()
    {
      return configurationExtendor_;
    }

  } //end of namespace constrained
} //end of namespace hpp
