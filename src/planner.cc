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

#include <iostream>
#include <ostream>
#include <fstream>

#include <KineoWorks2/kwsShooterConfigSpace.h>
#include <KineoModel/kppConfigComponent.h>
#include <KineoModel/kppSMLinearComponent.h>
#include <KineoWorks2/kwsMultiplePlanner.h>

#include <jrl/mal/matrixabstractlayer.hh>

#include <hpp/gik/constraint/transformation-constraint.hh>
#include <hpp/gik/constraint/com-constraint.hh>
#include <hpp/gik/constraint/relative-transformation-constraint.hh>
#include <hpp/gik/constraint/relative-com-constraint.hh>
#include <hpp/gik/constraint/plane-constraint.hh>
#include <hpp/gik/constraint/parallel-constraint.hh>

#include <hpp/kwsio/configuration.hh>
#include <hpp/util/debug.hh>
#include <hpp/util/exception.hh>
#include <hpp/model/humanoid-robot.hh>
#include <hpp/model/joint.hh>

#include <hpp/constrained/roadmap-builder.hh>
#include <hpp/constrained/config-projector.hh>
#include <hpp/constrained/goal-config-generator.hh>
#include <hpp/constrained/kws-constraint.hh>
#include <hpp/constrained/config-optimizer.hh>

#include "hpp/constrained/planner/planner.hh"
// #include "../src/roboptim/path-optimizer.hh"



namespace hpp {
  namespace constrained {
    Planner::Planner():
      goalConfigGenerators_ (),
      configurationExtendor_ (NULL)
    {
      configurationShooter_ = CkwsShooterConfigSpace::create();
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
      robot->computeForwardKinematics();
      robot->computeForwardKinematics();

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

      CjrlFoot* foot = rightFootSupporting ? robot->rightFoot() : robot->leftFoot();
      vector3d anklePInFoot;
      foot->getAnklePositionInLocalFrame (anklePInFoot);

      matrix4d ankleTInFoot;
      ankleTInFoot(0,0) = 1;
      ankleTInFoot(1,1) = 1;
      ankleTInFoot(2,2) = 1;
      ankleTInFoot(3,3) = 1;
      ankleTInFoot(0,3) = anklePInFoot[0];
      ankleTInFoot(1,3) = anklePInFoot[1];
      ankleTInFoot(2,3) = anklePInFoot[2];

      matrix4d footTInAnkle;
      MAL_S4x4_INVERSE (ankleTInFoot, footTInAnkle, double);

      matrix4d footT;
      MAL_S4x4_C_eq_A_by_B (footT,
			    ankleT,
			    footTInAnkle);

      ChppGikComConstraint * comConstraint =
	new ChppGikComConstraint(*robot, footT(0,3), footT(1,3));

      o_soc.clear();
      o_soc.push_back(ankleConstraint);
      o_soc.push_back(comConstraint);
    }

    void
    Planner::buildDoubleSupportSlidingStaticStabilityConstraints(CkwsConfigShPtr i_config,
								 std::vector<CjrlGikStateConstraint*> & o_soc)
    {
     hpp::model::HumanoidRobotShPtr robot =
	KIT_DYNAMIC_PTR_CAST(hpp::model::HumanoidRobot,i_config->device());

      if (!robot) {
	return;
      }

      robot->hppSetCurrentConfig(*i_config);
      robot->computeForwardKinematics();
      robot->computeForwardKinematics();

      CjrlJoint * rightAnkle = robot->rightAnkle();
      matrix4d rightAnkleT = rightAnkle->currentTransformation();
      hppDout (info, "rightAnkleT = " << rightAnkleT);
      CjrlJoint * leftAnkle = robot->leftAnkle();
      matrix4d leftAnkleT = leftAnkle->currentTransformation();
      hppDout (info, "leftAnkleT = " << leftAnkleT);

      matrix4d inverseRightAnkleT,relativeLeftAnkleT;
      MAL_S4x4_INVERSE (rightAnkleT,inverseRightAnkleT,double);
      MAL_S4x4_C_eq_A_by_B (relativeLeftAnkleT,
			     inverseRightAnkleT,
			     leftAnkleT);

      double rightAnkleHeight = MAL_S4x4_MATRIX_ACCESS_I_J(rightAnkleT,2,3);

      ChppGikPlaneConstraint * rightAnklePlaneConstraint =
	new ChppGikPlaneConstraint(*robot,*rightAnkle,vector3d(0,0,0),vector3d(0,0,rightAnkleHeight),vector3d(0,0,1));

      ChppGikParallelConstraint * rightAnkleParallelConstraint =
	new ChppGikParallelConstraint(*robot,*rightAnkle,vector3d(0,0,1),vector3d(0,0,1));

      ChppGikRelativeTransformationConstraint * leftAnkleConstraint =
	new ChppGikRelativeTransformationConstraint(*robot,*leftAnkle,rightAnkle,relativeLeftAnkleT);

      vector3d com = robot->positionCenterOfMass();
      vector4d comH(com[0],com[1],com[2],1);
      vector4d relativeComPos;
      MAL_S4x4_C_eq_A_by_B (relativeComPos,
			     inverseRightAnkleT,
			     comH);

      ChppGikRelativeComConstraint * comConstraint =
	new ChppGikRelativeComConstraint(*robot,rightAnkle,relativeComPos[0],relativeComPos[1]);

      o_soc.clear();
      o_soc.push_back(rightAnklePlaneConstraint);
      o_soc.push_back(rightAnkleParallelConstraint);
      o_soc.push_back(leftAnkleConstraint);
      o_soc.push_back(comConstraint);
    }


    ktStatus Planner::addHppProblem(CkppDeviceComponentShPtr robot,
				    double penetration)
    {
      model::DeviceShPtr hppRobot = KIT_DYNAMIC_PTR_CAST (model::Device, robot);
      assert (hppRobot);
      if (hpp::core::Planner::addHppProblem (robot, penetration) != KD_OK) {
	return KD_ERROR;
      }
      goalConfigGenerators_.push_back (GoalConfigGenerator::create (hppRobot));
      return KD_OK;
    }

    ktStatus Planner::removeHppProblem()
    {
      if (hpp::core::Planner::removeHppProblem () != KD_OK) {
	return KD_ERROR;
      }
      goalConfigGenerators_.pop_back ();
      return KD_OK;
    }

    ktStatus Planner::addHppProblemAtBeginning(CkppDeviceComponentShPtr robot,
					       double penetration)
    {
      model::DeviceShPtr hppRobot = KIT_DYNAMIC_PTR_CAST (model::Device, robot);
      assert (hppRobot);

      if (hpp::core::Planner::addHppProblemAtBeginning (robot, penetration)
	  != KD_OK) {
	return KD_ERROR;
      }
      goalConfigGenerators_.push_front (GoalConfigGenerator::create (hppRobot));
      return KD_OK;
    }

    ktStatus Planner::removeHppProblemAtBeginning()
    {
      if (hpp::core::Planner::removeHppProblemAtBeginning () != KD_OK) {
	return KD_ERROR;
      }
      goalConfigGenerators_.pop_front ();
      return KD_OK;
    }

    ktStatus Planner::setDynamicProperties (const unsigned int rank,
					    const bool isDynamic)
    {
      CkppDeviceComponentShPtr kppDevice = robotIthProblem (rank);
      if (!kppDevice)
	{
	  hppDout (error, "Null pointer to robot in problem " << rank);
	  return KD_ERROR;
	}

      hpp::model::DeviceShPtr robot
	= KIT_DYNAMIC_PTR_CAST(hpp::model::Device, robotIthProblem (rank));
      if (!robot)
	{
	  hppDout (error, "Robot in problem " << rank
		   << " is not a hpp robot");
	  return KD_ERROR;
	}

      std::string property,value;
      property="TimeStep"; value="0.005";
      robot->setProperty (property,value);
      property="ComputeCoM"; value="true";
      robot->setProperty (property,value);

      if (isDynamic)
	{
	  property="ComputeAccelerationCoM"; value="true";
	  robot->setProperty (property,value);
	  property="ComputeBackwardDynamics"; value="true";
	  robot->setProperty (property,value);
	  property="ComputeMomentum"; value="true";
	  robot->setProperty (property,value);
	  property="ComputeAcceleration"; value="true";
	  robot->setProperty (property,value);
	  property="ComputeVelocity"; value="true";
	  robot->setProperty (property,value);
	  property="ComputeSkewCom"; value="true";
	  robot->setProperty (property,value);
	}
      else
	{
	  property="ComputeAccelerationCoM"; value="false";
	  robot->setProperty (property,value);
	  property="ComputeBackwardDynamics"; value="false";
	  robot->setProperty (property,value);
	  property="ComputeMomentum"; value="false";
	  robot->setProperty (property,value);
	  property="ComputeAcceleration"; value="false";
	  robot->setProperty (property,value);
	  property="ComputeVelocity"; value="false";
	  robot->setProperty (property,value);
	  property="ComputeSkewCom"; value="false";
	  robot->setProperty (property,value);
	}

      // Set ZMP computation property if robot is a humanoid robot.
      if (KIT_DYNAMIC_PTR_CAST(hpp::model::HumanoidRobot,
			       robotIthProblem (rank)))
	if (isDynamic)
	  {
	    property="ComputeZMP"; value="true";
	    robot->setProperty (property,value);
	  }
	else
	  {
	    property="ComputeZMP"; value="false";
	    robot->setProperty (property,value);
	  }
      else
	hppDout (notice, "robot at probem " << rank
		 << " is not a humanoid robot.");

      return KD_OK;
    }

    ktStatus
    Planner::initializeProblem()
    {
      std::cout << "Planner::InitializeProblem..." << std::endl;

      if(!configurationExtendor_){
	std::cerr << "No configuration extendor" << std::endl;
	return KD_ERROR;
      }

      hpp::model::HumanoidRobotShPtr robot =
	KIT_DYNAMIC_PTR_CAST(hpp::model::HumanoidRobot, robotIthProblem (0));

      if (!robot) {
	std::cerr << "Expecting a HumanoidRobot" << std::endl;
	return KD_ERROR;
      }

      // Deactivate dynamic quantities computation during geometric
      // planning.
      if (KD_OK != setDynamicProperties (0, false))
	{
	  hppDout (error, "Could not set dynamic properties for robot.");
	  return KD_ERROR;
	}

      CkwsRoadmapShPtr roadmap = CkwsRoadmap::create(robot);
      CkwsDiffusingRdmBuilderShPtr rdmBuilder =
	DiffusingRoadmapBuilder::create(roadmap,configurationExtendor_);

      CkwsDiffusionShooterShPtr shooter = CkwsShooterConfigSpace::create();
      rdmBuilder->diffusionShooter(shooter);

      rdmBuilder->diffuseFromProblemStart(true);
      rdmBuilder->diffuseFromProblemGoal(true);

      roadmapBuilderIthProblem (0, rdmBuilder);
      steeringMethodIthProblem(0, CkppSMLinearComponent::create ());

      KwsConstraintShPtr constraint = KwsConstraint::create
	("Whole-Body Constraint", configurationExtendor_);
      robot->userConstraints()->add(constraint);

      CkwsConfigShPtr initConfig;
      robot->getCurrentConfig(initConfig);
      initConfIthProblem(0,initConfig);
      robot->addConfigComponent(CkppConfigComponent::create
				(initConfig,std::string("Init config")));

      CkwsLoopOptimizerShPtr randomOptimizer = CkwsRandomOptimizer::create();
      randomOptimizer->penetration (hppProblem (0)->penetration ());

      // Initialize goal configuration optimizer
      CkwsConfigShPtr halfSittingCfg;
      robot->getCurrentConfig(halfSittingCfg);
      std::vector<CkwsPathPlannerShPtr> optVector;
      if (goalExtendor_) {
	ConfigOptimizerShPtr goalOptimizer =
	  ConfigOptimizer::create (robot, goalExtendor_, halfSittingCfg);
	optVector.push_back (goalOptimizer);
      }
      optVector.push_back (randomOptimizer);
      CkwsMultiplePlannerShPtr combinedOptimizer =
	CkwsMultiplePlanner::create (optVector);
      pathOptimizerIthProblem(0, combinedOptimizer);

      hppProblem (0)->alwaysOptimize (true);

      return KD_OK;
    }

    ktStatus
    Planner::generateGoalConfigurations(unsigned int rank,
					unsigned int nb_configs)
    {
      GoalConfigGeneratorShPtr goalConfigGenerator =
	goalConfigGenerators_ [rank];
      if (!goalConfigGenerator) {
	hppDout (error, "No goal config generator defined.");
	return KD_ERROR;
      }
      CkppDeviceComponentShPtr robot = robotIthProblem(rank);
      CkwsRoadmapBuilderShPtr rdmBuilder = roadmapBuilderIthProblem(rank);

      CkwsConfigShPtr currentCfg;
      robot->getCurrentConfig(currentCfg);

      unsigned int nb_validConfigs=0;
      unsigned int nb_try=0;
      unsigned int nb_maxTry=10;
      while (nb_try < nb_maxTry && nb_validConfigs < nb_configs) {
	CkwsConfigShPtr randomConfig = CkwsConfig::create(*currentCfg);
	if (goalConfigGenerator->generate (*randomConfig)) {
	  CkwsNodeShPtr newNode(rdmBuilder->roadmapNode(*randomConfig));
	  addGoalConfIthProblem(rank,randomConfig);
	  if (rdmBuilder->addGoalNode(newNode) == KD_OK) {
	    nb_validConfigs++;
	  } else {
	    nb_try++;
	    hppDout (info,
		     "but could not be added as goal config to the roadmap.");
	  }
	} else {
	  nb_try++;
	}
      }
      if (nb_validConfigs < nb_configs) {
	hppDout (error, "Failed to generate " << nb_configs
		 << " goal configurations, generated only "
		 << nb_validConfigs);
	return KD_ERROR;
      }
      return KD_OK;
    }

    void Planner::
    goalConfigGenerator(unsigned int rank,
			   GoalConfigGeneratorShPtr goalConfigGenerator)
    {
      goalConfigGenerators_ [rank] = goalConfigGenerator;
    }

    GoalConfigGeneratorShPtr Planner::
    goalConfigGenerator (unsigned int rank) const
    {
      return goalConfigGenerators_ [rank];
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

    static const double TIME_STEP = 0.005;
    ktStatus Planner::writeSeqPlayFile (unsigned int rank, unsigned int pathId,
					const std::string& prefix)
    {
      std::ofstream file_pos, file_zmp, file_rpy;
      file_pos.exceptions (std::ofstream::failbit | std::ofstream::badbit);
      file_zmp.exceptions (std::ofstream::failbit | std::ofstream::badbit);
      file_rpy.exceptions (std::ofstream::failbit | std::ofstream::badbit);
      std::string filename;
      try {
	filename = prefix + std::string (".pos");
	file_pos.open (filename.c_str ());
	filename = prefix + std::string (".zmp");
	file_zmp.open (filename.c_str ());
	filename = prefix + std::string (".hip");
	file_rpy.open (filename.c_str ());
      } catch (const std::ofstream::failure& exc) {
	hppDout (error, "Failed to open file " + filename);
	return KD_ERROR;
      }
      CkwsPathShPtr path (getPath (rank, pathId));
      double L = path->length ();
      double T = 12.*L;
      // Cubic parameterization
      double p0 = 0;
      double p1 = 0;
      double p2 = 3*L/(T*T);
      double p3 = (-2.*L)/(T*T*T);

       model::HumanoidRobotShPtr robot =
	 KIT_DYNAMIC_PTR_CAST (model::HumanoidRobot, robotIthProblem (rank));

      for (double t=0; t<=T; t+=TIME_STEP) {
	CkwsConfig kwsConfig (robot);
	//double l = t*t*(p2 + t*p3);
	double l = L*t/T;
	if (path->getConfigAtDistance (l, kwsConfig) != KD_OK) {
	  hppDout (error, "failed to get configuration");
	  return KD_ERROR;
	}
	std::vector< double > kwsDofVector;
	kwsConfig.getDofValues (kwsDofVector);
	vectorN q (kwsDofVector.size ());
	robot->kwsToJrlDynamicsDofValues (kwsDofVector, q);
	if (!robot->hppSetCurrentConfig (kwsConfig)) {
	  hppDout (error, "Failed to set current configuration.");
	  return KD_ERROR;
	}
	matrix4d waistPos = robot->getRootJoint ()->jrlJoint ()
	  ->currentTransformation ();
	vector3d comPos = robot->positionCenterOfMass();
	comPos (2) = 0;
	matrix4d invWaistPos;
	MAL_S4x4_INVERSE (waistPos, invWaistPos, double);
	vector3d zmp = MAL_S4x4_RET_A_by_B (invWaistPos, comPos);

	file_rpy << t << " " << q [3] << " " << q [4] << " " << q [5]
		 << std::endl;
	file_zmp << t << " " << zmp [0] << " " << zmp [1] << " " << zmp [2]
		 << std::endl;
	assert (kwsDofVector.size () == 46);
	file_pos << t << " ";
	for (std::vector <double>::size_type i=6; i<=28; ++i)
	  file_pos << kwsDofVector [i] << " ";
	// Right hand
	for (std::vector <double>::size_type i=34; i<=40; ++i)
	  file_pos << kwsDofVector [i] << " ";
	// Left arm
	for (std::vector <double>::size_type i=29; i<=33; ++i)
	  file_pos << kwsDofVector [i] << " ";
	// Left hand
	for (std::vector <double>::size_type i=41; i<=45; ++i) {
	  file_pos << kwsDofVector [i];
	  if (i < 6+39) file_pos << " ";
	}
	file_pos << std::endl;
      }
      file_pos.close ();
      file_zmp.close ();
      file_rpy.close ();
      return KD_OK;
    }
  } //end of namespace constrained
} //end of namespace hpp
