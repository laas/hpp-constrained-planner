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
	DYNAMIC_PTR_CAST(hpp::model::HumanoidRobot,i_config->device());

      if (!robot) {
	return;
      }

      robot->hppSetCurrentConfig(i_config);
 
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
    Planner::buildSingleSupportStaticStabilityConstraints(CwsConfigShPtr i_config,
							  bool rightFootSupporting,
							  std::vector<CjrlGikStateConstraint*> & o_soc)
    {
      hpp::model::HumanoidRobotShPtr robot = 
	DYNAMIC_PTR_CAST(hpp::model::HumanoidRobot,i_config->device());

      if (!robot) {
	return;
      }

      robot->hppSetCurrentConfig(i_config);

      CjrlJoint * ankle = isRightFoot ? robot->rightAnkle() : robot->leftAnkle();
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
      return KD_OK;
    }

    ktStatus
    Planner::generateGoalConfigurations(unsigned int rank, unsigned int nb_configs)
    {
      if (!goalConfigGenerator_) {
	return KD_ERROR;
      }
      unsigned int nb_validConfigs=0;
      unsigned int nb_try=0;
      unsigned int nb_maxTry=10000;
      while ( (nb_try < nb_maxTry) 
	      && (nb_validConfigs < nb_configs) ) {
	//Shoot config
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
