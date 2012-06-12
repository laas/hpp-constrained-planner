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

#include <KineoKCDModel/kppKCDBox.h>

#include <hpp/model/humanoid-robot.hh>
#include <hpp/model/joint.hh>

#include <hpp/gik/robot/mask-factory.hh>

#include <hpp/constrained/roadmap-builder.hh>
#include <hpp/constrained/config-projector.hh>
#include <hpp/constrained/goal-config-generator.hh>
#include <hpp/constrained/kws-constraint.hh>

#include <hpp/constrained/planner/grasping-planner.hh>


namespace hpp {
  namespace constrained {
    GraspingPlanner::
    GraspingPlanner::size_type GraspingPlanner::robotId = 0;

    GraspingPlanner::GraspingPlanner(bool i_isRightHand,vector3d i_target):
      isRightHand_(i_isRightHand),
      target_(i_target),
      positionConstraint_(0),
      orientationConstraint_ (0),
      gazeConstraint_ (0)
    {
      srand(time(0));
    }

    GraspingPlanner::~GraspingPlanner()
    {
      if(configurationExtendor_) {
	configurationExtendor_->resetConstraints();
	delete configurationExtendor_;
      }
    }

    ktStatus
    GraspingPlanner::initializeProblem()
    {
      hpp::model::HumanoidRobotShPtr robot =
	KIT_DYNAMIC_PTR_CAST(hpp::model::HumanoidRobot, robotIthProblem (0));

      if (!robot) {
	return KD_ERROR;
      }
      CkwsConfigShPtr halfSittingConfig;
      robot->getCurrentConfig(halfSittingConfig);

      /* Build gik solver weights */
      ChppGikMaskFactory maskFactory(&(*robot));
      vectorN weightVector = maskFactory.weightsDoubleSupport ();

      /* Initialize hand position constraint */
      CjrlJoint * wrist = isRightHand_ ? robot->rightWrist() : robot->leftWrist() ;
      positionConstraint_ = new ChppGikPositionConstraint (*robot,*wrist,vector3d(0,0,0),target_);

      /* Initialize goal manifold stack of constraints */
      std::vector<CjrlGikStateConstraint*>  goalSoc;
      buildDoubleSupportStaticStabilityConstraints(halfSittingConfig,goalSoc);
      goalSoc.push_back(positionConstraint_);
      goalSoc.push_back(orientationConstraint_);
      goalSoc.push_back(gazeConstraint_);

      GoalConfigGeneratorShPtr gcg =
	GoalConfigGenerator::create (robot);
      gcg->setConstraints(goalSoc);
      gcg->getGikSolver()->weights(weightVector);
      goalConfigGenerator (robotId, gcg);

      /* Initialize planning manifold stack of constraints */
      std::vector<CjrlGikStateConstraint*> planningSoc;
      buildDoubleSupportStaticStabilityConstraints(halfSittingConfig,planningSoc);
      configurationExtendor_ = new ConfigExtendor(robot);
      configurationExtendor_->setConstraints(planningSoc);
      configurationExtendor_->getGikSolver()->weights(weightVector);

      // Initialize goal configuration optimizer
      goalExtendor_ = new ConfigExtendor (robot);
      goalExtendor_->setConstraints (goalSoc);
    }

  } //end of namespace constrained
} //end of namespace hpp
