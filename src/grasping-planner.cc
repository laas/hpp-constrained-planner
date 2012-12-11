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
	KIT_DYNAMIC_PTR_CAST(hpp::model::HumanoidRobot,
			     robotIthProblem (robotId));

      if (!robot) {
	return KD_ERROR;
      }
      robot->getCurrentConfig(halfSittingConfig_);

      // Initialize hand
      setHand (isRightHand_);
      return (Planner::initializeProblem());
    }

    void
    GraspingPlanner::setTarget(const vector3d & target)
    {
      target_ = target;
      if (positionConstraint_) positionConstraint_->worldTarget (target);
      if (gazeConstraint_) gazeConstraint_->worldTarget (target);
    }

    void
    GraspingPlanner::setHand(bool i_isRightHand)
    {
      isRightHand_ = i_isRightHand;
      hpp::model::HumanoidRobotShPtr robot =
	KIT_DYNAMIC_PTR_CAST(hpp::model::HumanoidRobot,
			     robotIthProblem (robotId));
      /* Build gik solver weights */
      ChppGikMaskFactory maskFactory(&(*robot));
      vectorN weightVector = maskFactory.wholeBodyMask ();

      /* Initialize hand position constraint */
      vector3d handCenter;
      vector3d thumbAxis;
      CjrlHand* hand = isRightHand_ ? robot->rightHand() : robot->leftHand();
      hand->getCenter (handCenter);
      hand->getThumbAxis (thumbAxis);
      CjrlJoint* wrist = hand->associatedWrist ();
      if (positionConstraint_) delete positionConstraint_;
      positionConstraint_ =
	new ChppGikPositionConstraint (*robot,*wrist, handCenter,target_);
      if (orientationConstraint_) delete orientationConstraint_;
      orientationConstraint_ =
	new ChppGikParallelConstraint (*robot, *wrist, thumbAxis,
				       vector3d (0,0,1));
      gazeConstraint_ =
	new ChppGikGazeConstraint (*robot, target_);

      // Initialize goal manifold stack of constraints
      std::vector<CjrlGikStateConstraint*>  goalSoc;
      buildDoubleSupportStaticStabilityConstraints(halfSittingConfig_,
						   goalSoc);
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
      buildDoubleSupportStaticStabilityConstraints(halfSittingConfig_,
						   planningSoc);
      if (configurationExtendor_) delete configurationExtendor_;
      configurationExtendor_ = new ConfigExtendor(robot);
      configurationExtendor_->setConstraints(planningSoc);
      configurationExtendor_->getGikSolver()->weights(weightVector);

      // Initialize goal configuration optimizer
      goalExtendor_ = new ConfigExtendor (robot);
      goalExtendor_->setConstraints (goalSoc);
    }

  } //end of namespace constrained
} //end of namespace hpp
