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

#ifndef HPP_CONSTRAINED_GRASPING_PLANNER_HH
#define HPP_CONSTRAINED_GRASPING_PLANNER_HH

#include <jrl/mal/matrixabstractlayer.hh>

#include <hpp/gik/constraint/position-constraint.hh>

#include <hpp/constrained/planner/planner.hh>

namespace hpp {
  namespace constrained {
    /**
     * \brief Planner for grasping tasks while keeping static balance.
     */
    class GraspingPlanner : public Planner
    {
    public:
      /**
       * \brief Constructor.
       */
      GraspingPlanner(bool i_isRightHand = true,vector3d i_target=vector3d(0.6,-0.1,0.9));

      /**
       * \brief Destructor.
       */
      ~GraspingPlanner();

      /**
       * \brief
       * Initialize grasping problem. The current configuration of the robot is used to define static stability constraints.
       */
      virtual
      ktStatus
      initializeProblem();

      /**
       * \brief
       * Set the target for the grasping task. Must be called before initializeProblem() in order to be taken into account.
       * @param i_target Point target in world frame
       */
      void
      setTarget(vector3d & i_target);

      /**
       * \brief
       * Set the hand performing the task. Must be called before initializeProblem() in order to be taken into account.
       * @param i_isRightHand true if the robot should use its right hand, false if it is the left one
       */
      void
      setHand(bool i_isRightHand);

     private:
      /**
       * \brief Grasping hand.
       */
      bool isRightHand_;

      /**
       * \brief Target in world frame.
       */
      vector3d target_;

      /**
       * \brief Gik goal task.
       */
      ChppGikPositionConstraint * positionConstraint_;

    };
  } //end of namespace constrained
} //end of namespace hpp

#endif //HPP_CONSTRAINED_GRASPING_PLANNER_HH
