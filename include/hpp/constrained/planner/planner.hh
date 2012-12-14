// Copyright (C) 2011 by Sebastien Dalibard.
//
// This file is part of the hpp-constrained-planner.
//
// hpp-constrained-planner is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either
// version 3 of the License, or (at your option) any later version.
//
// hpp-constrained-planner is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with hpp-constrained-planner.  If not, see
// <http://www.gnu.org/licenses/>.

#ifndef HPP_CONSTRAINED_PLANNER_HH
#define HPP_CONSTRAINED_PLANNER_HH

#include <deque>

#include <KineoWorks2/kwsDiffusionShooter.h>

#include <jrl/mal/matrixabstractlayer.hh>
#include <gikTask/jrlGikStateConstraint.h>

#include <hpp/core/planner.hh>

#include <hpp/constrained/fwd.hh>
#include "hpp/constrained/planner/fwd.hh"

namespace hpp {
  namespace constrained {
    /// Motion planning strategies for planning on constrained manifolds.

    /// This class relies on two objects from the hpp-constrained package:
    /// \li a ConfigProjector, which defines the goal submanifold,
    /// \li and a ConfigExtendor which defines the valid submanifold on which
    /// the planning is done.

    /// For genericity purpose, these two objects are not initialized in the
    /// hpp::constrained::Planner class. The user can either derive from
    /// hpp::constrained::Planner to define a planner adapted to a given
    /// problem (see for example the GraspingPlanner class), or initialize
    /// these objects outside the Planner class and pass them through their
    /// respective setters.
    class Planner : public hpp::core::Planner
    {
    public:
      typedef unsigned int size_type;
      /// Constructor.
      Planner();

      /// Destructor.
      virtual ~Planner();

      /// Build a stack of constraints corresponding to fixed double support
      /// stability.

      /// \param i_config Configuration where the feet are at the desired
      /// configuration
      /// \param o_soc Output stack of constraints
      static
      void
      buildDoubleSupportStaticStabilityConstraints
      (CkwsConfigShPtr i_config,
       std::vector<CjrlGikStateConstraint*> & o_soc);

      ///Builds a stack of constraints corresponding to fixed single support
      /// stability.

      /// \param i_config configuration where the foot is at the desired
      /// configuration
      /// \param rightFootSupporting Is the single support foot  right or left
      /// \param o_soc Output stack of constraints
      static
      void
      buildSingleSupportStaticStabilityConstraints
      (CkwsConfigShPtr i_config, bool rightFootSupporting,
       std::vector<CjrlGikStateConstraint*> & o_soc);

      /// Builds a stack of constraints corresponding to sliding double support
      /// stability

      /// The two feet are flat on the ground, at a fixed relative position,
      /// the relative position of the CoM is also fixed, but the robot
      /// is allowed to move globally.
      /// \param i_config Configuration where the feet are at the desired
      /// configuration
      /// \param o_soc Output stack of constraints
      static
      void
      buildDoubleSupportSlidingStaticStabilityConstraints
      (CkwsConfigShPtr i_config, std::vector<CjrlGikStateConstraint*> & o_soc);

      /// \name Problem definition
      /// @{

      /// \brief Add a Problem to the Problem vector with the associed robot.
       /// \param robot A shared pointer to a robot
      /// \param penetration Dynamic penetration for validating direct paths.
      /// \return KD_OK if success, KD_ERROR otherwise

      /// Call parent implementation and synchronize vector of goal
      /// configuration generators.
      virtual ktStatus addHppProblem(CkppDeviceComponentShPtr robot,
				     double penetration);

      /// \brief Remove a Problem at the end of the Problem vector.
      /// \return KD_OK if success, KD_ERROR otherwise
      /// Call parent implementation and synchronize vector of goal
      /// configuration generators.
      virtual ktStatus removeHppProblem();

      /// \brief Add a Problem at beginning of the Problem vector
      /// \param robot A shared pointer to a robot
      /// \param penetration dynamic penetration for validating direct paths.
      /// \return KD_OK if success, KD_ERROR otherwise

      /// Call parent implementation and synchronize vector of goal
      /// configuration generators.
      virtual ktStatus addHppProblemAtBeginning(CkppDeviceComponentShPtr robot,
						double penetration);

      /// \brief Remove a Problem at the beginning the Problem vector.
      /// \return KD_OK if success, KD_ERROR otherwise

      /// Call parent implementation and synchronize vector of goal
      /// configuration generators.
      virtual ktStatus removeHppProblemAtBeginning();

      /// @}
      /// Initialize constrained motion planning problem.

      /// \note Subclasses must first initialize the goal configuration
      /// generator and the configuration extendor and then call this function.
      /// \return KD_OK | KD_ERROR
      virtual
      ktStatus
      initializeProblem();

      /// Generate random goal configurations.
      /// \param rank Id of problem in vector
      /// \param nb_configs Number of goal configuration to generate
      /// \return KD_OK if succeeded in producing the required number of
      /// configurations, KD_ERROR otherwise.
      ///
      /// Generate a random configuration about the current robot
      /// configuration, project this configuration using
      /// Planner::goalConfigGenerator_, test for collision. Perform 50
      /// trials. Each success is added as a goal configuration to the problem:
      /// (hpp::core::Planner::addGoalConfIthProblem).
      ktStatus
      generateGoalConfigurations(unsigned int rank, unsigned int nb_configs);

      /// Set the goal configuration generator.
      /// \param i_goalConfigGenerator New goal configuration generator
      void
      goalConfigGenerator (unsigned int rank,
			   GoalConfigGeneratorShPtr goalConfigGenerator);

      /// Get the goal configuration generator.
      /// \return Goal configuration generator at given rank
      /// used
      GoalConfigGeneratorShPtr
      goalConfigGenerator(unsigned int rank) const;

      /// \brief Set the configuration extendor.
      /// \param i_configExtendor New configuration extendor
      void
      setConfigurationExtendor(ConfigExtendor * i_configExtendor);

      /// Get the configuration extendor.
      /// \return o_configExtendor Configuration extendor currently used
      ConfigExtendor* getConfigurationExtendor();

      /// Write a path in a openhrp seqplay files
      ktStatus writeSeqPlayFile (unsigned int rank, unsigned int pathId,
				 const std::string& prefix);

    private:
      /// Configuration projector used to generate goal configurations.
      std::deque <GoalConfigGeneratorShPtr> goalConfigGenerators_;

    protected:
      /// \brief Set robot dynamic properties.
      ///
      /// \param rank problem rank
      ///
      /// \param isDynamic specify whether or not dynamic quantities
      /// should be computed. In all cases, the center of mass is
      /// computed.
      ktStatus setDynamicProperties (const unsigned int rank,
				     const bool isDynamic);

      /// Configuration extendor, used by the roadmap builder to plan a
      /// constrained path
      ConfigExtendor * configurationExtendor_;

      /// Configuration extendor used by the ConfigOptimizer
      ///   Constraints are 
      ///     \li stability constraints,
      ///     \li goal task
      ///   Goal configuration is half-sitting.
      ConfigExtendor* goalExtendor_;

      /// Random configuration shooter used to generate random goal
      /// configurations
      CkwsDiffusionShooterShPtr configurationShooter_;

    };
  } //end of namespace constrained
} //end of namespace hpp

#endif //HPP_CONSTRAINED_PLANNER_HH
