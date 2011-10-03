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

#ifndef HPP_CONSTRAINED_PLANNER_HH
#define HPP_CONSTRAINED_PLANNER_HH

#include <hpp/core/planner.hh>

#include <hpp/constrained/fwd.hh>

namespace hpp {
  namespace constrained {
    /**
     * \brief Motion planning strategies for planning on constrained manifolds.
     
     This class relies on two objects from the hpp-constrained package: a 
     config-projector, which defines the goal submanifold, and a config-extendor
     which defines the valid submanifold on which the planning is done.

     For genericity purpose, these two objects are not initialized in the 
     hpp::constrained::Planner class. The user can either derive from 
     hpp::constrained::Planner to define a planner adapted to a given problem (see 
     for example the GraspingPlanner class), or initialize these objects outside the 
     Planner class and pass them through their respective setters.
    */
    class Planner : public hpp::core::Planner
    {
    public:
      /**
       * \brief Constructor.
       */
      Planner();
      
      /**
       * \brief Destructor.
       */
      ~Planner();

      /**
       * \brief
       * Builds a stack of constraints corresponding to fixed double support stability. 
       * @param i_config Configuration where the feet are at the desired configuration
       * @param o_soc Output stack of constraints
       */
      static
      void
      buildDoubleSupportStaticStabilityConstraints(CkwsConfigShPtr i_config,
						   std::vector<CjrlGikStateConstraint*> & o_soc);
      
      /**
       * \brief
       * Builds a stack of constraints corresponding to fixed single support stability. 
       * @param i_config configuration where the foot is at the desired configuration
       * @param rightFootSupporting Is the single support foot  right or left
       * @param o_soc Output stack of constraints
       */
      static
      void
      buildSingleSupportStaticStabilityConstraints(CkwsConfigShPtr i_config,
						   bool rightFootSupporting,
						   std::vector<CjrlGikStateConstraint*> & o_soc);

      /**
       * \brief
       * Initialize constrained motion planning problem. Subclasses must first initialize the goal 
       * configuration generator and the configuration extendor and then call this function.
       * @return KD_OK | KD_ERROR
       */
      virtual
      ktStatus 
      initializeProblem();

      /**
       * \brief Generate random goal configurations.
       * @param rank Id of problem in vector
       * @param nb_configs Number of goal configuration to generate
       * @return KD_OK | KD_ERROR
       */
      ktStatus
      generateGoalConfigurations(unsigned int rank, unsigned int nb_configs);

      /**
       * \brief Set the goal configuration generator.
       * @param i_goalConfigGenerator New goal configuration generator
       */
      void
      setGoalConfigGenerator(ConfigProjector * i_goalConfigGenerator);
      
      /**
       * \brief Get the goal configuration generator.
       * @return o_goalConfigGenerator Goal configuration generator currently used
       */
      ConfigProjector *
      getGoalConfigGenerator();

      /**
       * \brief Set the configuration extendor.
       * @param i_configExtendor New configuration extendor
       */
      void
      setConfigurationExtendor(ConfigExtendor * i_configExtendor);

      /**
       * \brief Get the configuration extendor.
       * @return o_configExtendor Configuration extendor currently used
       */
      ConfigExtendor *
      getConfigurationExtendor();


    protected:
      /**
       * \brief Configuration projector used to generate goal configurations.
       */
      ConfigProjector * goalConfigGenerator_;
       
      /**
       * \brief Configuration extendor, used by the roadmap builder to plan a constrained path
       */
      ConfigExtendor * configurationExtendor_;

    };
  } //end of namespace constrained
} //end of namespace hpp

#endif //HPP_CONSTRAINED_PLANNER_HH
