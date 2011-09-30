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

namespace hpp {
  namespace constrained {
    class Planner : public hpp::core::Planner
    {
      Planner();
      
      ~Planner();
    };
  } //end of namespace constrained
} //end of namespace hpp


#endif //HPP_CONSTRAINED_PLANNER_HH
