/*
 * residual.cpp
 *
 *  Created on: 08.06.2017
 *      Author: burrimi
 */

#include "filter_test/residual.h"

namespace tsif {


void ResidualBase::setMeasurementTimelines(std::vector<Timeline*> timelines) { measurement_timelines_ = timelines; }


}  // namespace tsif
