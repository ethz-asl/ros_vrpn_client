/*
 * Copyright 2015 Alex Millane, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef TEST_HELPER_LIBRARY_H
#define TEST_HELPER_LIBRARY_H

#include <math.h>

#include <Eigen/Geometry>

void euler2quat(double roll, double pitch, double yaw, double* q);

void calculate3dRmsError(double truth[][3], double est[][3],
                         const int trajectory_length, const int start_index,
                         double* error);

void calculateQuaternionRmsError(double truth[][4], double est[][4],
                                 const int trajectory_length,
                                 const int start_index, double* error);

#endif  // TEST_HELPER_LIBRARY_H
