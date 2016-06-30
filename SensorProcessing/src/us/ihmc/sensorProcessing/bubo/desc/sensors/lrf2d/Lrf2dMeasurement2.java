/*
 * Copyright (c) 2013-2014, Peter Abeles. All Rights Reserved.
 *
 * This file is part of Project BUBO.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package us.ihmc.sensorProcessing.bubo.desc.sensors.lrf2d;


/**
 * LADAR data that's specifies the range and angle for each measurement.  More general
 * purpose than {@link Lrf2dMeasurement} but can't precompute trigonometric operations.
 *
 * @author Peter Abeles
 */
public class Lrf2dMeasurement2 {

	// range measurements in standard units (e.g. meters)
	public double meas[];
	// the angle of each measurement in radians
	public double angle[];
	// number of valid measurements in this data.
	public int numMeas;

}
