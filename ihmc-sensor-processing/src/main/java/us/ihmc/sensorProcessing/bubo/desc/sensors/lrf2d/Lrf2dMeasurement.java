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
 * <p>
 * Range observations from a planar ladar.  The angular components for each scan is described
 * by the PlanarLadarParam associated with the sensor at the time the observations were made.
 * </p>
 * <p/>
 * <p>
 * The number of measurements is provided to make it easier to reuse the same object for
 * different sources that have different number of range measurements and for transmitting this
 * information.
 * </p>
 *
 * @author Peter Abeles
 */
public class Lrf2dMeasurement {

	// range measurements in standard units (e.g. meters)
	public double meas[];
	// number of valid measurements in this data.
	public int numMeas;

	public Lrf2dMeasurement(int numMeas) {
		this.numMeas = numMeas;
		meas = new double[numMeas];
	}

	public Lrf2dMeasurement() {
	}

	public void setMeasurements( Lrf2dMeasurement src ) {
		System.arraycopy(src.meas,0,meas,0,numMeas);
	}

	public void setMeasurements( double []meas ) {
		System.arraycopy(meas,0,this.meas,0,numMeas);
	}
}
