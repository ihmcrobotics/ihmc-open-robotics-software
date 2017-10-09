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

package us.ihmc.sensorProcessing.bubo.clouds.fit;

import georegression.struct.se.Se2_F64;
import us.ihmc.sensorProcessing.bubo.desc.sensors.lrf2d.Lrf2dParam;

/**
 * Computes a rigid body motion {@link Se2_F64} which when applied to the 'source' scan will minimize the difference
 * between it and the 'destination' scan.
 *
 * @author Peter Abeles
 */
public interface Lrf2dScanToScan {

	/**
	 * Specifies the sensors intrinsic characteristics.
	 *
	 * @param param Sensor parameters.
	 */
	public void setSensorParam(Lrf2dParam param);

	/**
	 * If {@link #process(georegression.struct.se.Se2_F64)} returned true then this is the motion
	 * from the source to destination scans.
	 *
	 * @return The found motion.
	 */
	public Se2_F64 getSourceToDestination();

	/**
	 * Specifies range measurements for the destination scan.
	 *
	 * NOTE: A local copy of input array is made.  The data can be modified any time after the function
	 * has exited.
	 *
	 * @param scan range measurements.
	 */
	public void setDestination(double[] scan);

	/**
	 * Specifies range measurements for the source scan.
	 *
	 * NOTE: A local copy of input array is made.  The data can be modified any time after the function
	 * has exited.
	 *
	 * @param scan range measurements
	 */
	public void setSource(double[] scan);

	/**
	 * Takes the data associated with the source scan and sets it to be the destination scan.  Often times scan matching
	 * is done on a sequence of data and what was the second scan will become the first scan when new data arrives.
	 * This method is intended to avoid unnecessary recomputing or copying of data.
	 */
	public void assignSourceToDestination();

	/**
	 * Finds the motion which minimizes the error first and second scan.
	 *
	 * @param hintSrcToDst An initial estimate of the transform from the source to destination scan.  Often from odometry.
	 * @return true if registration was successful or false if not.
	 */
	public boolean process(Se2_F64 hintSrcToDst);

	/**
	 * A number which represents the error associated with the scan.  Primarily for debugging purposes,
	 * its meaning is implementation dependent.
	 *
	 * @return Error associated with the scan registration
	 */
	public double getError();

	/**
	 * Returns number of scans that were matched
	 * @return number of scans
	 */
	public int totalScansMatched();
}
