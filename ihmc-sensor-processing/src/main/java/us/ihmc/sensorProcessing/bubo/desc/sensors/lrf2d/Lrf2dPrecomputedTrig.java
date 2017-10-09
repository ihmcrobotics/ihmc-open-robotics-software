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

import georegression.struct.point.Point2D_F64;
import georegression.struct.point.Vector2D_F64;

/**
 * <p>
 * To cut down on the required computations when computing the location of each LRF scan point in 2D cartesian
 * coordinates in the sensors reference frame trigometric functions are precomputed.
 * The results of each computation can be stored in an internal (x,y) variables or written to a 2D point.
 * </p>
 *
 * @author Peter Abeles
 */
// todo move out of the desc package?
public class Lrf2dPrecomputedTrig {

	// Where each sine and cosine function is saved.  One for each scan.
	public double c[];
	public double s[];

	// end point of LRF scan
	public double x;
	public double y;

	/**
	 * Precomputes trigometric functions for the specified 2D LRF
	 *
	 * @param config
	 */
	public Lrf2dPrecomputedTrig(final Lrf2dParam config) {

		double start = config.getStartAngle();
		double sweep = config.getSweepAngle();
		final int N = config.getNumberOfScans();

		c = new double[N];
		s = new double[N];

		for (int i = 0; i < N; i++) {
			double theta = start + i*sweep/(N-1);
			c[i] = Math.cos(theta);
			s[i] = Math.sin(theta);
		}
	}

	/**
	 * Computes the 2D coordinate of the LRF scan point and saves it in the internal x and y variables.
	 *
	 * @param index Which LRF scan is being considered.
	 * @param range The scan's range from the sensor.
	 */
	public void computeEndPoint(int index, double range) {
		x = c[index] * range;
		y = s[index] * range;
	}

	/**
	 * Computes the 2D coordinate of the LRF scan point and saves it in the provided point.
	 *
	 * @param index Which LRF scan is being considered.
	 * @param range The scan's range from the sensor.
	 * @param pt    where the results are stored.
	 */
	public void computeEndPoint(int index, double range, Point2D_F64 pt) {
		pt.x = c[index] * range;
		pt.y = s[index] * range;
	}

	public void computeDirection( int index , Vector2D_F64 pt ) {
		pt.x = c[index];
		pt.y = s[index];
	}
}
