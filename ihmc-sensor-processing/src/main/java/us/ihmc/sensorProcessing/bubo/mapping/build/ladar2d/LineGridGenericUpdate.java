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

package us.ihmc.sensorProcessing.bubo.mapping.build.ladar2d;

/**
 * <p>
 * Provides generalized algorithms for updating a map by traversing along a line,
 * which is useful for handling LADAR and similar sensors.  Line traversal and the
 * update functions have been separated for ease of testing and implementing.
 * </p>
 * <p/>
 * <p>
 * The line is drawn using Bresenham's algorithm.  The distance along the line as well
 * as the grid coordinate is passed onto an update function.
 * </p>
 *
 * @author Peter Abeles
 */
public abstract class LineGridGenericUpdate {

	/**
	 * Updates the map along the specified line.  All points are in grid coordinates.
	 *
	 * @param x0     Beginning of line. x-coordinate.
	 * @param y0     Beginning of line. y-coordinate.
	 * @param x1     Ending of line. x-coordinate.
	 * @param y1     Ending of line. y-coordinate.
	 * @param length The length of the specified line.  This is here since it is often already
	 *               known and recomputing it is unnecessary.
	 */
	public void update(double x0, double y0, double x1, double y1, double length) {
		boolean steep = Math.abs(y1 - y0) > Math.abs(x1 - x0);
		if (steep) {
			// swap x0 y0
			double t = x0;
			x0 = y0;
			y0 = t;
			// swap x1 y1
			t = x1;
			x1 = y1;
			y1 = t;
		}

		boolean reverse = x0 > x1;
		if (reverse) {
			// swap x0 y1
			double t = x0;
			x0 = x1;
			x1 = t;
			// swap y0 y1
			t = y0;
			y0 = y1;
			y1 = t;
		}

		int deltax = (int) (x1 - x0);
		int deltay = (int) Math.abs(y1 - y0);
		double error = 0;
		double deltaError = (double) deltay / (double) deltax;
		int ystep = y0 < y1 ? 1 : -1;
		int y = (int) y0;

		double deltaDist = reverse ? -length / deltax : length / deltax;
		double dist = reverse ? length : 0;

		for (int x = (int) x0; x <= x1; x++, dist += deltaDist) {
			if (steep) {
				update(y, x, dist);
			} else {
				update(x, y, dist);
			}
			error += deltaError;
			if (error >= 0.5) {
				y += ystep;
				error = error - 1.0;
			}
		}
	}

	/**
	 * The map grid coordinate that needs to be updated and the distance it is along the line.
	 *
	 * @param x    x-axis in map grid coordinates.
	 * @param y    y-axis in map grid coordinates.
	 * @param dist Distance along the line.
	 */
	protected abstract void update(int x, int y, double dist);
}
