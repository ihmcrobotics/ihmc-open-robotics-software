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

package us.ihmc.sensorProcessing.bubo.maps.d2.grid;

/**
 * Interface for 32bit floating point occupancy grid.  An occupancy grid stores the probability of a square region
 * being occupied by an obstacle or not.  A value of 1 means 100% and 0 means 0%, 50% is unknown or equal probability.
 *
 * @author Peter Abeles
 */
public interface OccupancyGrid2D_F32 extends OccupancyGrid2D {

	/**
	 * Sets the specified cell to 'value'.
	 *
	 * @param x     x-coordinate of the cell.
	 * @param y     y-coordinate of the cell.
	 * @param value The cell's new value.
	 */
	public void set(int x, int y, float value);

	/**
	 * Gets the value of the cell at the specified coordinate.
	 *
	 * @param x x-coordinate of the cell.
	 * @param y y-coordinate of the cell.
	 * @return The cell's value.
	 */
	public float get(int x, int y);

	/**
	 * Checks to see if the provided value is within the valid range.
	 *
	 * @param value the value being tested
	 * @return if it is valid or not
	 */
	public boolean isValid(float value);
}
