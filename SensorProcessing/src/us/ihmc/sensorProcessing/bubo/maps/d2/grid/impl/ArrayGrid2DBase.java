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

package us.ihmc.sensorProcessing.bubo.maps.d2.grid.impl;

import us.ihmc.sensorProcessing.bubo.maps.d2.grid.OccupancyGrid2D;

/**
 * Implementation of some basic functions in OccupancyGrid2D
 *
 * @author Peter Abeles
 */
public abstract class ArrayGrid2DBase implements OccupancyGrid2D {

	// the map's width
	protected int width;
	// the map's height
	protected int height;

	protected ArrayGrid2DBase(int width, int height) {
		this.width = width;
		this.height = height;
	}

	@Override
	public boolean isInBounds(int x, int y) {
		return (x >= 0 && y >= 0 && x < width && y < height);
	}

	@Override
	public int getWidth() {
		return width;
	}

	@Override
	public int getHeight() {
		return height;
	}

	protected void checkBounds(int x, int y) {
		if (x >= width || x < 0)
			throw new IllegalArgumentException("x is out of bounds. x = " + x);

		if (y >= height || y < 0)
			throw new IllegalArgumentException("y is out of bounds.  y= " + y);
	}
}
