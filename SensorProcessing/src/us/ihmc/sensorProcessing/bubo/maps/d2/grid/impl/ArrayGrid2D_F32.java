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

import us.ihmc.sensorProcessing.bubo.maps.d2.grid.OccupancyGrid2D_F32;

/**
 * Dense array floating point implementation of  OccupancyGrid2D_F32.
 *
 * @author Peter Abeles
 */
public class ArrayGrid2D_F32 extends ArrayGrid2DBase implements OccupancyGrid2D_F32 {

	// grid map in a row major format
	private float data[];

	public ArrayGrid2D_F32(int width, int height) {
		super(width, height);

		data = new float[width * height];
	}

	@Override
	public void set(int x, int y, float value) {
		checkBounds(x, y);

		data[y * width + x] = value;
	}

	@Override
	public float get(int x, int y) {
		checkBounds(x, y);

		return data[y * width + x];
	}

	@Override
	public void clear() {
		for (int i = 0; i < data.length; i++) {
			data[i] = 0.5f;
		}
	}

	@Override
	public boolean isKnown(int x, int y) {
		return data[y * width + x] != 0.5f;
	}

	@Override
	public boolean isValid(float value) {
		return value >= 0f && value <= 1f;
	}
}
