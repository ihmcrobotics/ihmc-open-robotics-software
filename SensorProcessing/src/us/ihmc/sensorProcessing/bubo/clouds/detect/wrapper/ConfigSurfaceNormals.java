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

package us.ihmc.sensorProcessing.bubo.clouds.detect.wrapper;

/**
 * Configuration for {@link us.ihmc.sensorProcessing.bubo.clouds.detect.alg.ApproximateSurfaceNormals}.
 *
 * @author Peter Abeles
 */
public class ConfigSurfaceNormals {
	/**
	 * Number of neighbors it will use to approximate normal.  Can be useful to set to a higher number if
	 * nearest-neighbor graph is used by other algorithms
	 */
	public int numNeighbors;
	/**
	 * The maximum distance apart two points can be for them to be neighbors.  By default this is set to
	 * {@link Double#MAX_VALUE}.
	 */
	public double maxDistanceNeighbor = Double.MAX_VALUE;

	public ConfigSurfaceNormals(int numNeighbors, double maxDistanceNeighbor) {
		this.numNeighbors = numNeighbors;
		this.maxDistanceNeighbor = maxDistanceNeighbor;
	}

	public void checkConfig() {
	}
}

