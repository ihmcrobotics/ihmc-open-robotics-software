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
 * Configuration for {@link us.ihmc.sensorProcessing.bubo.clouds.detect.alg.MergeShapesPointVectorNN}.
 *
 * @author Peter Abeles
 */
public class ConfigMergeShapes {
	/**
	 * Minimum fraction of points in common that two objects have for them to be considered for merging.  Try 0.6
	 */
	public double commonPointsFraction = 0.6;

	/**
	 * Minimum fraction of points which belong to another the other shape for them to be merged. Try 0.9
	 */
	public double commonMembershipFraction = 0.9;

	public ConfigMergeShapes(double commonPointsFraction,
							 double commonMembershipFraction) {
		this.commonPointsFraction = commonPointsFraction;
		this.commonMembershipFraction = commonMembershipFraction;
	}

	public ConfigMergeShapes() {
	}
}
