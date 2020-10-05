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

package us.ihmc.sensorProcessing.bubo.clouds;

import org.ddogleg.fitting.modelset.ransac.RansacMulti;

import us.ihmc.sensorProcessing.bubo.clouds.detect.PointCloudShapeFinder;
import us.ihmc.sensorProcessing.bubo.clouds.detect.alg.ApproximateSurfaceNormals;
import us.ihmc.sensorProcessing.bubo.clouds.detect.alg.FindAllOfShapeInCloud;
import us.ihmc.sensorProcessing.bubo.clouds.detect.alg.PointVectorNN;
import us.ihmc.sensorProcessing.bubo.clouds.detect.wrapper.ConfigMultiShapeRansac;
import us.ihmc.sensorProcessing.bubo.clouds.detect.wrapper.ConfigSurfaceNormals;
import us.ihmc.sensorProcessing.bubo.clouds.detect.wrapper.Ransac_to_PointCloudShapeFinder;

/**
 * Factory for creating implementations of {@link PointCloudShapeFinder}.
 *
 * @author Peter Abeles
 */
public class FactoryPointCloudShape {
	/**
	 * Uses RANSAC to robustly find a single shape in the point cloud when given a set of shape types.
	 *
	 * @param configNormal Configuration for computing the shape's normal
	 * @param configRansac Configuration for RANSAC
	 * @return PointCloudShapeFinder
	 */
	public static PointCloudShapeFinder ransacSingle(ConfigSurfaceNormals configNormal,
													 ConfigMultiShapeRansac configRansac) {

		configNormal.checkConfig();

		ApproximateSurfaceNormals surface = new ApproximateSurfaceNormals(
				configNormal.numNeighbors, configNormal.maxDistanceNeighbor);

		RansacMulti<PointVectorNN> ransac = new RansacMulti<PointVectorNN>(
				configRansac.randSeed, configRansac.maxIterations, configRansac.models, PointVectorNN.class);

		return new Ransac_to_PointCloudShapeFinder(surface, ransac,
				configRansac.modelManagers, configRansac.fitters, configRansac.minimumPoints,
				configRansac.types);
	}

	/**
	 * Uses RANSAC to robustly find a single shape in the point cloud when given a set of shape types.
	 *
	 * @param configNormal Configuration for computing the shape's normal
	 * @param configRansac Configuration for RANSAC
	 * @return PointCloudShapeFinder
	 */
	public static PointCloudShapeFinder ransacSingleAll(ConfigSurfaceNormals configNormal,
														ConfigMultiShapeRansac configRansac) {

		configNormal.checkConfig();

		ApproximateSurfaceNormals surface = new ApproximateSurfaceNormals(
				configNormal.numNeighbors, configNormal.maxDistanceNeighbor);

		RansacMulti<PointVectorNN> ransac = new RansacMulti<PointVectorNN>(
				configRansac.randSeed, configRansac.maxIterations, configRansac.models, PointVectorNN.class);

		return new FindAllOfShapeInCloud(surface, ransac,
				configRansac.modelManagers, configRansac.fitters, configRansac.minimumPoints,
				configRansac.maximumNumberOfShapes,configRansac.types);
	}
}
