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

import java.util.ArrayList;
import java.util.List;

import org.ddogleg.fitting.modelset.ransac.RansacMulti;

import georegression.fitting.cylinder.CodecCylinder3D_F64;
import georegression.fitting.plane.CodecPlaneGeneral3D_F64;
import georegression.fitting.sphere.CodecSphere3D_F64;
import us.ihmc.sensorProcessing.bubo.clouds.detect.CloudShapeTypes;
import us.ihmc.sensorProcessing.bubo.clouds.detect.PointCloudShapeFinder;
import us.ihmc.sensorProcessing.bubo.clouds.detect.alg.ApproximateSurfaceNormals;
import us.ihmc.sensorProcessing.bubo.clouds.detect.alg.ConfigSchnabel2007;
import us.ihmc.sensorProcessing.bubo.clouds.detect.alg.FindAllOfShapeInCloud;
import us.ihmc.sensorProcessing.bubo.clouds.detect.alg.PointCloudShapeDetectionSchnabel2007;
import us.ihmc.sensorProcessing.bubo.clouds.detect.alg.PointVectorNN;
import us.ihmc.sensorProcessing.bubo.clouds.detect.alg.PostProcessShapes;
import us.ihmc.sensorProcessing.bubo.clouds.detect.alg.RemoveFalseShapes;
import us.ihmc.sensorProcessing.bubo.clouds.detect.alg.ShapeDescription;
import us.ihmc.sensorProcessing.bubo.clouds.detect.wrapper.ConfigMultiShapeRansac;
import us.ihmc.sensorProcessing.bubo.clouds.detect.wrapper.ConfigRemoveFalseShapes;
import us.ihmc.sensorProcessing.bubo.clouds.detect.wrapper.ConfigSurfaceNormals;
import us.ihmc.sensorProcessing.bubo.clouds.detect.wrapper.Ransac_to_PointCloudShapeFinder;
import us.ihmc.sensorProcessing.bubo.clouds.detect.wrapper.Schnable2007_to_PointCloudShapeFinder;

/**
 * Factory for creating implementations of {@link PointCloudShapeFinder}.
 *
 * @author Peter Abeles
 */
public class FactoryPointCloudShape {

	/**
	 * Returns an implementation of {@link PointCloudShapeFinder} which detects points using an Octree and RANSAC.
	 * By performing RANSAC locally inside Octree nodes its able to take advantage of the local structure of shapes
	 * and detect shapes at different scales.  Based on [1] paper.  See JavaDoc and code comments for significant
	 * deviations from original paper.
	 *
	 * @param configNormal Configuration for approximation of surface normals.
	 * @param configRansac Configuration for {@link PointCloudShapeDetectionSchnabel2007}.
	 * @param configPost   Configuration for {@link RemoveFalseShapes}.
	 * @return Implementation of {@link PointCloudShapeFinder}.
	 * @see ApproximateSurfaceNormals
	 * @see PointCloudShapeDetectionSchnabel2007
	 * <p/>
	 * [1] Schnabel, Ruwen, Roland Wahl, and Reinhard Klein. "Efficient RANSAC for Point‐Cloud Shape Detection."
	 * Computer Graphics Forum. Vol. 26. No. 2. Blackwell Publishing Ltd, 2007.
	 */
	public static PointCloudShapeFinder ransacOctree(ConfigSurfaceNormals configNormal,
													 ConfigSchnabel2007 configRansac,
													 ConfigRemoveFalseShapes configPost) {
		configNormal.checkConfig();

		PointCloudShapeDetectionSchnabel2007 alg = new PointCloudShapeDetectionSchnabel2007(configRansac);

		ApproximateSurfaceNormals surface = new ApproximateSurfaceNormals(configNormal.numNeighbors, configNormal.maxDistanceNeighbor);

//		PostProcessShapes postProcess = new MergeShapesPointVectorNN(
//				configMerge.commonMembershipFraction,configMerge.commonMembershipFraction);
		PostProcessShapes postProcess = new RemoveFalseShapes(configPost.ratio);

		postProcess.setup(configRansac.models, alg.getRefineShape());

		List<CloudShapeTypes> shapeList = new ArrayList<CloudShapeTypes>();

		for (ShapeDescription d : configRansac.models) {
			if (d.codec instanceof CodecSphere3D_F64) {
				shapeList.add(CloudShapeTypes.SPHERE);
			} else if (d.codec instanceof CodecCylinder3D_F64) {
				shapeList.add(CloudShapeTypes.CYLINDER);
			} else if (d.codec instanceof CodecPlaneGeneral3D_F64) {
				shapeList.add(CloudShapeTypes.PLANE);
			} else {
				throw new IllegalArgumentException("Unknown shape contained in configRansac.  Probably a bug.");
			}
		}

		return new Schnable2007_to_PointCloudShapeFinder(surface, alg, postProcess, shapeList);
	}

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
