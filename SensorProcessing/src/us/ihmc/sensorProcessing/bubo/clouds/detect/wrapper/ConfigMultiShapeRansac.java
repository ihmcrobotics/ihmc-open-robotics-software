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

import java.util.ArrayList;
import java.util.List;

import org.ddogleg.fitting.modelset.ModelFitter;
import org.ddogleg.fitting.modelset.ModelManager;
import org.ddogleg.fitting.modelset.ransac.RansacMulti;

import georegression.fitting.cylinder.FitCylinderToPoints_F64;
import georegression.fitting.cylinder.ModelManagerCylinder3D_F64;
import georegression.fitting.plane.ModelManagerPlaneGeneral3D_F64;
import georegression.fitting.sphere.FitSphereToPoints_F64;
import georegression.fitting.sphere.ModelManagerSphere3D_F64;
import us.ihmc.sensorProcessing.bubo.clouds.detect.CloudShapeTypes;
import us.ihmc.sensorProcessing.bubo.clouds.detect.alg.ModelFitter_P_to_PVNN;
import us.ihmc.sensorProcessing.bubo.clouds.detect.alg.PointVectorNN;
import us.ihmc.sensorProcessing.bubo.clouds.detect.shape.DistanceCylinderToPointVectorNN;
import us.ihmc.sensorProcessing.bubo.clouds.detect.shape.DistancePlaneToPointVectorNN;
import us.ihmc.sensorProcessing.bubo.clouds.detect.shape.DistanceSphereToPointVectorNN;
import us.ihmc.sensorProcessing.bubo.clouds.detect.shape.GenerateCylinderPointVector;
import us.ihmc.sensorProcessing.bubo.clouds.detect.shape.GeneratePlanePointVector;
import us.ihmc.sensorProcessing.bubo.clouds.detect.shape.GenerateSpherePointVector;

/**
 * Configures RANSAC which can match different shapes.
 *
 * @author Peter Abeles
 */
public class ConfigMultiShapeRansac {

	/**
	 * Random seed used by RANSAC
	 */
	public long randSeed = 0xFEEDBEEF;
	/**
	 * The maximum number of iterations RANSAC will perform.  Default 1000
	 */
	public int maxIterations = 1000;

	/**
	 * The minimum number of points a shape must have for it to be accepted
	 */
	public int minimumPoints = 0;

	/**
	 * The maximum number of shapes it will find
	 */
	public int maximumNumberOfShapes = Integer.MAX_VALUE;

	/**
	 * Description of the shapes RANSAC will search for.
	 */
	public List<RansacMulti.ObjectType> models;

	/**
	 * Which types of shapes are contains in the models list.  The elements in each list correspond to each other.
	 */
	public List<CloudShapeTypes> types;

	/**
	 * Used to create new shape parameter data structures. The elements in this list correspond to elements in models.
	 */
	public List<ModelManager> modelManagers;
	/**
	 * Used to create new shape parameter data structures. The elements in this list correspond to elements in models.
	 */
	public List<ModelFitter<Object, PointVectorNN>> fitters;

	/**
	 * Creates a default set of parameters which can detect
	 *
	 * @param fitIterations           Number of iterations when refining a shape
	 * @param angleTolerance          Tolerance in radians to reject a model from an initial sample set
	 * @param ransacDistanceThreshold Euclidean distance that RANSAC considers a point an inlier
	 * @param shapes                  A list of shape you wish to detect.  If empty or null then all possible shapes will be detected.
	 * @return ConfigSchnabel2007
	 */
	public static ConfigMultiShapeRansac createDefault(int fitIterations,
													   double angleTolerance,
													   double ransacDistanceThreshold,
													   CloudShapeTypes... shapes) {
		if (shapes == null || shapes.length == 0) {
			shapes = CloudShapeTypes.values();
		}
		List<RansacMulti.ObjectType> objects = new ArrayList<RansacMulti.ObjectType>();
		List<ModelManager> modelManagers = new ArrayList<ModelManager>();
		List<ModelFitter<Object, PointVectorNN>> fitters = new ArrayList<ModelFitter<Object, PointVectorNN>>();

		for (CloudShapeTypes shape : shapes) {
			switch (shape) {
				case SPHERE: {
					RansacMulti.ObjectType sphere = new RansacMulti.ObjectType();
					sphere.modelManager = new ModelManagerSphere3D_F64();
					sphere.modelDistance = new DistanceSphereToPointVectorNN(angleTolerance);
					sphere.modelGenerator = new GenerateSpherePointVector(angleTolerance, ransacDistanceThreshold);
					sphere.thresholdFit = ransacDistanceThreshold;
					objects.add(sphere);
					modelManagers.add(new ModelManagerSphere3D_F64());
					fitters.add(new ModelFitter_P_to_PVNN(new FitSphereToPoints_F64(fitIterations)));
				}
				break;

				case CYLINDER: {
					RansacMulti.ObjectType cylinder = new RansacMulti.ObjectType();
					cylinder.modelManager = new ModelManagerCylinder3D_F64();
					cylinder.modelDistance = new DistanceCylinderToPointVectorNN(angleTolerance);
					cylinder.modelGenerator = new GenerateCylinderPointVector(angleTolerance, ransacDistanceThreshold);
					cylinder.thresholdFit = ransacDistanceThreshold;
					objects.add(cylinder);
					modelManagers.add(new ModelManagerCylinder3D_F64());
					fitters.add(new ModelFitter_P_to_PVNN(new FitCylinderToPoints_F64(fitIterations)));
				}
				break;

				case PLANE: {
					RansacMulti.ObjectType plane = new RansacMulti.ObjectType();
					plane.modelManager = new ModelManagerPlaneGeneral3D_F64();
					plane.modelDistance = new DistancePlaneToPointVectorNN(angleTolerance);
					plane.modelGenerator = new GeneratePlanePointVector(angleTolerance);
					plane.thresholdFit = ransacDistanceThreshold;
					objects.add(plane);
					modelManagers.add(new ModelManagerPlaneGeneral3D_F64());
					fitters.add(new ModelFitter_P_to_PVNN(new PlaneGeneralSvd_to_ModelFitter()));
				}
				break;

				default:
					throw new IllegalArgumentException("Unsupported shape: " + shape);
			}
		}

		ConfigMultiShapeRansac config = new ConfigMultiShapeRansac();
		config.models = objects;
		config.modelManagers = modelManagers;
		config.fitters = fitters;
		config.types = new ArrayList<CloudShapeTypes>();
		for (CloudShapeTypes t : shapes) {
			config.types.add(t);
		}

		return config;
	}
}
