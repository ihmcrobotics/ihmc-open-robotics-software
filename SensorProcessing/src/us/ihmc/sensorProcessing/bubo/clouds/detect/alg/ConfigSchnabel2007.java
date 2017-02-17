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

package us.ihmc.sensorProcessing.bubo.clouds.detect.alg;

import java.util.ArrayList;
import java.util.List;

import georegression.fitting.cylinder.CodecCylinder3D_F64;
import georegression.fitting.cylinder.FitCylinderToPoints_F64;
import georegression.fitting.cylinder.ModelManagerCylinder3D_F64;
import georegression.fitting.plane.CodecPlaneGeneral3D_F64;
import georegression.fitting.plane.ModelManagerPlaneGeneral3D_F64;
import georegression.fitting.sphere.CodecSphere3D_F64;
import georegression.fitting.sphere.FitSphereToPoints_F64;
import georegression.fitting.sphere.ModelManagerSphere3D_F64;
import us.ihmc.sensorProcessing.bubo.clouds.detect.CloudShapeTypes;
import us.ihmc.sensorProcessing.bubo.clouds.detect.shape.DistanceCylinderToPointVectorNN;
import us.ihmc.sensorProcessing.bubo.clouds.detect.shape.DistancePlaneToPointVectorNN;
import us.ihmc.sensorProcessing.bubo.clouds.detect.shape.DistanceSphereToPointVectorNN;
import us.ihmc.sensorProcessing.bubo.clouds.detect.shape.GenerateCylinderPointVector;
import us.ihmc.sensorProcessing.bubo.clouds.detect.shape.GeneratePlanePointVector;
import us.ihmc.sensorProcessing.bubo.clouds.detect.shape.GenerateSpherePointVector;
import us.ihmc.sensorProcessing.bubo.clouds.detect.wrapper.PlaneGeneralSvd_to_ModelFitter;

/**
 * Tuning parameters for {@link PointCloudShapeDetectionSchnabel2007}
 *
 * @author Peter Abeles
 */
public class ConfigSchnabel2007 {
	/**
	 * description of the shapes it will search for
	 */
	public List<ShapeDescription> models;
	/**
	 * Maximum number of iterations it will perform while refining a shape after detection by RANSAC.
	 */
	public int localFitMaxIterations = 100;
	/**
	 * Exit criteria for shape refinement.  When the model parameters change less than this it exits.
	 */
	public double localFitChangeThreshold = 1e-8;
	/**
	 * Children will be created in an Octree node when the number of points contained inside exceed this number.
	 * Optimal value is data dependent.  Try 50.
	 */
	public int octreeSplit = 100;
	/**
	 * THe minimum number of points that are in the inlier set for a shape for the shape to be accepted.
	 */
	public int minModelAccept = 50;
	/**
	 * RANSAC will be extended by this many iterations each time a new best fit model is found.
	 */
	public int ransacExtension = 15;
	/**
	 * The total maximum number of allowed iterations to find a shape.  This is used to limit the time
	 * spent searching for shapes.  Try 1000
	 */
	public int maximumAllowedIterations = 1000;
	/**
	 * Random seed used in various places, such as RANSAC
	 */
	public long randomSeed = 0xDEADBEEF;

	/**
	 * Creates a default set of parameters which can detect
	 *
	 * @param fitIterations           Number of iterations when refining a shape
	 * @param angleTolerance          Tolerance in radians to reject a model from an initial sample set
	 * @param ransacDistanceThreshold Euclidean distance that RANSAC considers a point an inlier
	 * @param shapes                  A list of shape you wish to detect.  If empty or null then all possible shapes will be detected.
	 * @return ConfigSchnabel2007
	 */
	public static ConfigSchnabel2007 createDefault(int fitIterations,
												   double angleTolerance,
												   double ransacDistanceThreshold,
												   CloudShapeTypes... shapes) {
		if (shapes == null || shapes.length == 0) {
			shapes = CloudShapeTypes.values();
		}
		List<ShapeDescription> objects = new ArrayList<ShapeDescription>();

		int index = 0;
		for (CloudShapeTypes shape : shapes) {
			switch (shape) {
				case SPHERE: {
					ShapeDescription sphere = new ShapeDescription();
					sphere.modelManager = new ModelManagerSphere3D_F64();
					sphere.modelDistance = new DistanceSphereToPointVectorNN(angleTolerance);
					sphere.modelGenerator = new GenerateSpherePointVector(angleTolerance, ransacDistanceThreshold);
					sphere.modelFitter = new ModelFitter_P_to_PVNN(new FitSphereToPoints_F64(fitIterations));
					sphere.codec = new CodecSphere3D_F64();
					sphere.thresholdFit = ransacDistanceThreshold;
					objects.add(sphere);
				}
				break;

				case CYLINDER: {
					ShapeDescription cylinder = new ShapeDescription();
					cylinder.modelManager = new ModelManagerCylinder3D_F64();
					cylinder.modelDistance = new DistanceCylinderToPointVectorNN(angleTolerance);
					cylinder.modelGenerator = new GenerateCylinderPointVector(angleTolerance, ransacDistanceThreshold);
					cylinder.modelFitter = new ModelFitter_P_to_PVNN(new FitCylinderToPoints_F64(fitIterations));
					cylinder.codec = new CodecCylinder3D_F64();
					cylinder.thresholdFit = ransacDistanceThreshold;
					objects.add(cylinder);
				}
				break;

				case PLANE: {
					ShapeDescription plane = new ShapeDescription();
					plane.modelManager = new ModelManagerPlaneGeneral3D_F64();
					plane.modelDistance = new DistancePlaneToPointVectorNN(angleTolerance);
					plane.modelGenerator = new GeneratePlanePointVector(angleTolerance);
					plane.modelFitter = new ModelFitter_P_to_PVNN(new PlaneGeneralSvd_to_ModelFitter());
					plane.codec = new CodecPlaneGeneral3D_F64();
					plane.thresholdFit = ransacDistanceThreshold;
					objects.add(plane);
				}
				break;

				default:
					throw new IllegalArgumentException("Unsupported shape: " + shape);
			}
			index++;
		}

		ConfigSchnabel2007 config = new ConfigSchnabel2007();
		config.models = objects;

		return config;
	}

	/**
	 * Checks to see if the specified parameters are internally consistent
	 */
	public void checkConfig() {

	}
}
