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

import org.ddogleg.fitting.modelset.ModelFitter;
import org.ddogleg.fitting.modelset.ModelManager;
import org.ddogleg.fitting.modelset.ransac.RansacMulti;
import org.ddogleg.struct.FastQueue;
import org.ddogleg.struct.GrowQueue_B;

import georegression.struct.point.Point3D_F64;
import georegression.struct.shapes.Box3D_F64;
import us.ihmc.sensorProcessing.bubo.clouds.detect.CloudShapeTypes;
import us.ihmc.sensorProcessing.bubo.clouds.detect.PointCloudShapeFinder;

/**
 * Uses RANSAC to find a best fit shape using the entire point cloud.  Approximate normals are first computed.
 *
 * @author Peter Abeles
 */
public class FindAllOfShapeInCloud implements PointCloudShapeFinder {

	RansacMulti<PointVectorNN> ransac;
	ApproximateSurfaceNormals surfaceNormals;
	List<CloudShapeTypes> shapeList;

	FastQueue<PointVectorNN> pointNormList = new FastQueue<PointVectorNN>(PointVectorNN.class, false);
	FastQueue<PointVectorNN> prunedList = new FastQueue<PointVectorNN>(PointVectorNN.class, false);
	// reference to cloud data
	List<Point3D_F64> cloud;
	// mark which points are inliers and which are not
	GrowQueue_B marks = new GrowQueue_B();
	// storage for the matched shape
	FastQueue<Shape> output = new FastQueue<Shape>(Shape.class, true);
	// optimizes the fit parameters to the inlier set
	List<ModelFitter<Object, PointVectorNN>> fitters;
	// storage for optimized parameters
	List<Object> models = new ArrayList<Object>();
	List<ModelManager> modelManagers;
	// the minimum number of points a shape needs for it to be accepted
	private int minimumPoints;

	// The maximum number of shapes it will find
	private int maxShapes;

	SplitIntoClustersNN splitter = new SplitIntoClustersNN();

	/**
	 * Specifies internal algorithms.
	 *
	 * @param surfaceNormals Algorithm used to compute surface normals
	 * @param ransac         RANSAC configured with the models its matching
	 * @param minimumPoints  The minimum number of points it will need to match
	 * @param maxShapes      The maximum number of shapes it will search for
	 * @param shapeList      List of shapes matching the RANSAC configuration
	 */
	public FindAllOfShapeInCloud(ApproximateSurfaceNormals surfaceNormals,
								 RansacMulti<PointVectorNN> ransac,
								 List<ModelManager> modelManagers,
								 List<ModelFitter<Object, PointVectorNN>> fitters,
								 int minimumPoints,
								 int maxShapes,
								 List<CloudShapeTypes> shapeList) {
		this.surfaceNormals = surfaceNormals;
		this.ransac = ransac;
		this.modelManagers = modelManagers;
		this.fitters = fitters;
		this.minimumPoints = minimumPoints;
		this.maxShapes = maxShapes;
		this.shapeList = shapeList;

		for (int i = 0; i < modelManagers.size(); i++) {
			models.add(modelManagers.get(i).createModelInstance());
		}
	}

	@Override
	public void process(List<Point3D_F64> cloud, Box3D_F64 boundingBox) {
		this.cloud = cloud;
		output.reset();
		pointNormList.reset();
		surfaceNormals.process(cloud, pointNormList);

		int numSplitFail = 0;

		for (int outerIter = 0; outerIter < maxShapes && numSplitFail < 5; outerIter++) {

			System.out.println("Cloud size    "+pointNormList.size());

			// run ransac and if it failed just give up
			if (!ransac.process(pointNormList.toList())) {
				System.out.println("Finished due to RANSAC false");
				return;
			}


			List<PointVectorNN> inliers = ransac.getMatchSet();
			System.out.println("  inlier size "+inliers.size());
			if (inliers.size() < minimumPoints) {
				System.out.println("Finished due to less than minimum points");
				return;
			}

			splitter.process(inliers);
			boolean allGood = false;

			for( List<PointVectorNN> cluster : splitter.getClusters() ) {
				if (cluster.size() < minimumPoints) {
					continue;
				}
				System.out.println("  cluster size "+cluster.size());
				allGood = true;

				// save the results
				addShapeToOutput(cluster);

				// mark the points for removal
				for (int i = 0; i < cluster.size(); i++) {
					cluster.get(i).used = true;
				}
			}

			if( !allGood ) {
				System.out.println("Split failed");
				numSplitFail++;
				continue;
			}

			// remove points which match this model from the list so they aren't selected again
			prunedList.reset();
			for (int i = 0; i < pointNormList.size(); i++) {
				PointVectorNN p = pointNormList.get(i);
				if( !p.used ) {
					prunedList.add(p);
				}
			}

			FastQueue<PointVectorNN> swap = pointNormList;
			pointNormList = prunedList;
			prunedList = swap;
		}
	}

	private void addShapeToOutput(List<PointVectorNN> inliers) {
		ModelFitter<Object, PointVectorNN> fitter = fitters.get(ransac.getModelIndex());
		Object shapeParam = models.get(ransac.getModelIndex());

		fitter.fitModel(inliers, ransac.getModelParameters(), shapeParam);

		// convert the results into output format
		Shape os = output.grow();
		os.parameters = modelManagers.get(ransac.getModelIndex()).createModelInstance();
		modelManagers.get(ransac.getModelIndex()).copyModel(shapeParam,os.parameters);
		os.type = shapeList.get(ransac.getModelIndex());
		os.points.clear();
		os.indexes.reset();

		// add the points to it
		for (int j = 0; j < inliers.size(); j++) {
			PointVectorNN pv = inliers.get(j);
			os.points.add(pv.p);
			os.indexes.add(pv.index);
		}
	}

	@Override
	public List<Shape> getFound() {
		return output.toList();
	}

	@Override
	public void getUnmatched(List<Point3D_F64> unmatched) {
		marks.resize(cloud.size());
		for (int i = 0; i < cloud.size(); i++) {
			marks.data[i] = false;
		}

		List<PointVectorNN> inliers = ransac.getMatchSet();
		for (int j = 0; j < inliers.size(); j++) {
			PointVectorNN pv = inliers.get(j);
			marks.data[pv.index] = true;
		}

		for (int i = 0; i < cloud.size(); i++) {
			if (!marks.data[i]) {
				unmatched.add(cloud.get(i));
			}
		}
	}

	@Override
	public List<CloudShapeTypes> getShapesList() {
		return shapeList;
	}

	@Override
	public boolean isSupportMultipleObjects() {
		return false;
	}
}
