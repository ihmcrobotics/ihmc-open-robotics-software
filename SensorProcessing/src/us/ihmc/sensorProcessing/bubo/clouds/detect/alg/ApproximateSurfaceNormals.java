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

import org.ddogleg.nn.FactoryNearestNeighbor;
import org.ddogleg.nn.NearestNeighbor;
import org.ddogleg.struct.FastQueue;

import georegression.fitting.plane.FitPlane3D_F64;
import georegression.struct.point.Point3D_F64;

/**
 * Given a point cloud, estimate the tangent to the surface at each point and record the nearest neighbors.
 * At each point, there are two possible directions for the tangent and one is arbitrarily selected for each point.
 * No effort is made to make them consistent.
 * <p></p>
 * The implementation below is inspired by [1], in that it approximates the normal using the nearest neighbors.
 * Unlike in [1], the normalizes are no organized in any way so that their directions is consistent.  The data
 * structures have also been customized to provide support for {@link PointCloudShapeDetectionSchnabel2007}.
 * <p></p>
 * The number of nearest-neighbors found and the number of which are used to compute the plane can be different.
 * The number of NN used to compute the plane must be more than 2 and the number computed total must be >=
 * the number used to compute the plane.  The points used to compute the plane are the ones closest to the point.
 * <p></p>
 * [1] Hoppe, H., DeRose, T., Duchamp, T., McDonald, J., & Stuetzle, W. "Surface reconstruction from unorganized
 * points" 1992, Vol. 26, No. 2, pp. 71-78. ACM.
 *
 * @author Peter Abeles
 */
public class ApproximateSurfaceNormals {

	// The algorithm used to search for nearest neighbors
	private PointCloudToGraphNN createGraph;

	// the local plane computed using neighbors
	private FitPlane3D_F64 fitPlane = new FitPlane3D_F64();
	// array to store points used to compute plane
	private List<Point3D_F64> fitList = new ArrayList<Point3D_F64>();

	// storage for the center point when computing the local surface normal
	private Point3D_F64 center = new Point3D_F64();

	/**
	 * Configures approximation algorithm
	 *
	 * @param nn                  Which nearest-neighbor algorithm to use
	 * @param numNeighbors        Number of neighbors it will find.  Can be useful if another algorithm wants more neighbors than this one will need.
	 * @param maxDistanceNeighbor The maximum distance two points can be from each other to be considered a neighbor
	 */
	public ApproximateSurfaceNormals(NearestNeighbor<PointVectorNN> nn,
									 int numNeighbors, double maxDistanceNeighbor) {
		this.createGraph = new PointCloudToGraphNN(nn,numNeighbors,maxDistanceNeighbor);
	}

	/**
	 * Configures approximation algorithm and uses a K-D tree by default.
	 *
	 * @param numNeighbors        Number of neighbors it will use to approximate normal
	 * @param maxDistanceNeighbor The maximum distance two points can be from each other to be considered a neighbor
	 */
	public ApproximateSurfaceNormals(int numNeighbors, double maxDistanceNeighbor) {
		this.createGraph = new PointCloudToGraphNN((NearestNeighbor)FactoryNearestNeighbor.kdtree(),numNeighbors,maxDistanceNeighbor);
	}

	/**
	 * Process point cloud and finds the shape's normals.  If a normal could not be estimated for the point
	 * its vector is set to (0,0,0).  A normal cannot be found for points with 1 or less neighbors.
	 *
	 * @param cloud  Input: 3D point cloud
	 * @param output Output: Storage for the point cloud with normals. Must set declareInstances to false.
	 */
	public void process(List<Point3D_F64> cloud, FastQueue<PointVectorNN> output) {

		// convert the point cloud into a format that the NN algorithm can recognize
		createGraph.process(cloud);

		FastQueue<PointVectorNN> listPointVector = createGraph.getListPointVector();

		// compute surface normal for each point using their neighbors
		for (int i = 0; i < listPointVector.size; i++) {
			PointVectorNN p = listPointVector.get(i);
			computeSurfaceNormal(p);
			output.add(p);
		}
	}

	/**
	 * Fits a plane to the nearest neighbors around the point and sets point.normal.
	 */
	protected void computeSurfaceNormal(PointVectorNN point) {
		// need 3 points to compute a plane.  which means you need two neighbors and 'point'
		if (point.neighbors.size >= 2) {
			fitList.clear();

			fitList.add(point.p);
			for (int i = 0; i < point.neighbors.size; i++) {
				PointVectorNN n = point.neighbors.get(i);
				fitList.add(n.p);
			}

			fitPlane.svd(fitList, center, point.normal);
		} else {
			point.normal.set(0, 0, 0);
		}
	}
}
