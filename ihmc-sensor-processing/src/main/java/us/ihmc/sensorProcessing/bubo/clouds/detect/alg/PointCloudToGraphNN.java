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

import java.util.List;
import java.util.Stack;

import org.ddogleg.nn.NearestNeighbor;
import org.ddogleg.nn.NnData;
import org.ddogleg.struct.FastQueue;

import georegression.struct.point.Point3D_F64;

/**
 * Creates a nearest-neighbor graph out of a point cloud
 *
 * @author Peter Abeles
 */
public class PointCloudToGraphNN {

	NearestNeighbor<PointVectorNN> nn;

	// the maximum distance a neighbor can be
	private double maxDistanceNeighbor;

	// number of nearest-neighbors it will search for
	private int numNeighbors;

	// stores and recycles Point3D converted to double[] for use in NN
	private Stack<double[]> unusedNnData = new Stack<double[]>();
	private Stack<double[]> usedNnData = new Stack<double[]>();

	// point normal data which is stored in the graph
	private FastQueue<PointVectorNN> listPointVector = new FastQueue<PointVectorNN>(PointVectorNN.class, true);

	// results of NN search
	private FastQueue<NnData<PointVectorNN>> resultsNN = new FastQueue<NnData<PointVectorNN>>((Class) NnData.class, true);

	public PointCloudToGraphNN(NearestNeighbor<PointVectorNN> nn,
							   int numNeighbors ,
							   double maxDistanceNeighbor ) {
		this.nn = nn;
		this.numNeighbors = numNeighbors;
		this.maxDistanceNeighbor = maxDistanceNeighbor;
	}

	/**
	 * Converts points into a format understood by the NN algorithm and initializes it
	 */
	public void process(List<Point3D_F64> cloud) {
		nn.init(3);

		// swap the two lists to recycle old data and avoid creating new memory
		Stack<double[]> tmp = unusedNnData;
		unusedNnData = usedNnData;
		usedNnData = tmp;
		// add the smaller list to the larger one
		unusedNnData.addAll(usedNnData);
		usedNnData.clear();

		// convert the point cloud into the NN format
		for (int i = 0; i < cloud.size(); i++) {
			Point3D_F64 p = cloud.get(i);

			double[] d;
			if (unusedNnData.isEmpty()) {
				d = new double[3];
			} else {
				d = unusedNnData.pop();
			}

			d[0] = p.x;
			d[1] = p.y;
			d[2] = p.z;

			usedNnData.add(d);
		}

		// declare the output data for creating the NN graph
		listPointVector.reset();
		for (int i = 0; i < cloud.size(); i++) {
			PointVectorNN p = listPointVector.grow();
			p.reset();
			p.p = cloud.get(i);
			p.index = i;
		}

		findNeighbors();
	}

	private void findNeighbors() {
		// find the nearest-neighbor for each point in the cloud
		nn.setPoints(usedNnData, listPointVector.toList());

		for (int i = 0; i < listPointVector.size; i++) {
			// find the nearest-neighbors
			resultsNN.reset();

			double[] targetPt = usedNnData.get(i);
			// numNeighbors+1 since the target node will also be returned and is removed
			nn.findNearest(targetPt, maxDistanceNeighbor, numNeighbors + 1, resultsNN);

			PointVectorNN p = listPointVector.get(i);

			// save the results
			p.neighbors.reset();
			for (int j = 0; j < resultsNN.size; j++) {
				NnData<PointVectorNN> n = resultsNN.get(j);

				// don't add the point to its own list of neighbors list
				if (n.point != targetPt) {
					p.neighbors.add(n.data);
				}
			}
		}
	}

	public FastQueue<PointVectorNN> getListPointVector() {
		return listPointVector;
	}
}
