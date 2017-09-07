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

import org.ddogleg.struct.FastQueue;

/**
 * Uses nearest-neighbor connectivity graph to find clusters of connected points.  Connect points not
 * in the input list are ignored.  "matchMarker" is modified.
 *
 * @author Peter Abeles
 */
@SuppressWarnings("ForLoopReplaceableByForEach")
public class SplitIntoClustersNN {

	List<List<PointVectorNN>> clusters = new ArrayList<List<PointVectorNN>>();

	FastQueue<PointVectorNN> open = new FastQueue<PointVectorNN>(PointVectorNN.class,false);

	/**
	 * Splits the provided cloud into clusters using the NN graph
	 * @param cloud List of points.
	 */
	public void process(List<PointVectorNN> cloud ) {
		System.out.println("SplitIntoClustersNN cloud "+cloud.size());
		clusters.clear();

		// Make that neighbors which are not a member of cloud are not included in island
		// creation.
		for (int i = 0; i < cloud.size(); i++) {
			PointVectorNN p = cloud.get(i);
			for (int j = 0; j < p.neighbors.size(); j++) {
				p.neighbors.get(j).matchMarker = -2;
			}
		}

		for (int i = 0; i < cloud.size(); i++) {
			cloud.get(i).matchMarker = -1;
		}

		for (int i = 0; i < cloud.size(); i++) {
			PointVectorNN p = cloud.get(i);
			if( p.matchMarker == -1 ) {
				markNeighbors(p);
			}
		}
		System.out.println("  total clusters "+clusters.size());
	}

	/**
	 * Put all the neighbors into the same list
	 */
	private void markNeighbors(PointVectorNN p ) {
		List<PointVectorNN> cluster = new ArrayList<PointVectorNN>();
		clusters.add(cluster);

		int mark = clusters.size();

		open.reset();
		open.add(p);
		p.matchMarker = mark;

		while( open.size() > 0 ) {
			p = open.removeTail();
			cluster.add(p);

			for (int i = 0; i < p.neighbors.size(); i++) {
				PointVectorNN n = p.neighbors.get(i);

				if( n.matchMarker == -1 ) {
					n.matchMarker = mark;
					open.add(n);
				}
				// neighbors are not always symmetric.  This it is possible to have an island in one direction
				// this case will be ignored.
			}
		}
	}

	public List<List<PointVectorNN>> getClusters() {
		return clusters;
	}
}
