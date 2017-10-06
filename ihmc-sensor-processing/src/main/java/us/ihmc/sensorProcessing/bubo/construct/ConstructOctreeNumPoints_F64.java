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

package us.ihmc.sensorProcessing.bubo.construct;

import georegression.struct.point.Point3D_F64;

/**
 * Creates an octree by adding new cells to the octree only when the number of points in a cell exceeds a specified
 * number.  The divider point is always the center of the cube and the same graph
 * will be produced independent of the order in which points are added.
 *
 * @author Peter Abeles
 */
public class ConstructOctreeNumPoints_F64 extends ConstructOctree_F64 {

	// create a new node in the graph when the number of points exceeds
	private int divideThreshold;


	/**
	 * Specifies graph construction parameters
	 *
	 * @param divideThreshold Create a new node with the number of points exceeds this threshold
	 */
	public ConstructOctreeNumPoints_F64(int divideThreshold) {
		this.divideThreshold = divideThreshold;
	}

	/**
	 * Adds a point to the Octree.  If needed it will grow the tree
	 *
	 * @param point The point which is to be added
	 * @return The node which contains the point
	 */
	@Override
	public Octree_F64 addPoint(Point3D_F64 point, Object data) {
		// declare the structure which stores the point and data
		Octree_F64.Info info = storageInfo.grow();
		info.point = point;
		info.userData = data;

		Octree_F64 node = tree;
		tree.points.add(info);

		while (true) {
			if (node.isLeaf()) {
				// see if it needs to create a new node
				if (node.points.size() > divideThreshold) {
					node.children = getChildrenArray();
					computeDivider(node.space, node.divider);

					// create a new child for point to go into
					int index = node.getChildIndex(point);
					Octree_F64 child = checkAddChild(node, index, info);

					// Create new children where appropriate for all points in node, but 'point'
					for (int i = 0; i < node.points.size - 1; i++) {
						Octree_F64.Info<Point3D_F64> infoP = node.points.get(i);
						int indexP = node.getChildIndex(infoP.point);

						// see if the node exists
						checkAddChild(node, indexP, infoP);
					}

					// check for the pathological case where all the points are identical
					boolean pathological = checkPathological(node);

					if (pathological) {
						// avoid infinite recursion by not splitting this node yet
						undoSplit(node);
						// search is done since it's at a leaf
						return node;
					} else {
						node = child;
					}
				} else {
					return node;
				}
			} else {
				int index = node.getChildIndex(point);
				node = checkAddChild(node, index, info);
			}
		}
	}

	/**
	 * If all the points are identical it will recurse forever since it can't split them.
	 */
	private boolean checkPathological(Octree_F64 node) {

		boolean pathological = true;
		Point3D_F64 first = node.points.data[0].point;
		for (int i = 1; i < node.points.size; i++) {
			Point3D_F64 p = node.points.data[i].point;
			if (first.x != p.x || first.y != p.y || first.z != p.z) {
				pathological = false;
				break;
			}
		}

		return pathological;
	}
}
