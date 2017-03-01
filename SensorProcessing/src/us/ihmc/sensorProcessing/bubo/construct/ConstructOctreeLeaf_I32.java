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

import java.util.ArrayList;
import java.util.List;

import georegression.metric.Intersection3D_I32;
import georegression.struct.point.Point3D_I32;
import georegression.struct.shapes.Box3D_I32;

/**
 * Constructs an octree by adding points to the smallest possible leaf which contains
 * the point.  The smallest possible leaf is 1x1x1 wide and will thus have the same coordinate
 * as the point which was added to it.
 *
 * When a new point is added it is only added to the leaf node and not any of the other nodes on the
 * path from the root to the leaf.  This constructor is geared towards maps, where all the information
 * of interest can be found in leafs which are the smallest possible size.
 *
 * @author Peter Abeles
 */
public class ConstructOctreeLeaf_I32 extends ConstructOctree_I32 {

	/**
	 * Adds the point to the smallest leaf at this point.
	 *
	 * NOTE: This function is of dubious value since all points at the leaf will have
	 * the same coordinate so in many situations there is no point in keeping a list.
	 * Instead the leaf itself could be used.  See {@link #addLeaf(georegression.struct.point.Point3D_I32)}.
	 */
	@Override
	public Octree_I32 addPoint(Point3D_I32 point, Object data) {

		// declare the structure which stores the point and data
		Octree.Info<Point3D_I32> info = storageInfo.grow();
		info.point = point;
		info.userData = data;

		Octree_I32 leaf = addLeaf(point);
		if( leaf != null )
			leaf.points.add(info);

		return leaf;
	}

	/**
	 * Finds the smallest leaf at this point.  If none exist then a new one is created.  New nodes in
	 * the graph are created as it searches.  A reference to the leaf is returned.
	 * @param point Point which is contained by the desired smallest leaf.
	 * @return Smallest possible leaf at that point.
	 */
	public Octree_I32 addLeaf( Point3D_I32 point ) {
		if( !tree.contained(point) )
			return null;

		Octree_I32 node = tree;

		while( true ) {
			if( node.isSmallest() ) {
				return node;
			} else {
				if( node.isLeaf() ) {
					node.children = getChildrenArray();
					computeDivider(node.space, node.divider);
				}
				// Traverse down to the next child
				int index = node.getChildIndex(point);
				node = checkAddChild(node, index);
			}
		}
	}

	/**
	 * Finds and creates all leaf nodes which intersect the provided region.  If desired,
	 * external storage can be provided to avoid declaring new memory.
	 *
	 * @param region The region which is being tested for intersection
	 * @param output (Optional) Storage for list containing leafs.  Is cleared.
	 * @param workspace (Optional) Storage for internal book keeping. Is cleared.
	 * @return List of all the leaves inside the region.
	 */
	public List<Octree_I32> addLeafsIntersect( Box3D_I32 region , List<Octree_I32> output ,
											   List<Octree_I32> workspace) {
		if( output == null )
			output = new ArrayList<Octree_I32>();
		if( workspace == null )
			workspace = new ArrayList<Octree_I32>();

		List<Octree_I32> open = workspace;
		open.clear();
		output.clear();

		open.add(tree);

		while( !open.isEmpty() ) {
			Octree_I32 node = open.remove(open.size()-1);

			if( node.isLeaf() ) {
				if( node.isSmallest() ) {
					output.add(node);
					continue;
				} else {
					node.children = getChildrenArray();
					computeDivider(node.space, node.divider);
				}
			}

			// add all children which are contained inside the region
			for (int i = 0; i < 8; i++) {
				Octree_I32 child = checkAddChild(node, i);
				if( child != null && Intersection3D_I32.intersect(region,child.space)) {
					open.add(child);
				}
			}
		}

		return output;
	}

	/**
	 * Searches for all leafs which are of the smallest size within the region.  The
	 * tree will not be modified by this operation and no new leafs are added.
	 *
	 * @param region The region which is being tested for intersection
	 * @param output (Optional) Storage for list containing leafs.  Is cleared.
	 * @param workspace (Optional) Storage for internal book keeping. Is cleared.
	 * @return List of all the leaves inside the region.
	 */
	public List<Octree_I32> findLeafsIntersect( Box3D_I32 region , List<Octree_I32> output ,
												List<Octree_I32> workspace) {
		if( output == null )
			output = new ArrayList<Octree_I32>();
		if( workspace == null )
			workspace = new ArrayList<Octree_I32>();

		List<Octree_I32> open = workspace;
		open.clear();
		output.clear();

		open.add(tree);

		while( !open.isEmpty() ) {
			Octree_I32 node = open.remove(open.size()-1);

			if( node.isLeaf() ) {
				if( node.isSmallest() ) {
					output.add(node);
				}
				continue;
			}

			// add all children which are contained inside the region
			for (int i = 0; i < 8; i++) {
				Octree_I32 child = node.children[i];
				if( child != null && Intersection3D_I32.intersect(region,child.space)) {
					open.add(child);
				}
			}
		}

		return output;
	}
}
