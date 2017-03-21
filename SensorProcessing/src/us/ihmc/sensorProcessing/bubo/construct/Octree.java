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

import java.util.List;

import org.ddogleg.struct.FastQueue;

import georegression.struct.GeoTuple;

/**
 * Octree data structure which uses a point to define sub-structure inside.  Each node in the tree contains 8 children.
 * Which child a point belongs to is defined by the 'space' of the node and the 'divider' point, see below. If
 * a node is a leaf then the 'children' data structure will be null.  Each 3D point can optionally have a Object
 * reference associated with it.  For example, color or other information.
 * <p/>
 * <p>
 * <ul>
 * <li>children[0] = p.x <  divider.x and p.y <  divider.y and p.z <  divider.z</li>
 * <li>children[1] = p.x <  divider.x and p.y >= divider.y and p.z <  divider.z</li>
 * <li>children[2] = p.x >= divider.x and p.y <  divider.y and p.z <  divider.z</li>
 * <li>children[3] = p.x >= divider.x and p.y >= divider.y and p.z <  divider.z</li>
 * <li>children[4] = p.x <  divider.x and p.y <  divider.y and p.z >= divider.z</li>
 * <li>children[5] = p.x <  divider.x and p.y >= divider.y and p.z >= divider.z</li>
 * <li>children[6] = p.x >= divider.x and p.y <  divider.y and p.z >= divider.z</li>
 * <li>children[7] = p.x >= divider.x and p.y >= divider.y and p.z >= divider.z</li>
 * </ul>
 * </p>
 *
 * @author Peter Abeles
 */
@SuppressWarnings("unchecked")
public abstract class Octree< O extends Octree, P extends GeoTuple> {
	/**
	 * The control point used to segment the space out into 8 children.  This is commonly the center
	 * of the cube.
	 */
	public P divider;

	/**
	 * Children of the node.  If a leaf then the children will be null.
	 */
	public O children[];

	/**
	 * The parent of this node
	 */
	public O parent;

	/**
	 * Reference to user specified data
	 */
	public Object userData;

	/**
	 * Points contained inside this node.  Depending on how it was constructed, all the points might be
	 * contained in the leafs or not.  New points are not declared by the FastQueue, just the storage array
	 */
	public FastQueue<Info<P>> points = new FastQueue<Info<P>>((Class)Info.class, false);

	/**
	 * Returns true if it is a leaf node or false if it is not
	 *
	 * @return true for leaf node and false if not.
	 */
	public boolean isLeaf() {
		return children == null;
	}

	/**
	 * Finds all the nodes in the Octree which contain the point.  The search stops when it hits a leaf.
	 *
	 * @param point (Input) Point which is being searched for
	 * @param path  (Output) All the nodes which contain point.  Order will be from general to specific.
	 */
	public void findPathToPoint(P point, List<Octree> path) {
		Octree node = this;

		// see if it is inside this space
		if (!contained(point))
			return;

		while (node != null) {
			path.add(node);

			if (node.isLeaf())
				break;

			node = node.children[node.getChildIndex(point)];
		}
	}

	/**
	 * Traverses down the octree and searches for the deepest node which contains the point
	 *
	 * @param point Point in which the leaf is contained.
	 * @return The deepest node which contains the point.  null if it's not bounded by the Octree.
	 */
	public O findDeepest(P point) {
		// see if it is inside this space
		if (!contained(point))
			return null;

		O node = (O)this;

		while (true) {
			if (node.isLeaf()) {
				return node;
			} else {
				int index = node.getChildIndex(point);
				O next = (O)node.children[index];
				if (next == null)
					return node;
				else
					node = next;
			}
		}
	}

	/**
	 * Returns true if the specified point is contained inside the space occupied by this name.
	 * @param point A point
	 * @return true if the point is contained inside and false if not
	 */
	public abstract boolean contained( P point );

	/**
	 * Given a point inside the cube, return which child it belongs in.
	 *
	 * @param point A Point in space
	 * @return index of the child which contains it.
	 */
	public abstract int getChildIndex(P point);

	public <T> T getUserData() {
		return (T) userData;
	}

	public void setUserData(Object userData) {
		this.userData = userData;
	}

	public static class Info<P extends GeoTuple> {
		/**
		 * The point which was added to the Octree
		 */
		public P point;
		/**
		 * User specified data
		 */
		public Object userData;

		public <T>T getUserData() {
			return (T)userData;
		}
	}
}
