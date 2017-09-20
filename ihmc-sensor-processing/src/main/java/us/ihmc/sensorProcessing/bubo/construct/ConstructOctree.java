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

import java.lang.reflect.Array;
import java.util.List;
import java.util.Stack;

import org.ddogleg.struct.FastQueue;

import georegression.struct.GeoTuple;

/**
 * Base class for constructing octrees.  Contains internal functions for managing data structures.  All data
 * structures are managed by this tree and can be reclaimed by calling {@link #reset}.
 *
 * @author Peter Abeles
 */
@SuppressWarnings("unchecked")
public abstract class ConstructOctree< O extends Octree, P extends GeoTuple> {

	// the Octree that it modifies
	protected O tree;

	// save references to all data structures declared to create the tree
	protected FastQueue<Octree.Info<P>> storageInfo = new FastQueue<Octree_F64.Info<P>>((Class)Octree.Info.class, true);
	// Contains all nodes in the tree
	protected FastQueue<O> storageNodes;
	protected Stack<O[]> storageChildren = new Stack<O[]>();

	protected Class<O> octreeType;
	/**
	 * Specifies graph construction parameters
	 */
	public ConstructOctree( Class<O> octreeType ) {
		this.octreeType = octreeType;
		storageNodes = new FastQueue<O>(octreeType, true);

		this.tree = storageNodes.grow();
	}

	/**
	 * Discards the existing tree structure and recycles its data.  All reference to external data not owned
	 * by the graph is discarded
	 */
	public void reset() {
		// remove references to external data
		for (int i = 0; i < storageInfo.size; i++) {
			Octree.Info info = storageInfo.data[i];
			info.userData = null;
			info.point = null;
		}
		storageInfo.reset();

		for (int i = 0; i < storageNodes.size; i++) {
			O o = storageNodes.data[i];
			if (o.children != null) {
				for (int j = 0; j < 8; j++) {
					o.children[j] = null;
				}

				storageChildren.add((O[])o.children);
			}
			o.userData = null;
			o.parent = null;
			o.children = null;
			o.points.reset();
		}
		storageNodes.reset();

		// add the root tree again now that everything has been cleaned up
		this.tree = storageNodes.grow();
	}

	/**
	 * Adds all points to the Octree
	 *
	 * @param points List of points to add
	 */
	public void addPoints(List<P> points) {
		int N = points.size();
		for (int i = 0; i < N; i++) {
			addPoint(points.get(i), null);
		}
	}

	/**
	 * Adds a point to the Octree.  If needed it will grow the tree
	 *
	 * @param point The point which is to be added
	 * @return The node which contains the point
	 */
	public abstract O addPoint(P point, Object data);

	/**
	 * A node was just split then it was realized that it should not have been split.  Undoes the split
	 * and recycles the data
	 *
	 * @param node Node which needs to become a leaf again.
	 */
	protected void undoSplit(O node) {
		for (int i = 0; i < 8; i++) {
			O o = (O)node.children[i];
			if (o != null) {
				// the order might be different, but the N most recent will be recycled
				storageNodes.removeTail();

				o.parent = null;
				o.points.reset();
				node.children[i] = null;
			}
		}
		storageChildren.add((O[])node.children);
		node.children = null;
	}

	/**
	 * Checks to see if the child already exists.  If not it creates the child.  Info is added to
	 * the child's points.
	 */
	protected O checkAddChild(O node, int index, Octree.Info info) {
		O child = checkAddChild(node, index);
		child.points.add(info);
		return child;
	}

	/**
	 * Checks to see if the child already exists.  If not it creates the child.  If the
	 * child won't occupy a physically possible space then null is returned.
	 */
	protected O checkAddChild(O node, int index) {
		O child = (O)node.children[index];
		if (child == null) {
			node.children[index] = child = storageNodes.grow();
			child.parent = node;
			setChildSpace(node, index, child);
			// no points to add to child since none of the previous ones belong to it
			if( !isSpaceValid(child)) {
				// remove this child
				storageNodes.removeTail();
				node.children[index] = child = null;
			}
		}
		return child;
	}

	/**
	 * Sets the space the child occupies based on its parent and which child it is.
	 * @param parent Parent node.
	 * @param index Which child on the parent
	 * @param child The child.
	 */
	public abstract void setChildSpace(O parent, int index, O child );

	/**
	 * Returns an array of Octree of length 8 with null elements.
	 */
	protected O[] getChildrenArray() {
		if (storageChildren.isEmpty()) {
			return (O[])Array.newInstance(octreeType,8);
		} else {
			return storageChildren.pop();
		}
	}

	/**
	 * Returns the Octree it has constructed
	 */
	public O getTree() {
		return tree;
	}

	/**
	 * List of all nodes in use
	 */
	public FastQueue<O> getAllNodes() {
		return storageNodes;
	}

	/**
	 * List of all points and associated data passed to the tree
	 */
	public FastQueue<Octree.Info<P>> getAllPoints() {
		return storageInfo;
	}

	/**
	 * Checks to see if the provided cube has a non-zero positive volume.  If the divisor is the
	 * same as one of the corners then physically impossible spaces can be constructed.
	 */
	public abstract boolean isSpaceValid( O node );
}
