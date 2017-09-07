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
import georegression.struct.shapes.Box3D_F64;

/**
 * Base class for constructing {@link us.ihmc.sensorProcessing.bubo.construct.Octree_F64}.
 *
 * @author Peter Abeles
 */
public abstract class ConstructOctree_F64 extends ConstructOctree<Octree_F64,Point3D_F64>{

	public ConstructOctree_F64() {
		super(Octree_F64.class);
	}

	/**
	 * Initializes the Octree.  The space contained by the Octree is specified by the passed in cube.
	 * {@link #reset} is automatically called by this function
	 *
	 * @param cube Space which is contained by the Octree.
	 */
	public void initialize(Box3D_F64 cube) {
		reset();
		tree.space.set(cube);
	}

	@Override
	public void setChildSpace(Octree_F64 parent, int index, Octree_F64 child) {
		setChildSpace(parent.space,parent.divider,index,child.space);
	}

	/**
	 * Sets the divider to the center of space
	 */
	public static void computeDivider(Box3D_F64 space, Point3D_F64 divider) {

		divider.x = (space.p0.x + space.p1.x) / 2.0;
		divider.y = (space.p0.y + space.p1.y) / 2.0;
		divider.z = (space.p0.z + space.p1.z) / 2.0;
	}

	public static void setChildSpace(Box3D_F64 parentSpace, Point3D_F64 parentDivider, int index,
									 Box3D_F64 childSpace) {

		childSpace.p0.set(parentSpace.p0);

		// no change for index 0
		if (index == 1) {
			childSpace.p0.y = parentDivider.y;
		} else if (index == 2) {
			childSpace.p0.x = parentDivider.x;
		} else if (index == 3) {
			childSpace.p0.x = parentDivider.x;
			childSpace.p0.y = parentDivider.y;
		} else if (index == 4) {
			childSpace.p0.z = parentDivider.z;
		} else if (index == 5) {
			childSpace.p0.y = parentDivider.y;
			childSpace.p0.z = parentDivider.z;
		} else if (index == 6) {
			childSpace.p0.x = parentDivider.x;
			childSpace.p0.z = parentDivider.z;
		} else if (index == 7) {
			childSpace.p0.x = parentDivider.x;
			childSpace.p0.y = parentDivider.y;
			childSpace.p0.z = parentDivider.z;
		}

		childSpace.p1.x = childSpace.p0.x + parentSpace.getLengthX() / 2.0;
		childSpace.p1.y = childSpace.p0.y + parentSpace.getLengthY() / 2.0;
		childSpace.p1.z = childSpace.p0.z + parentSpace.getLengthZ() / 2.0;
	}

	/**
	 * Always returns true since the divisor can never be the same as any of the corners.  Unless
	 * someone really screwed up.
	 */
	@Override
	public boolean isSpaceValid(Octree_F64 node) {
		return true;
	}
}
