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

import georegression.metric.Intersection3D_F64;
import georegression.struct.point.Point3D_F64;
import georegression.struct.shapes.Box3D_F64;

/**
 * {@link us.ihmc.sensorProcessing.bubo.construct.Octree} implementation for doubles.
 *
 * @author Peter Abeles
 */
public class Octree_F64 extends Octree<Octree_F64,Point3D_F64>{
	/**
	 * Defines the space in which this node is contained
	 */
	public Box3D_F64 space = new Box3D_F64();
	/**
	 * The control point used to segment the space out into 8 children.  This is commonly the center
	 * of the cube.
	 */
	public Point3D_F64 divider = new Point3D_F64();

	@Override
	public boolean contained(Point3D_F64 point) {
		return Intersection3D_F64.contained(space, point);
	}

	/**
	 * Given a point inside the cube, return which child it belongs in.
	 *
	 * @param point A Point in space
	 * @return index of the child which contains it.
	 */
	@Override
	public int getChildIndex(Point3D_F64 point) {
		int quad;

		if (point.x < divider.x) {
			if (point.y < divider.y) {
				quad = 0;
			} else {
				quad = 1;
			}
		} else {
			if (point.y < divider.y) {
				quad = 2;
			} else {
				quad = 3;
			}
		}
		if (point.z >= divider.z) {
			quad += 4;
		}

		return quad;
	}
}
