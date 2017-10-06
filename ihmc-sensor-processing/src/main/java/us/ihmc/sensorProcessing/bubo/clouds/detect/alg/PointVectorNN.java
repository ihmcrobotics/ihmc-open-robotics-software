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

import org.ddogleg.struct.FastQueue;

import georegression.struct.point.Point3D_F64;
import georegression.struct.point.Vector3D_F64;

/**
 * A point, normal vector tangent to the surface, and its neighbors.  Intended for use in
 * {@link PointCloudShapeDetectionSchnabel2007}
 *
 * @author Peter Abeles
 */
public class PointVectorNN {
	/**
	 * Reference to the point in the point cloud
	 */
	public Point3D_F64 p;
	/**
	 * Normal to the surface at p.  Is normalized to one.
	 */
	public Vector3D_F64 normal = new Vector3D_F64();

	/**
	 * Points which are its neighbors.  Does not include this point.
	 */
	public FastQueue<PointVectorNN> neighbors = new FastQueue<PointVectorNN>(PointVectorNN.class, false);

	/**
	 * Unique ID assigned to the point when it is constructed.  The 'index' in the same as the index in the list.
	 */
	public int index;

	/**
	 * Used to keep track of points which have been searched already
	 */
	public int matchMarker = 0;

	/**
	 * If true then the point is already used by a shape
	 */
	public boolean used;

	public PointVectorNN(double x, double y, double z, double nx, double ny, double nz) {
		p = new Point3D_F64(x, y, z);
		normal.set(nx, ny, nz);
	}

	public PointVectorNN() {
	}

	public String toString() {
		return "PointVector P( " + p.x + " , " + p.y + " , " + p.z + " ) Normal( " + normal.x + " , " + normal.y + " , " + normal.z + " )";
	}

	public void reset() {
		p = null;
		matchMarker = -1;
		used = false;
		index = -1;
		neighbors.reset();
	}
}
