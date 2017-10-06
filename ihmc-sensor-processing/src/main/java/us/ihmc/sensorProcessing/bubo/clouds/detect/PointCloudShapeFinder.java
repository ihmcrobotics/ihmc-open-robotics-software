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

package us.ihmc.sensorProcessing.bubo.clouds.detect;

import java.util.ArrayList;
import java.util.List;

import org.ddogleg.struct.GrowQueue_I32;

import georegression.struct.point.Point3D_F64;
import georegression.struct.shapes.Box3D_F64;

/**
 * High level interface for finding different primitive shapes inside a point cloud. A list of found shapes is returned
 * and points which were not assigned to any shapes.  For each shape the following information is returned, the
 * shape type, shape parameters,and set of points belonging to the shape.  Depending on the implementations
 * a single point can belong to one or more shapes.
 *
 * @author Peter Abeles
 */
public interface PointCloudShapeFinder {

	/**
	 * <p>
	 * Searches the point cloud for primitive shapes.
	 * </p>
	 * <p>
	 * WARNING: To reduce memory overhead, object which are passed have there reference saved.  This means you
	 * shouldn't modify the input if you care about the points returned with the output shape.
	 * </p>
	 *
	 * @param cloud       The point cloud which is to be searched
	 * @param boundingBox Optional bounding box which is known to contain all the points.
	 *                    If none is provided set to null.
	 */
	public void process(List<Point3D_F64> cloud, Box3D_F64 boundingBox);

	/**
	 * <p>
	 * List of shapes that were found in the point cloud.
	 * </p>
	 * <p>
	 * WARNING: All data structures returned are owned by this class.  They are subject to being modified
	 * the next time process is called.  The points might also be references to the input point cloud.
	 * </p>
	 *
	 * @return List of shapes
	 */
	public List<Shape> getFound();

	/**
	 * <p>
	 * A list of points which were not matched to any shape.  Calling this function can result in additional
	 * computations and should only be called once for each call to
	 * {@link #process(java.util.List, georegression.struct.shapes.Box3D_F64)};
	 * </p>
	 * <p/>
	 * <p>
	 * WARNING: All data structures returned are owned by this class.  They are subject to being modified
	 * the next time process is called.  The points might also be references to the input point cloud.
	 * </p>
	 *
	 * @param unmatched Storage for Point3D_F64 which matched the shape.
	 */
	public void getUnmatched(List<Point3D_F64> unmatched);

	/**
	 * Returns a list of shapes that it can find
	 *
	 * @return List of shape types
	 */
	public List<CloudShapeTypes> getShapesList();

	/**
	 * Can it detect multiple objects in the point cloud or does it just search for a single object?
	 *
	 * @return If true it supports finding multiple object.  If false then it will only search for a single object.
	 */
	public boolean isSupportMultipleObjects();

	/**
	 * Storage for output
	 */
	public static class Shape {
		/**
		 * The type of shape
		 */
		public CloudShapeTypes type;
		/**
		 * Parameters for the shape.  See {@link CloudShapeTypes} for data structute type.
		 */
		public Object parameters;
		/**
		 * List of points that matched this shape
		 */
		public List<Point3D_F64> points = new ArrayList<Point3D_F64>();
		/**
		 * Index of the point in the original cloud
		 */
		public GrowQueue_I32 indexes = new GrowQueue_I32();

		public <T>T getParameters() {
			return (T)parameters;
		}
	}
}
