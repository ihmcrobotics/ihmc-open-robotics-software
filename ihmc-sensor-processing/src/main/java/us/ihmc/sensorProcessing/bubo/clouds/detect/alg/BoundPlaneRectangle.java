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

import org.ddogleg.struct.FastQueue;
import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.DecompositionFactory;
import org.ejml.interfaces.decomposition.EigenDecomposition;

import georegression.geometry.GeometryMath_F64;
import georegression.metric.ClosestPoint3D_F64;
import georegression.struct.plane.PlaneGeneral3D_F64;
import georegression.struct.point.Point2D_F64;
import georegression.struct.point.Point3D_F64;
import georegression.struct.point.Vector2D_F64;
import georegression.struct.point.Vector3D_F64;

/**
 * Given an equation for a plane and a set of points which belong to the plane, find a bounding rectangle which
 * lies on the plane.  The rectangle is found by projecting the points into a 2D coordinate system on the plane.
 * SVD is then used to find the dominant axis and the minimum/maximum distance along each axis is found.  A rectangle
 * is found and project back into 3D space.
 *
 * @author Peter Abeles
 */
public class BoundPlaneRectangle {

	FastQueue<Point2D_F64> points2D = new FastQueue<Point2D_F64>(Point2D_F64.class, true);

	Point3D_F64 origin = new Point3D_F64();
	Vector3D_F64 norm = new Vector3D_F64();
	Vector3D_F64 axisX = new Vector3D_F64();
	Vector3D_F64 axisY = new Vector3D_F64();

	Vector2D_F64 axis2X = new Vector2D_F64();
	Vector2D_F64 axis2Y = new Vector2D_F64();

	Point3D_F64 rect[];

	EigenDecomposition<DenseMatrix64F> eigen = DecompositionFactory.eig(3, true, true);
	DenseMatrix64F A = new DenseMatrix64F(2, 2);

	double meanX, meanY;


	public BoundPlaneRectangle() {
		rect = new Point3D_F64[4];
		for (int i = 0; i < rect.length; i++) {
			rect[i] = new Point3D_F64();
		}
	}

	public boolean process(PlaneGeneral3D_F64 plane, List<Point3D_F64> points) {
		if (points.isEmpty())
			throw new IllegalArgumentException("Points list cannot be empty");

		points2D.reset();

		// pick a point and find the closest point to the plane from it
		ClosestPoint3D_F64.closestPoint(plane, points.get(0), origin);

		// pick two arbitrary vectors to be the axis of the plane's 2D coordinate system
		norm.set(plane.A, plane.B, plane.C);
		axisX.set(plane.C, plane.A, plane.B);
		GeometryMath_F64.cross(norm, axisX, axisY);
		GeometryMath_F64.cross(norm, axisY, axisX);

		axisX.normalize();
		axisY.normalize();

		// find the 2D coordinate of each point
		meanX = meanY = 0;
		for (int i = 0; i < points.size(); i++) {
			Point3D_F64 p = points.get(i);

			double dx = p.x - origin.x;
			double dy = p.y - origin.y;
			double dz = p.z - origin.z;

			double X = axisX.x * dx + axisX.y * dy + axisX.z * dz;
			double Y = axisY.x * dx + axisY.y * dy + axisY.z * dz;

			points2D.grow().set(X, Y);

			meanX += X;
			meanY += Y;
		}

		// center of the points in the 2D coordinate system
		meanX /= points.size();
		meanY /= points.size();

		double dxdx = 0, dxdy = 0, dydy = 0;
		for (int i = 0; i < points2D.size(); i++) {
			Point2D_F64 p = points2D.get(i);

			double dx = p.x - meanX;
			double dy = p.y - meanY;

			dxdx += dx * dx;
			dxdy += dx * dy;
			dydy += dy * dy;
		}

		dxdx /= points2D.size();
		dxdy /= points2D.size();
		dydy /= points2D.size();

		// find eignevectors.  yes there is an analytic way to do this...
		if (!determineMajorAxises(dxdx, dxdy, dydy))
			return false;

		// find the rectangle in the new coordinate system
		double minX, minY;
		double maxX, maxY;

		minX = minY = Double.MAX_VALUE;
		maxX = maxY = -Double.MAX_VALUE;

		for (int i = 0; i < points2D.size(); i++) {
			Point2D_F64 p = points2D.get(i);

			double dx = p.x - meanX;
			double dy = p.y - meanY;

			double X = axis2X.x * dx + axis2X.y * dy;
			double Y = axis2Y.x * dx + axis2Y.y * dy;

			if (X < minX)
				minX = X;
			if (X > maxX)
				maxX = X;
			if (Y < minY)
				minY = Y;
			if (Y > maxY)
				maxY = Y;
		}

		convertTo3D(minX, minY, origin, rect[0]);
		convertTo3D(maxX, minY, origin, rect[1]);
		convertTo3D(maxX, maxY, origin, rect[2]);
		convertTo3D(minX, maxY, origin, rect[3]);

		return true;
	}

	private boolean determineMajorAxises(double dxdx, double dxdy, double dydy) {
		A.set(0, 0, dxdx);
		A.set(0, 1, dxdy);
		A.set(1, 0, dxdy);
		A.set(1, 1, dydy);

		if (!eigen.decompose(A))
			return false;

		DenseMatrix64F v0 = eigen.getEigenVector(0);
		DenseMatrix64F v1 = eigen.getEigenVector(1);

		axis2X.set(v0.get(0), v0.get(1));
		axis2Y.set(v1.get(0), v1.get(1));
		return true;
	}

	private void convertTo3D(double x, double y, Point3D_F64 origin, Point3D_F64 p3) {
		// rotated to standard 2D
		double X2 = x * axis2X.x + y * axis2Y.x + meanX;
		double Y2 = x * axis2X.y + y * axis2Y.y + meanY;

		// convert into 3D point
		p3.x = X2 * axisX.x + Y2 * axisY.x + origin.x;
		p3.y = X2 * axisX.y + Y2 * axisY.y + origin.y;
		p3.z = X2 * axisX.z + Y2 * axisY.z + origin.z;
	}

	/**
	 * Returns rotated rectangle in 3D which is order in clockwise or counterclockwise order.
	 *
	 * @return vertices of a rotated rectangle in 3D
	 */
	public Point3D_F64[] getRect() {
		return rect;
	}
}
