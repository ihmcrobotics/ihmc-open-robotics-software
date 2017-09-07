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

package us.ihmc.sensorProcessing.bubo.clouds.detect.tools;

import org.ejml.data.DenseMatrix64F;

import georegression.geometry.ConvertRotation3D_F64;
import georegression.geometry.GeometryMath_F64;
import georegression.struct.plane.PlaneNormal3D_F64;
import georegression.struct.point.Point3D_F64;
import georegression.struct.point.Vector3D_F64;
import georegression.struct.shapes.Cylinder3D_F64;
import georegression.struct.shapes.Sphere3D_F64;
import georegression.struct.so.Rodrigues_F64;

/**
 * Various tools for shapes in point clouds.
 *
 * @author Peter Abeles
 */
public class PointCloudShapeTools {

	/**
	 * Creates a 3D point on the surface of the provided sphere at the specified coordinate
	 *
	 * @param sphere Description of the sphere
	 * @param phi    angular coordinate in radians
	 * @param theta  angular coordinate in radians
	 * @return Point on the sphere
	 */
	public static Point3D_F64 createPt(Sphere3D_F64 sphere, double phi, double theta) {
		Point3D_F64 p = new Point3D_F64();
		p.set(0, 0, sphere.radius);


		Rodrigues_F64 rodX = new Rodrigues_F64(phi, new Vector3D_F64(1, 0, 0));
		DenseMatrix64F rotX = ConvertRotation3D_F64.rodriguesToMatrix(rodX, null);
		Rodrigues_F64 rodZ = new Rodrigues_F64(theta, new Vector3D_F64(0, 0, 1));
		DenseMatrix64F rotZ = ConvertRotation3D_F64.rodriguesToMatrix(rodZ, null);

		GeometryMath_F64.mult(rotX, p, p);
		GeometryMath_F64.mult(rotZ, p, p);
		p.x += sphere.center.x;
		p.y += sphere.center.y;
		p.z += sphere.center.z;

		return p;
	}

	/**
	 * Creates a 3D point on the surface of the provided plane at the specified coordinate.  The
	 * 2D coordinate on the plane is arbitrarily selected.
	 *
	 * @param plane Description of the plane
	 * @param x     2D coordinate on the plane, x-axis
	 * @param y     2D coordinate on the plane, y-axis
	 * @return Point on the plane
	 */
	public static Point3D_F64 createPt(PlaneNormal3D_F64 plane, double x, double y) {
		Point3D_F64 p = new Point3D_F64();
		p.set(x, y, 0);

		Vector3D_F64 v = new Vector3D_F64(0, 0, 1);
		Vector3D_F64 cross = v.cross(plane.n);
		if (Math.abs(cross.norm()) < 1e-8) {
			cross.set(0, 0, 1);
		} else {
			cross.normalize();
		}

		double angle = v.dot(plane.n);
		angle = Math.acos(angle / (plane.n.norm()));

		Rodrigues_F64 rod = new Rodrigues_F64(angle, cross);
		DenseMatrix64F R = ConvertRotation3D_F64.rodriguesToMatrix(rod, null);

		GeometryMath_F64.mult(R, p, p);
		p.x += plane.p.x;
		p.y += plane.p.y;
		p.z += plane.p.z;

		return p;
	}

	/**
	 * Creates a 3D point on the surface of the provided cylinder at the specified coordinate
	 *
	 * @param cylinder Description of the cylinder
	 * @param z        z-coordinate along cylinder's axis
	 * @param theta    angular coordinate in radians
	 * @return Point on the sphere
	 */
	public static Point3D_F64 createPt(Cylinder3D_F64 cylinder, double z, double theta) {
		Point3D_F64 p = new Point3D_F64();
		p.x = cylinder.radius * Math.cos(theta);
		p.y = cylinder.radius * Math.sin(theta);
		p.z = z;

		Vector3D_F64 axisZ = new Vector3D_F64(0, 0, 1);
		Vector3D_F64 cross = axisZ.cross(cylinder.line.slope);
		if (Math.abs(cross.norm()) < 1e-8) {
			cross.set(0, 0, 1);
		} else {
			cross.normalize();
		}

		double angle = axisZ.dot(cylinder.line.slope);
		angle = Math.acos(angle / (cylinder.line.slope.norm()));

		Rodrigues_F64 rod = new Rodrigues_F64(angle, cross);
		DenseMatrix64F R = ConvertRotation3D_F64.rodriguesToMatrix(rod, null);

		GeometryMath_F64.mult(R, p, p);
		p.x += cylinder.line.p.x;
		p.y += cylinder.line.p.y;
		p.z += cylinder.line.p.z;

		return p;
	}
}
