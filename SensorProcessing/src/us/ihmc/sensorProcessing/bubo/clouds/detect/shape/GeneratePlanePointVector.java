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

package us.ihmc.sensorProcessing.bubo.clouds.detect.shape;

import java.util.List;

import georegression.geometry.GeometryMath_F64;
import georegression.geometry.UtilPlane3D_F64;
import georegression.struct.plane.PlaneGeneral3D_F64;
import georegression.struct.plane.PlaneNormal3D_F64;
import georegression.struct.point.Vector3D_F64;
import us.ihmc.sensorProcessing.bubo.clouds.detect.alg.ModelGeneratorCheck;
import us.ihmc.sensorProcessing.bubo.clouds.detect.alg.PointVectorNN;

/**
 * Plane estimation for use in {@link us.ihmc.sensorProcessing.bubo.clouds.detect.alg.PointCloudShapeDetectionSchnabel2007}.  The equation of the plane is found
 * using 3 points.  It is then validated using the 3 normal vectors.  The normal vectors are accepted
 * if the angle between them and the recently found vector is less than a use specified tolerance.
 *
 * @author Peter Abeles
 */
public class GeneratePlanePointVector implements ModelGeneratorCheck<PlaneGeneral3D_F64, PointVectorNN> {

	// normal found from 3 points
	protected Vector3D_F64 n = new Vector3D_F64();
	// just used for converting formats
	protected PlaneNormal3D_F64 planeNormal = new PlaneNormal3D_F64();
	// tolerance cos(angle) for vector normals
	private double tolCosine;
	// storage for difference of points
	private Vector3D_F64 a = new Vector3D_F64();
	private Vector3D_F64 b = new Vector3D_F64();
	// used to accept or reject a model
	private CheckShapeParameters<PlaneGeneral3D_F64> check = new CheckShapeAcceptAll<PlaneGeneral3D_F64>();

	public GeneratePlanePointVector(double tolAngle) {
		this.tolCosine = Math.cos(tolAngle);
	}

	@Override
	public void setCheck(CheckShapeParameters<PlaneGeneral3D_F64> check) {
		this.check = check;
	}

	@Override
	public boolean generate(List<PointVectorNN> dataSet, PlaneGeneral3D_F64 output) {

		PointVectorNN pa = dataSet.get(0);
		PointVectorNN pb = dataSet.get(1);
		PointVectorNN pc = dataSet.get(2);

		// find the plane's normal vector
		GeometryMath_F64.sub(pa.p, pb.p, a);
		GeometryMath_F64.sub(pa.p, pc.p, b);

		GeometryMath_F64.cross(a, b, n);
		n.normalize();

		if (!checkModel(pa, pb, pc))
			return false;

		// asume the first point is one the plane
		planeNormal.n = n;
		planeNormal.p = pa.p;

		UtilPlane3D_F64.convert(planeNormal, output);

		return check.valid(output);
	}

	protected final boolean checkModel(PointVectorNN pa, PointVectorNN pb, PointVectorNN pc) {
		if (Math.abs(n.dot(pa.normal)) < tolCosine)
			return false;

		if (Math.abs(n.dot(pb.normal)) < tolCosine)
			return false;

		if (Math.abs(n.dot(pc.normal)) < tolCosine)
			return false;

		return true;
	}

	@Override
	public int getMinimumPoints() {
		return 3;
	}
}
