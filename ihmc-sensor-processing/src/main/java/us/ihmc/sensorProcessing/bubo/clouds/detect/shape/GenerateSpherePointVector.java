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

import georegression.metric.ClosestPoint3D_F64;
import georegression.struct.line.LineParametric3D_F64;
import georegression.struct.point.Vector3D_F64;
import georegression.struct.shapes.Sphere3D_F64;
import us.ihmc.sensorProcessing.bubo.clouds.detect.alg.ModelGeneratorCheck;
import us.ihmc.sensorProcessing.bubo.clouds.detect.alg.PointVectorNN;

/**
 * Sphere estimation for use in {@link us.ihmc.sensorProcessing.bubo.clouds.detect.alg.PointCloudShapeDetectionSchnabel2007}.  The sphere is estimated
 * using two points and their normal vectors. Third point is used to check the solution.
 *
 * @author Peter Abeles
 */
public class GenerateSpherePointVector implements ModelGeneratorCheck<Sphere3D_F64, PointVectorNN> {

	// tolerance cos(angle) for vector normals
	private double tolCosine;
	// tolerance for each point from the sphere
	private double tolDistance;

	// storage for vector from center to a point
	private Vector3D_F64 n = new Vector3D_F64();

	// line defined by two lines.  used to find sphere center
	private LineParametric3D_F64 lineA = new LineParametric3D_F64(false);
	private LineParametric3D_F64 lineB = new LineParametric3D_F64(false);

	// used to accept or reject a model
	private CheckShapeParameters<Sphere3D_F64> check = new CheckShapeAcceptAll<Sphere3D_F64>();

	public GenerateSpherePointVector(double tolAngle, double tolDistance) {
		this.tolCosine = Math.cos(tolAngle);
		this.tolDistance = tolDistance;
	}

	@Override
	public void setCheck(CheckShapeParameters<Sphere3D_F64> check) {
		this.check = check;
	}

	@Override
	public boolean generate(List<PointVectorNN> dataSet, Sphere3D_F64 output) {

		PointVectorNN pa = dataSet.get(0);
		PointVectorNN pb = dataSet.get(1);
		PointVectorNN pc = dataSet.get(2);

		lineA.p = pa.p;
		lineA.slope = pa.normal;

		lineB.p = pb.p;
		lineB.slope = pb.normal;

		// All points and their normal pass through the sphere's center
		ClosestPoint3D_F64.closestPoint(lineA, lineB, output.center);

		double ra = output.center.distance(pa.p);
		double rb = output.center.distance(pb.p);
		double rc = output.center.distance(pc.p);

		// radius is set to average distance
		output.radius = (ra + rb) / 2.0;

		if (!check.valid(output))
			return false;

		// check the solution
		if (Math.abs(ra - output.radius) > tolDistance)
			return false;
		if (Math.abs(rb - output.radius) > tolDistance)
			return false;
		if (Math.abs(rc - output.radius) > tolDistance)
			return false;

		return checkAngles(output, pa, pb, pc);
	}

	protected final boolean checkAngles(Sphere3D_F64 output, PointVectorNN pa, PointVectorNN pb, PointVectorNN pc) {
		n.set(pa.p.x - output.center.x, pa.p.y - output.center.y, pa.p.z - output.center.z);
		n.normalize();

		if (Math.abs(n.dot(pa.normal)) < tolCosine)
			return false;

		n.set(pb.p.x - output.center.x, pb.p.y - output.center.y, pb.p.z - output.center.z);
		n.normalize();

		if (Math.abs(n.dot(pb.normal)) < tolCosine)
			return false;

		n.set(pc.p.x - output.center.x, pc.p.y - output.center.y, pc.p.z - output.center.z);
		n.normalize();

		if (Math.abs(n.dot(pc.normal)) < tolCosine)
			return false;

		return true;
	}

	@Override
	public int getMinimumPoints() {
		return 3;
	}
}
