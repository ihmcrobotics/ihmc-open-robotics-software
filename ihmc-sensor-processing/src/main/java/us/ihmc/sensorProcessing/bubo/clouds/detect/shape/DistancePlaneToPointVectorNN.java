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

import org.ddogleg.fitting.modelset.DistanceFromModel;

import georegression.metric.Distance3D_F64;
import georegression.struct.plane.PlaneGeneral3D_F64;
import georegression.struct.point.Vector3D_F64;
import us.ihmc.sensorProcessing.bubo.clouds.detect.alg.PointVectorNN;

/**
 * Euclidean distance from a {@link PlaneGeneral3D_F64} for use with {@link us.ihmc.sensorProcessing.bubo.clouds.detect.alg.PointCloudShapeDetectionSchnabel2007}.
 *
 * @author Peter Abeles
 */
public class DistancePlaneToPointVectorNN implements DistanceFromModel<PlaneGeneral3D_F64, PointVectorNN> {

	PlaneGeneral3D_F64 model;
	Vector3D_F64 n = new Vector3D_F64();
	// tolerance cos(angle) for vector normals
	private double tolAngleCosine;

	public DistancePlaneToPointVectorNN(double tolAngle) {
		this.tolAngleCosine = Math.cos(tolAngle);
	}

	@Override
	public void setModel(PlaneGeneral3D_F64 model) {
		this.model = model;
		n.set(model.A, model.B, model.C);
		n.normalize();
	}

	@Override
	public double computeDistance(PointVectorNN pv) {


		if (Math.abs(n.dot(pv.normal)) < tolAngleCosine)
			return Double.MAX_VALUE;

		return Math.abs(Distance3D_F64.distance(model, pv.p));
	}

	@Override
	public void computeDistance(List<PointVectorNN> points, double[] distance) {
		for (int i = 0; i < points.size(); i++) {
			distance[i] = computeDistance(points.get(i));
		}
	}
}
