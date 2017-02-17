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
import georegression.struct.point.Point3D_F64;
import georegression.struct.shapes.Cylinder3D_F64;

/**
 * Euclidean distance from a {@link Cylinder3D_F64} for use with {@link us.ihmc.sensorProcessing.bubo.clouds.detect.alg.PointCloudShapeDetectionSchnabel2007}.
 *
 * @author Peter Abeles
 */
public class DistanceCylinderToPoint3D implements DistanceFromModel<Cylinder3D_F64, Point3D_F64> {

	Cylinder3D_F64 model;

	@Override
	public void setModel(Cylinder3D_F64 model) {
		this.model = model;
	}

	@Override
	public double computeDistance(Point3D_F64 pt) {
		return Math.abs(Distance3D_F64.distance(model, pt));
	}

	@Override
	public void computeDistance(List<Point3D_F64> points, double[] distance) {
		for (int i = 0; i < points.size(); i++) {
			distance[i] = Math.abs(Distance3D_F64.distance(model, points.get(i)));
		}
	}
}
