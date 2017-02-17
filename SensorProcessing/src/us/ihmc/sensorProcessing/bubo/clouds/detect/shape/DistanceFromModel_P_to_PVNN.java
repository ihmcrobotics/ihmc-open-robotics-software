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

import java.util.ArrayList;
import java.util.List;

import org.ddogleg.fitting.modelset.DistanceFromModel;

import georegression.struct.point.Point3D_F64;
import us.ihmc.sensorProcessing.bubo.clouds.detect.alg.PointVectorNN;

/**
 * Converts point from {@link us.ihmc.sensorProcessing.bubo.clouds.detect.alg.PointVectorNN} into {@link Point3D_F64} for distance calculations.
 *
 * @author Peter Abeles
 */
public class DistanceFromModel_P_to_PVNN<Model> implements DistanceFromModel<Model, PointVectorNN> {

	DistanceFromModel<Model, Point3D_F64> alg;

	List<Point3D_F64> points = new ArrayList<Point3D_F64>();

	public DistanceFromModel_P_to_PVNN(DistanceFromModel<Model, Point3D_F64> alg) {
		this.alg = alg;
	}

	@Override
	public void setModel(Model model) {
		alg.setModel(model);
	}

	@Override
	public double computeDistance(PointVectorNN pt) {
		return alg.computeDistance(pt.p);
	}

	@Override
	public void computeDistance(List<PointVectorNN> pointVectors, double[] distance) {

		points.clear();
		for (int i = 0; i < pointVectors.size(); i++) {
			PointVectorNN p = pointVectors.get(i);
			points.add(p.p);
		}

		alg.computeDistance(points, distance);
	}
}
