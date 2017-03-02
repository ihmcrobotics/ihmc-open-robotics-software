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

import java.util.ArrayList;
import java.util.List;

import org.ddogleg.fitting.modelset.ModelFitter;

import georegression.struct.point.Point3D_F64;

/**
 * Converts a {@link Point3D_F64} based {@link ModelFitter} into a {@link PointVectorNN} based one.
 *
 * @author Peter Abeles
 */
public class ModelFitter_P_to_PVNN<Model> implements ModelFitter<Model, PointVectorNN> {

	private ModelFitter<Model, Point3D_F64> model;

	private List<Point3D_F64> points = new ArrayList<Point3D_F64>();

	public ModelFitter_P_to_PVNN(ModelFitter<Model, Point3D_F64> model) {
		this.model = model;
	}

	@Override
	public boolean fitModel(List<PointVectorNN> dataSet, Model initial, Model found) {
		points.clear();
		for (int i = 0; i < dataSet.size(); i++) {
			PointVectorNN p = dataSet.get(i);
			points.add(p.p);
		}

		return model.fitModel(points, initial, found);
	}
}
