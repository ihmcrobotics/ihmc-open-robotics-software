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

package us.ihmc.sensorProcessing.bubo.clouds.detect.wrapper;

import java.util.List;

import org.ddogleg.fitting.modelset.ModelFitter;

import georegression.fitting.plane.FitPlane3D_F64;
import georegression.geometry.UtilPlane3D_F64;
import georegression.struct.plane.PlaneGeneral3D_F64;
import georegression.struct.plane.PlaneNormal3D_F64;
import georegression.struct.point.Point3D_F64;

/**
 * The optimal plane can be estimated exactly using SVD.  This class is a wrapper which allows it
 * to be used inside a {@link ModelFitter}.
 *
 * @author Peter Abeles
 */
public class PlaneGeneralSvd_to_ModelFitter implements ModelFitter<PlaneGeneral3D_F64, Point3D_F64> {

	FitPlane3D_F64 alg = new FitPlane3D_F64();

	PlaneNormal3D_F64 planeNorm = new PlaneNormal3D_F64();

	@Override
	public boolean fitModel(List<Point3D_F64> dataSet, PlaneGeneral3D_F64 initial, PlaneGeneral3D_F64 found) {

		if (!alg.svd(dataSet, planeNorm.p, planeNorm.n))
			return false;

		UtilPlane3D_F64.convert(planeNorm, found);

		return true;
	}
}
