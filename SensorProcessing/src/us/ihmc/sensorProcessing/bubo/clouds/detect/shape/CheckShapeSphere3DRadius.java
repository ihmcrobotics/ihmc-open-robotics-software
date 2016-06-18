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

import georegression.struct.shapes.Sphere3D_F64;

/**
 * Limits how large of a radius it will accept for a sphere. Returns true if radius <= maxRadius.
 *
 * @author Peter Abeles
 */
public class CheckShapeSphere3DRadius implements CheckShapeParameters<Sphere3D_F64> {

	double maxRadius;

	public CheckShapeSphere3DRadius(double maxRadius) {
		this.maxRadius = maxRadius;
	}

	@Override
	public boolean valid(Sphere3D_F64 param) {
		return param.radius <= maxRadius;
	}
}
