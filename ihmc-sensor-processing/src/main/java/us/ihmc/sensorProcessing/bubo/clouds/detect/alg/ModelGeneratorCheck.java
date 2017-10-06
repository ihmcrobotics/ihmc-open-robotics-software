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

import org.ddogleg.fitting.modelset.ModelGenerator;

import us.ihmc.sensorProcessing.bubo.clouds.detect.shape.CheckShapeParameters;

/**
 * Model generator which allows an optional check to be added to the parameters.
 *
 * @author Peter Abeles
 */
public interface ModelGeneratorCheck<Model, Point> extends ModelGenerator<Model, Point> {

	/**
	 * Specify a check that it should perform on the shape after parameters have been set
	 */
	public void setCheck(CheckShapeParameters<Model> check);
}
