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

package us.ihmc.sensorProcessing.bubo.mapping.build.ladar2d;

/**
 * Computes the probability that the true range of an object is at the specified value. A returned
 * value of 0.0 means it is 100% confident that there is no obstacle and 1.0 means there is 100%
 * confidence that there is an obstacle.
 *
 * @author Peter Abeles
 */
public interface LineRangeProbability {

	/**
	 * To extends the range along the map the line is projected so that the distribution can be fully
	 * described.  The distribution beyond the range plus extension are assumed to have a probability
	 * of 0.5.
	 *
	 * @return
	 */
	public double lineExtension();

	/**
	 * Sets the range of the measurement.
	 *
	 * @param meas Measurement's range.
	 */
	public void setRangeMeasurement(double meas);

	/**
	 * Computes the probability that the obstacle's range true value is 'dist'.  Pr(dist| measured range).
	 * For those who are pedantic, the area of integration is the map's cell size.
	 *
	 * @param dist Distance along the line.
	 * @return Probability that this is the true range.  Between 0 and 1.
	 */
	public float computeProbability(double dist);
}
