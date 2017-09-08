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
 * Computes a probabilistic function that has gaussian distribution as a function of the range.
 *
 * @author Peter Abeles
 */
public class LineRangeGaussian implements LineRangeProbability {

	// variance of the distribution
	double var;

	// distance from the mean before the PDF becomes essentially zero
	double rZero;
	// distance from the mean until the PDF is at 50% of its peak
	double rHalf;

	// current range measurement
	double rangeMeas;

	/**
	 * Creates a new LineRangeGaussian.
	 *
	 * @param stdev Distributions standard deviation.
	 */
	public LineRangeGaussian(double stdev) {
		var = stdev * stdev;

		// compute the point at which it becomes essentially zero
		rZero = Math.sqrt(-Math.log(0.01) * var);
		// find how far away it is until it reaches 50%
		rHalf = Math.sqrt(-Math.log(0.5) * var);

	}


	@Override
	public double lineExtension() {
		return rHalf;
	}

	@Override
	public void setRangeMeasurement(double meas) {
		this.rangeMeas = meas;
	}

	@Override
	public float computeProbability(double dist) {
		if (dist < rangeMeas - rZero) {
			return 0.0f;
		} else if (dist < rangeMeas + rHalf) {
			double r = rangeMeas - dist;

			// this will produce a number between minProb and 1.0
			return (float) Math.exp(-r * r / (2.0 * var));
		} else {
			return 0.5f;
		}
	}
}
