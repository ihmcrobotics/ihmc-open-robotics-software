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

package us.ihmc.sensorProcessing.bubo.desc.sensors.lrf2d;

import georegression.struct.point.Point2D_F64;
import us.ihmc.sensorProcessing.bubo.desc.IntrinsicParameters;

/**
 * Description of basic parameters that describe a typical planar LADAR, such as a SICK or Hokuyo.
 *
 * @author Peter Abeles
 */
// TODO should units be specified?
public class Lrf2dParam implements IntrinsicParameters {
	/**
	 * Type of scanner
	 */
	private String sensorType;
	/**
	 * The angle in radians that the scanner starts at
	 */
	private double startAngle;
	/**
	 * Angle which the laser sweeps from startAngle.  Positive indicates CCW rotation and negative CW rotation.
	 */
	private double sweepAngle;
	/**
	 * How many scans/range measurements are there in a single sweep
	 */
	private int numberOfScans;
	/**
	 * How far can the LADAR see objects.  A value > maxRange is considered to be a miss.
	 */
	private double maxRange;
	/**
	 * The laser's aperture.  How "wide" the beam is.
	 */
	private double beamAperture;
	/**
	 * Accuracy of the range measurement
	 */
	private double rangeAccuracy;

	/**
	 * @param sensorType    Type of scanner
	 * @param startAngle    Angle in radians that the scan starts at
	 * @param sweepAngle    How wide of an area is scanned.
	 * @param numberOfScans Number of scans performed in a single sweep
	 * @param maxRange      The maximum range of the sensor.
	 * @param beamAperture  How wide the beam is.  Angular units.
	 * @param rangeAccuracy How accurate the range measurement is.
	 */
	public Lrf2dParam(String sensorType,
					  double startAngle, double sweepAngle, int numberOfScans,
					  double maxRange, double beamAperture,
					  double rangeAccuracy) {
		this.sensorType = sensorType;
		this.startAngle = startAngle;
		this.sweepAngle = sweepAngle;
		this.numberOfScans = numberOfScans;
		this.maxRange = maxRange;
		this.beamAperture = beamAperture;
		this.rangeAccuracy = rangeAccuracy;
	}

	public Lrf2dParam() {
	}

	/**
	 * Checks to see if the provided range measurement is a valid range.
	 *
	 * @param range range measurement
	 * @return true if valid.
	 */
	public boolean isValidRange(double range) {
		if (Double.isNaN(range))
			return false;

		if (range >= maxRange)
			return false;

		return true;
	}

	/**
	 * Computes the end point of a scan for the specified index and range measurement.
	 */
	public Point2D_F64 computeLocation( int index , double range , Point2D_F64 output ) {
		if( output == null )
			output = new Point2D_F64();

		double theta = computeAngle(index);

		output.x = Math.cos(theta)*range;
		output.y = Math.sin(theta)*range;

		return output;
	}

	@Override
	public boolean isConstant() {
		return true;
	}

	public String getSensorType() {
		return sensorType;
	}

	public void setSensorType(String sensorType) {
		this.sensorType = sensorType;
	}

	public double getStartAngle() {
		return startAngle;
	}

	public void setStartAngle(double startAngle) {
		this.startAngle = startAngle;
	}

	public double getAngleIncrement() {
		return sweepAngle / (numberOfScans-1);
	}

	public double getSweepAngle() {
		return sweepAngle;
	}

	public void setSweepAngle(double sweepAngle) {
		this.sweepAngle = sweepAngle;
	}

	public int getNumberOfScans() {
		return numberOfScans;
	}

	public void setNumberOfScans(int numberOfScans) {
		this.numberOfScans = numberOfScans;
	}

	public double getMaxRange() {
		return maxRange;
	}

	public void setMaxRange(double maxRange) {
		this.maxRange = maxRange;
	}

	public double getBeamAperture() {
		return beamAperture;
	}

	public void setBeamAperture(double beamAperture) {
		this.beamAperture = beamAperture;
	}

	public double getRangeAccuracy() {
		return rangeAccuracy;
	}

	public void setRangeAccuracy(double rangeAccuracy) {
		this.rangeAccuracy = rangeAccuracy;
	}

	/**
	 * Returns the angle of the scan at the specified index.
	 */
	public double computeAngle(int index) {
		return startAngle + sweepAngle * index / (double) (numberOfScans-1);
	}
}
