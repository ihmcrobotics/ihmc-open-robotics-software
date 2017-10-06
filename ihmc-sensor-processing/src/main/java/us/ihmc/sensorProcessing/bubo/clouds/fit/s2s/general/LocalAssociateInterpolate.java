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

package us.ihmc.sensorProcessing.bubo.clouds.fit.s2s.general;

import java.util.ArrayList;
import java.util.List;

import org.ddogleg.struct.FastQueue;

import georegression.struct.point.Point2D_F64;
import us.ihmc.sensorProcessing.bubo.desc.sensors.lrf2d.Lrf2dParam;


/**
 * Instead of associating at the measured discrete points, association is performed using
 * a continuous function created using interpolation.
 * The type of interpolation is specified by the extending class.  Interpolation allows
 * the LRF to be registered against a smoother function than if the raw discrete points
 * are used and might be more physically realistic in some situations.
 *
 * @author Peter Abeles
 */
public abstract class LocalAssociateInterpolate implements AssociateLrfMeas {

	// description of the sensor
	protected Lrf2dParam param;
	// information on the tow scans
	protected ScanInfo scanMatch;
	protected ScanInfo scanRef;
	// list of associated points
	private List<Point2D_F64> srcPts = new ArrayList<Point2D_F64>();
	private FastQueue<Point2D_F64> dstPts = new FastQueue<Point2D_F64>(Point2D_F64.class,true);
	// how many radians around will it search for the best association point
	private double searchNeighborhood;
	// the maximum allowed distance between two associated points
	private double maxSeparation;
	// how many radians it will sample around the target angle
	private double samplePeriod;

	public LocalAssociateInterpolate(Lrf2dParam param,
									 double searchNeighborhood,
									 double maxSeparation,
									 double samplePeriod) {
		if (samplePeriod > searchNeighborhood) {
			throw new RuntimeException("Sample Period is more than the search neighborhood.  Probably a bug");
		}
		this.param = param;
		this.searchNeighborhood = searchNeighborhood;
		this.maxSeparation = maxSeparation;
		this.samplePeriod = samplePeriod;
	}

	public LocalAssociateInterpolate() {
	}

	@Override
	public void associate(ScanInfo scanSrc, ScanInfo scanDst) {
		if (samplePeriod == 0)
			throw new IllegalArgumentException("Must initialize the sampling period");

		this.scanMatch = scanSrc;
		this.scanRef = scanDst;

		srcPts.clear();
		dstPts.reset();

		Point2D_F64 best = new Point2D_F64();
		InterpolatedPoint interp = new InterpolatedPoint();

		final int N = param.getNumberOfScans();
		for (int i = 0; i < N; i++) {
			if (!scanSrc.vis[i]) {
				continue;
			}

			// TODO Change the algorithm a bit
			// Define search radius using an integer neighborhood
			// Interpolate using a local curve with the middle index given to it

			// define the local area being search
			double startTheta = scanSrc.theta[i] - searchNeighborhood;
			int numSamples = (int) Math.ceil(2.0 * searchNeighborhood / samplePeriod);

			setTarget(i);

			double bestDist = Double.MAX_VALUE;

			// TODO this won't handle a 360 degree sensor
			for (int j = 0; j <= numSamples; j++) {
				interp.angle = startTheta + samplePeriod * j;
				if (!interpolate(interp))
					continue;

				double dist = distToTarget(interp);
				if (dist < bestDist) {
					best.set(interp.point);
					bestDist = dist;
				}
			}

			if (bestDist < maxSeparation) {
				srcPts.add(scanSrc.pts[i]);
				dstPts.grow().set(best);
			}
		}
	}

	public void setParam(Lrf2dParam param) {
		this.param = param;
	}

	public void setSearchNeighborhood(double searchNeighborhood) {
		this.searchNeighborhood = searchNeighborhood;
	}

	public void setMaxSeparation(double maxSeparation) {
		this.maxSeparation = maxSeparation;
	}

	@Override
	public List<Point2D_F64> getListSource() {
		return srcPts;
	}

	@Override
	public List<Point2D_F64> getListDestination() {
		return dstPts.toList();
	}

	public void setSamplePeriod(double samplePeriod) {
		this.samplePeriod = samplePeriod;
	}

	/**
	 * Returns the LRF hit at the specified point.  It can be assumed that the requested angles are
	 * monotonically increasing or decreasing depending on the LRF's definition.
	 *
	 * @param point Data structure specifying where it should interpolate and where the interpolation
	 *              is written to.
	 * @return true if there is a point at that location
	 */
	public abstract boolean interpolate(InterpolatedPoint point);

	/**
	 * Specify which measurement in the from scan is the distance being measured against.
	 * This function is called before interpolate() or distToReference() in each cycle.  Thus
	 * any initialization can be done here.
	 */
	public abstract void setTarget(int indexMatch);

	/**
	 * Returns the distance of the specified point from the reference.
	 */
	public abstract double distToTarget(InterpolatedPoint point);

	public static class InterpolatedPoint {
		public Point2D_F64 point = new Point2D_F64();
		public double range;
		public double angle;
	}
}
