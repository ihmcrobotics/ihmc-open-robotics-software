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

import java.util.List;

import georegression.fitting.se.MotionSe2PointSVD_F64;
import georegression.metric.UtilAngle;
import georegression.struct.point.Point2D_F64;
import georegression.struct.se.Se2_F64;
import georegression.transform.se.SePointOps_F64;
import us.ihmc.sensorProcessing.bubo.clouds.fit.Lrf2dScanToScan;
import us.ihmc.sensorProcessing.bubo.desc.sensors.lrf2d.Lrf2dParam;
import us.ihmc.sensorProcessing.bubo.desc.sensors.lrf2d.Lrf2dPrecomputedTrig;
import us.ihmc.sensorProcessing.bubo.struct.StoppingCondition;


/**
 * <p>
 * Base class for scan-to-scan matching which automatically precomputes information which most 2D-LRF scan-to-scan
 * matching algorithms would need.  Simplifies the creating of algorithms at the cost of some performance.  Allows
 * for global or local association as well as interpolation.
 * </p>
 *
 * @author Peter Abeles
 */
public abstract class GeneralizedScanToScan implements Lrf2dScanToScan {

	// description of the sensor
	protected Lrf2dParam param;
	// decides when to stop iterating
	protected StoppingCondition stop;
	// original scan measurements
	protected double rangesSrc[];
	// various bits of information related to each scan
	protected ScanInfo scanDst; // the reference scan being registered to
	protected ScanInfo scanSrc; // the scan which is being registered
	// speeds up calculations
	private Lrf2dPrecomputedTrig lrf2pt;
	// the found total motion
	private Se2_F64 motion = new Se2_F64();
	// the final error
	private double foundError;

	// given associated points computes rigid body motion
	private MotionSe2PointSVD_F64 motionAlg = new MotionSe2PointSVD_F64();

	EstimationResults results = new EstimationResults();

	public GeneralizedScanToScan(StoppingCondition stop) {

		this.stop = stop;
	}

	@Override
	public void setSensorParam(Lrf2dParam param) {
		this.param = param;
		this.lrf2pt = new Lrf2dPrecomputedTrig(param);

		scanDst = new ScanInfo(param.getNumberOfScans());
		scanSrc = new ScanInfo(param.getNumberOfScans());
		rangesSrc = new double[param.getNumberOfScans()];
	}

	@Override
	public Se2_F64 getSourceToDestination() {
		return motion;
	}

	@Override
	public void setDestination(double[] scan) {
		System.arraycopy(scan, 0, scanDst.range, 0, param.getNumberOfScans());
	}

	@Override
	public void setSource(double[] scan) {
		System.arraycopy(scan, 0, scanSrc.range, 0, param.getNumberOfScans());
	}

	public void computeScanEndPoint(double scan[], Point2D_F64 pts[]) {
		final int N = param.getNumberOfScans();
		for (int i = 0; i < N; i++) {
			double r = scan[i];

			if ( param.isValidRange(r)) {
				lrf2pt.computeEndPoint(i, r, pts[i]);
			}
		}
	}

	@Override
	public void assignSourceToDestination() {
		ScanInfo temp = scanDst;
		scanDst = scanSrc;
		scanSrc = temp;
	}

	@Override
	public boolean process(Se2_F64 hintSrcToDst) {
		// save the original ranges
		System.arraycopy(scanSrc.range, 0, rangesSrc, 0, param.getNumberOfScans());
		// find the obstacle location
		computeScanEndPoint(scanSrc.range, scanSrc.pts);
		computeScanEndPoint(scanDst.range, scanDst.pts);

		// apply the hint, if any
		if (hintSrcToDst != null) {
			applyMotion(hintSrcToDst, scanSrc);
			motion.set(hintSrcToDst);
		} else
			motion.set(0, 0, 0);

		setVisibleByRange(scanDst); // todo could just do this when end point is computed

		stop.reset();
		while (true) {
			// compute the angle of each point in the current view
			projectScan(rangesSrc, scanSrc);

			// angle based visibility test
			checkVisibleByDeltaAngle(scanSrc);

			// find the motion which minimizes the error between the two scans
			// and applies it to the points in the source scan
			estimateAndApplyMotion(scanSrc,results);
			foundError = results.meanSqError;

			// increment
			motion = motion.concat(results.srcToDst, null);

			if (stop.isFinished(foundError))
				break;
		}

		// undo the recomputed ranges
		System.arraycopy(rangesSrc, 0, scanSrc.range, 0, param.getNumberOfScans());

		return true;
	}

	/**
	 * Inner function for estimating the motion between the current incarnations of the scans
	 */
	protected abstract void estimateAndApplyMotion( ScanInfo scanSrc , EstimationResults results );

	/**
	 * Sets visibility depending on the measured range being less than the max range.
	 */
	private void setVisibleByRange(ScanInfo info) {
		final int N = param.getNumberOfScans();
		final double maxRange = param.getMaxRange();
		for (int i = 0; i < N; i++) {
			double r = info.range[i];
			scanDst.vis[i] = r > 0 && info.range[i] <= maxRange;
		}
	}

	/**
	 * Computes the angle of each scan and flags visible based on measured range
	 */
	protected void projectScan(double measuredRange[], ScanInfo info) {
		final int N = param.getNumberOfScans();
		final double maxRange = param.getMaxRange();

		for (int i = 0; i < N; i++) {
			double r = measuredRange[i];

			if (r > 0 && r <= maxRange) {
				Point2D_F64 p = info.pts[i];
				info.theta[i] = Math.atan2(p.y, p.x);
				info.range[i] = p.norm();
				info.vis[i] = true;
			} else {
				info.range[i] = 0;
				info.vis[i] = false;
			}
		}
	}

	/**
	 * <p>
	 * Each angle should always be increasing or decreasing.  If this order is broken
	 * at any point then the point is not visible from the current point of view.
	 * </p>
	 * <p/>
	 * <p>
	 * See section 3.2 of: Feng Lu and Evangelos Milios, "Robot Pose Estimation in Unknown Environments by Matching 2D Range Scans"
	 * Journal of Intelligent and Robotics Systems, 18: 249-275, 1997.
	 * </p>
	 */
	protected void checkVisibleByDeltaAngle(ScanInfo info) {
		final int N = param.getNumberOfScans();
		boolean increasing = param.getAngleIncrement() > 0;
		double ang[] = info.theta;

		for (int i = 1; i < N; i++) {
			if (!info.vis[i])
				continue;

			double deltaAng = UtilAngle.minus(ang[i], ang[i - 1]);
			if (increasing) {
				if (deltaAng < 0)
					info.vis[i] = false;
			} else {
				if (deltaAng > 0)
					info.vis[i] = false;
			}
		}
	}

	/**
	 * Computes motion by associating points using the provided distance function then
	 * computing the Se2_F64 transform.
	 *
	 * @return found motion
	 */
	protected Se2_F64 computeMotion(AssociateLrfMeas assoc) {
		assoc.associate(scanSrc, scanDst);

		List<Point2D_F64> srcPts = assoc.getListSource();
		List<Point2D_F64> dstPts = assoc.getListDestination();

		motionAlg.process(srcPts, dstPts);

		return motionAlg.getTransformSrcToDst();
	}

	protected static void applyMotion(Se2_F64 motion, ScanInfo scan) {
		for (int i = 0; i < scan.pts.length; i++) {
			Point2D_F64 p = scan.pts[i];
			SePointOps_F64.transform(motion, p, p);
		}
	}

	protected static double computeMeanSqError(AssociateLrfMeas associate ) {
		double error = 0;
		List<Point2D_F64> listSrc = associate.getListSource();
		List<Point2D_F64> listDst = associate.getListDestination();

		for (int i = 0; i < listSrc.size(); i++) {
			Point2D_F64 s = listSrc.get(i);
			Point2D_F64 d = listDst.get(i);

			error += s.distance2(d);
		}
		return error / listSrc.size();
	}

	@Override
	public double getError() {
		return foundError;
	}

	protected static class EstimationResults {
		public Se2_F64 srcToDst = new Se2_F64();
		public double meanSqError;
	}
}
