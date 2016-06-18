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

package us.ihmc.sensorProcessing.bubo.clouds.fit.s2s;

import georegression.struct.point.Point2D_F64;
import us.ihmc.sensorProcessing.bubo.clouds.fit.s2s.general.GeneralizedScanToScan;
import us.ihmc.sensorProcessing.bubo.clouds.fit.s2s.general.LocalAssociateDiscrete;
import us.ihmc.sensorProcessing.bubo.clouds.fit.s2s.general.ScanInfo;
import us.ihmc.sensorProcessing.bubo.desc.sensors.lrf2d.Lrf2dParam;
import us.ihmc.sensorProcessing.bubo.struct.StoppingCondition;


/**
 * <p>
 * Implementation of {@link bubo.clouds.fit.algs.IterativeClosestPoint ICP} which has been specialized for scans from a 2D
 * laser rangefinder (LRF).  Potential associations are only considered in a window around a
 * scan based on its index.  No interpolation is performed.
 * </p>
 *
 * @author Peter Abeles
 */
public class Lrf2dScanToScan_LocalICP extends GeneralizedScanToScan {

	// code for associating points
	private DistanceAssociate assoc;

	/**
	 * Constructor.
	 *
	 * @param stop              Optimization stopping condition.
	 * @param associationRadius How many indexes around the current
	 * @param maxSeparation     Maximum distance two points can be for them to be associated.
	 */
	public Lrf2dScanToScan_LocalICP(StoppingCondition stop, int associationRadius, double maxSeparation) {
		super(stop);

		assoc = new DistanceAssociate(associationRadius, maxSeparation * maxSeparation);

	}

	@Override
	public void setSensorParam(Lrf2dParam param) {
		super.setSensorParam(param);
		assoc.setParam(param);
	}

	@Override
	public int totalScansMatched() {
		return assoc.getListSource().size();
	}

	@Override
	protected void estimateAndApplyMotion( ScanInfo scanSrc , EstimationResults results) {
		results.srcToDst.set(computeMotion(assoc));
		applyMotion(results.srcToDst,scanSrc);
		results.meanSqError = computeMeanSqError(assoc);
	}

	/**
	 * Associates points based on cartesian distance.
	 */
	protected class DistanceAssociate extends LocalAssociateDiscrete {
		Point2D_F64 srcPt;

		protected DistanceAssociate(int searchNeighborhood, double maxSeparation) {
			super(searchNeighborhood, maxSeparation);
		}

		@Override
		public void setTarget(int index) {
			srcPt = scanSrc.pts[index];
		}

		@Override
		public double distToTarget(int index) {
			return scanDst.pts[index].distance2(srcPt);
		}

	}
}
