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

import georegression.struct.point.Point2D_F64;
import us.ihmc.sensorProcessing.bubo.desc.sensors.lrf2d.Lrf2dParam;


/**
 * Association is done to the LRF sample points with no interpolation.  Child classes
 * are given the index of the scan being considered.
 *
 * @author Peter Abeles
 */
public abstract class LocalAssociateDiscrete implements AssociateLrfMeas {
	//  LRF scan information.
	protected ScanInfo scanSrc;
	protected ScanInfo scanDst;
	// description of the sensor
	private Lrf2dParam param;
	// list of associated points
	private List<Point2D_F64> srcPts = new ArrayList<Point2D_F64>();
	private List<Point2D_F64> dstPts = new ArrayList<Point2D_F64>();

	// how many indexes away from the target index will it search
	private int searchNeighborhood;
	// the maximum distance away two points can be to be associated
	private double maxSeparation;

	protected LocalAssociateDiscrete(int searchNeighborhood,
									 double maxSeparation) {
		this.searchNeighborhood = searchNeighborhood;
		this.maxSeparation = maxSeparation;
	}

	public void setParam(Lrf2dParam param) {
		this.param = param;
	}

	@Override
	public void associate(ScanInfo scanSrc, ScanInfo scanDst) {

		this.scanSrc = scanSrc;
		this.scanDst = scanDst;

		srcPts.clear();
		dstPts.clear();

		for (int i = 0; i < param.getNumberOfScans(); i++) {
			if (!scanSrc.vis[i]) {
				continue;
			}

			int bestIndex = findBestMatch(scanDst, i);

			if (bestIndex != -1 ) {
				srcPts.add(scanSrc.pts[i]);
				dstPts.add(scanDst.pts[bestIndex]);
			}
		}
	}

	private int findBestMatch(ScanInfo scan, int target) {
		int min = target - searchNeighborhood;
		int max = target + searchNeighborhood;
		if (min < 0) min = 0;
		if (max > param.getNumberOfScans()) max = param.getNumberOfScans();

		int bestIndex = -1;
		double bestDistance = maxSeparation;

		setTarget(target);

		for (int j = min; j < max; j++) {
			if (!scan.vis[j])
				continue;

			double dist = distToTarget(j);

			if (dist < bestDistance) {
				bestDistance = dist;
				bestIndex = j;
			}
		}
		return bestIndex;
	}

	@Override
	public List<Point2D_F64> getListSource() {
		return srcPts;
	}

	@Override
	public List<Point2D_F64> getListDestination() {
		return dstPts;
	}

	/**
	 * Specifies which measurement in the match scan that the distance is being measured against.
	 *
	 * @param index which point from the list
	 */
	public abstract void setTarget( int index);

	/**
	 * Distance from reference to the specified index
	 */
	public abstract double distToTarget( int index);
}
