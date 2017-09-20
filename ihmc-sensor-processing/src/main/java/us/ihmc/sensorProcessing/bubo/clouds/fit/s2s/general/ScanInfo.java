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

import georegression.struct.point.Point2D_F64;


/**
 * Contains information of a LRF scan.  The measured ranges as well as information derived
 * from the scan is stored.
 *
 * @author Peter Abeles
 */
public class ScanInfo {
	// location of points in 2D cartesian space
	public Point2D_F64 pts[];
	// if the points are "visible"
	public boolean vis[];
	// raw range measurements
	public double range[];
	// the angle of each measurement
	// this changes when a scan is projected onto a new view point
	public double theta[];

	public ScanInfo(int N) {
		pts = new Point2D_F64[N];
		vis = new boolean[N];
		range = new double[N];
		theta = new double[N];

		for (int i = 0; i < N; i++) {
			pts[i] = new Point2D_F64();
		}
	}
}
