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

import georegression.struct.point.Point2D_F64;


/**
 * Interface for associating LRF scans inside of {@link GeneralizedScanToScan}.  Associated points in each
 * scan is returned in two ordered list where elements with the same index are associated.
 *
 * @author Peter Abeles
 */
public interface AssociateLrfMeas {

	/**
	 * Associates LRF measurements in the two scans.
	 *
	 * @param scanSrc Source scan.
	 * @param scanDst Destination scan.
	 */
	public void associate(ScanInfo scanSrc, ScanInfo scanDst);

	public List<Point2D_F64> getListSource();

	public List<Point2D_F64> getListDestination();

}
