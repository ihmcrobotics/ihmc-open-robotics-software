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

import java.util.List;

import org.ddogleg.fitting.modelset.DistanceFromModel;
import org.ddogleg.fitting.modelset.ransac.RansacMulti;
import org.ddogleg.struct.FastQueue;

/**
 * Customized version of {@link RansacMulti}.  Instead of finding the inlier set using the sampling set it uses the
 * global set.  This is done by traversing through the nearest-neighbor graph, starting with the
 * points used to generate the model, and finding points which are close to the shape.
 * <p/>
 * Deviations from paper:
 * <ul>
 * <li>connected components:  Does not use bitmap technique.  Uses nearest neighbor information instead.</li>
 * <li>exit iteration: When no better model has been found after N iterations</li>
 * <ul>
 *
 * @author Peter Abeles
 */
public class RansacShapeDetection extends RansacMulti<PointVectorNN> {

	// finds the set of points which match the model
	private FindMatchSetPointVectorNN matchFinder;

	// The maximum number of iterations is set to the current number of iterations plus this number when
	// a better model is found
	private int maxExtension;

	public RansacShapeDetection(long randSeed, int maxExtension,
								FindMatchSetPointVectorNN matchFinder,
								List<ObjectType> objectTypes) {
		super(randSeed, -1, objectTypes, PointVectorNN.class);
		this.maxExtension = maxExtension;
		this.matchFinder = matchFinder;
	}

	public void reset() {
	}

	@Override
	protected void initialize(List<PointVectorNN> dataSet) {
		super.initialize(dataSet);
		maxIterations = maxExtension * 2;
	}

	/**
	 * Finds the match set by searching the nearest-neighbor graph of the initialSample set.  Points
	 * are marked with a unique ID for this function call so that it knows which ones it has examined.
	 */
	@Override
	protected <Model> void selectMatchSet(DistanceFromModel<Model, PointVectorNN> modelDistance,
										  double threshold, Model param) {
		candidatePoints.clear();
		matchFinder.setModelDistance(modelDistance);
		matchFinder.selectMatchSet(initialSample.toList(), param, threshold, false, candidatePoints);
	}

	@Override
	protected void setBestModel(Object param) {
		super.setBestModel(param);
		// extend how long it can run for
		maxIterations = Math.max(maxIterations, iteration + maxExtension);
	}

	/**
	 * Provided for testing purposes
	 */
	@Override
	protected List<PointVectorNN> getCandidatePoints() {
		return super.getCandidatePoints();
	}

	@Override
	protected FastQueue<PointVectorNN> getInitialSample() {
		return initialSample;
	}
}
