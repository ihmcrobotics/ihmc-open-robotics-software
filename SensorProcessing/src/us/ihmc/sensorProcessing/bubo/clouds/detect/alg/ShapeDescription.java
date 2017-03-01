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

import java.util.Stack;

import org.ddogleg.fitting.modelset.DistanceFromModel;
import org.ddogleg.fitting.modelset.ModelCodec;
import org.ddogleg.fitting.modelset.ModelFitter;
import org.ddogleg.fitting.modelset.ModelManager;

import us.ihmc.sensorProcessing.bubo.clouds.detect.shape.CheckShapeParameters;

/**
 * Mathematically describes a shape, how to estimate the shape, how to compute the distance a point
 * is from the shape, and how far away a point can be from the shape to be considered part of it.
 *
 * @author Peter Abeles
 */
public class ShapeDescription<Model> {

	/**
	 * how close a point needs to be considered part of the model
	 */
	public double thresholdFit;
	/**
	 * Creates new instances and copies
	 */
	public ModelManager<Model> modelManager;
	/**
	 * generates an initial model given a set of points
	 */
	public ModelGeneratorCheck<Model, PointVectorNN> modelGenerator;
	/**
	 * Used to refine an initial model estimate when given an initial estimate
	 */
	public ModelFitter<Model, PointVectorNN> modelFitter;
	/**
	 * computes the distance a point is from the model
	 */
	public DistanceFromModel<Model, PointVectorNN> modelDistance;
	/**
	 * Specifies an optional check which can be used to remove physically impossible shapes.  If null then
	 * no check will be performed
	 */
	public CheckShapeParameters<Model> modelCheck;
	/**
	 * Converts the model parameter into double[]
	 */
	public ModelCodec<Model> codec;

	// storage for models so that they can be recycled
	protected Stack<Model> used = new Stack<Model>();
	protected Stack<Model> unused = new Stack<Model>();

	public void reset() {
		unused.addAll(used);
		used.clear();
	}

	/**
	 * Takes the most recently created model and removes it from the used list and puts it into the unused list
	 */
	public void recycleTail() {
		Model m = used.remove(used.size() - 1);
		unused.add(m);
	}

	public Model createModel() {
		Model m;
		if (unused.isEmpty()) {
			m = modelManager.createModelInstance();
		} else {
			m = unused.pop();
		}
		used.add(m);
		return m;
	}
}
