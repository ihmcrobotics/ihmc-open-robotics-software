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

import java.util.ArrayList;
import java.util.List;

import org.ddogleg.struct.FastQueue;
import org.ddogleg.struct.GrowQueue_I32;

/**
 * todo comment
 *
 * @author Peter Abeles
 */
// TODO add shape merging
public class RemoveFalseShapes implements PostProcessShapes {

	List<ShapeDescription> models;

	GrowQueue_I32 cloudToShape = new GrowQueue_I32();
	FastQueue<PixelInfo> shapePixels = new FastQueue<PixelInfo>(PixelInfo.class, true);

	List<FoundShape> goodShapes = new ArrayList<FoundShape>();

	double thresholdDiscard;

	public RemoveFalseShapes(double thresholdDiscard) {
		this.thresholdDiscard = thresholdDiscard;
	}

	@Override
	public void setup(List<ShapeDescription> models, LocalFitShapeNN refine) {
		this.models = models;
	}

	@Override
	public void process(List<FoundShape> input, int cloudSize) {
		goodShapes.clear();

		cloudToShape.resize(cloudSize);
		for (int i = 0; i < cloudToShape.size; i++) {
			cloudToShape.data[i] = -1;
		}

		pruneFalseShapes(input, goodShapes);

	}

	private void mergeSimilarShapes(List<FoundShape> input, List<FoundShape> output) {
		for (int i = 0; i < input.size(); i++) {
			FoundShape shape = input.get(i);

			// mark all the points which belong to this shape
			markPointsMember(shape, 1);

			// iterate through all the other shapes and see which ones match it and have a high ratio
			//TODO finish merge code

			// clean up
			markPointsMember(shape, -1);
		}
	}

	private void pruneFalseShapes(List<FoundShape> input, List<FoundShape> output) {

		output.addAll(input);

		// find the fraction of points for each shape in which another point has less error
		double suckRatios[] = new double[input.size()];

		for (int iter = 0; iter < 100; iter++) {
			System.out.println("Prune iteration " + iter);
			for (int i = 0; i < output.size(); i++) {
				FoundShape shape = output.get(i);

				setupShapeForFalse(shape);

				for (int j = 0; j < output.size(); j++) {
					if (i == j)
						continue;

					compareToShape(output.get(j));
				}

				// compute how many points are better describe by other shapes and decide if the shape should be kept
				int totalSuck = 0;
				for (int j = 0; j < shapePixels.size; j++) {
					PixelInfo info = shapePixels.get(j);

					if (info.matched) //info.internal > info.external )
						totalSuck++;
				}
				suckRatios[i] = totalSuck / (double) shapePixels.size;
				System.out.println("  suck ratio = " + suckRatios[i]);
				// clean up
				markPointsMember(shape, -1);
			}

			// select the worst one
			double worstRatio = 0;
			int worstIndex = 0;
			for (int i = 0; i < output.size(); i++) {
				if (suckRatios[i] > worstRatio) {
					worstRatio = suckRatios[i];
					worstIndex = i;
				}
			}

			if (worstRatio > thresholdDiscard) {
				output.remove(worstIndex);
			} else {
				break;
			}
		}
	}

	private void compareToShape(FoundShape shape) {
		ShapeDescription desc = models.get(shape.whichShape);
		desc.modelDistance.setModel(shape.modelParam);

		for (int i = 0; i < shape.points.size(); i++) {
			PointVectorNN pv = shape.points.get(i);

			int which = cloudToShape.data[pv.index];
			if (which < 0)
				continue;

			PixelInfo info = shapePixels.get(which);

			info.matched = true;

			double d = desc.modelDistance.computeDistance(pv);
			info.external = Math.min(d, info.external);
		}
	}

	/**
	 * Marks points in the point cloud as belonging to this shape for false shape detection
	 *
	 * @param shape
	 */
	private void setupShapeForFalse(FoundShape shape) {
		ShapeDescription desc = models.get(shape.whichShape);

		shapePixels.reset();

		desc.modelDistance.setModel(shape.modelParam);

		for (int i = 0; i < shape.points.size(); i++) {
			PointVectorNN pv = shape.points.get(i);

			cloudToShape.data[pv.index] = i;

			PixelInfo info = shapePixels.grow();
			info.internal = desc.modelDistance.computeDistance(shape.points.get(i));
			info.external = Double.MAX_VALUE;
			info.matched = false;
		}
	}

	private void markPointsMember(FoundShape shape, int value) {
		for (int i = 0; i < shape.points.size(); i++) {
			PointVectorNN pv = shape.points.get(i);

			cloudToShape.data[pv.index] = value;
		}
	}

	@Override
	public List<FoundShape> getOutput() {
		return goodShapes;
	}

	public static class PixelInfo {
		double external;
		double internal;
		boolean matched;
	}
}
