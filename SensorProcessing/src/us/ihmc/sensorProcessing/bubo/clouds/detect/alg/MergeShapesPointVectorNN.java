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

import org.ddogleg.fitting.modelset.DistanceFromModel;
import org.ddogleg.struct.GrowQueue_B;
import org.ddogleg.struct.GrowQueue_I32;

/**
 * Given a set of shapes and points which are members of the shapes, identify shapes which are really the same
 * and merge them together.  The algorithm works by first picking a target shape.  Then a test is done to find
 * which other shapes share points with the target.  Shapes which share a lot with the target are then compared
 * by computing their inlier sets using a distance function.  Shapes which have a lot in common are then
 * merged into one shape by adding all the points of one into the other and discarding the other.
 * <p/>
 * The shape parameters are not modified after merging.  A more optimal solution can be found by recomputing
 * the parameters using the modified membership point list.
 *
 * @author Peter Abeles
 */
public class MergeShapesPointVectorNN implements PostProcessShapes {
	// one boolean for each shape in the cloud.  true if member of the targeted shape
	protected GrowQueue_B member = new GrowQueue_B();
	// contains the index of shapes which might be mergable with the current target
	protected GrowQueue_I32 qualified = new GrowQueue_I32();
	// contains functions used to describe each type of possible shape
	private List<ShapeDescription> models;
	// storage for the final output shapes after merging
	private List<FoundShape> output = new ArrayList<FoundShape>();
	// used when determining if two shape should be merged.  mutual memberships
	private List<PointVectorNN> membersAinB = new ArrayList<PointVectorNN>();
	private List<PointVectorNN> membersBinA = new ArrayList<PointVectorNN>();

	// was the points list in the dominant shape modified
	private boolean modifiedDominant;

	// improves members list and shape parameters
	private LocalFitShapeNN refine;

	// minimum overlap to consider merging the shapes
	private double commonPointsFraction;

	// if a shape shares this function of points after the more rigorous test
	private double commonMembershipFraction;

	/**
	 * Configures the class
	 *
	 * @param commonPointsFraction     Minimum fraction of points in common that two objects have for them to
	 *                                 be considered for merging.  Try 0.6
	 * @param commonMembershipFraction Minimum fraction of points which belong to another the other shape for them to be merged.
	 *                                 Try 0.9
	 */
	public MergeShapesPointVectorNN(double commonPointsFraction, double commonMembershipFraction) {
		this.models = models;
		this.commonPointsFraction = commonPointsFraction;
		this.commonMembershipFraction = commonMembershipFraction;
		this.refine = refine;
	}

	/**
	 * @param models Describe of the different shapes that it can merge
	 * @param refine Used to improve the shape parameters and list of member points after merging.
	 *               If null then no refinement is done.
	 */
	@Override
	public void setup(List<ShapeDescription> models, LocalFitShapeNN refine) {
		this.models = models;
		this.refine = refine;
	}

	/**
	 * Searches for point which can be merged.  Results are returned by calling output.
	 *
	 * @param input     Input list.  Is modified..
	 * @param cloudSize Number of points in the original point cloud.
	 */
	@Override
	public void process(List<FoundShape> input, int cloudSize) {
		// can assume it is already all false.  boolean initializes to false and it should be cleaned up and
		// set to false again before this function exits
		member.resize(cloudSize);

		output.clear();
		output.addAll(input);

		// find which shapes have which points as members
		for (int i = 0; i < output.size() && output.size() > 1; ) {
			FoundShape shapeA = output.get(i);

			// mark which points belong ot this shape
			markPoints(shapeA.points, true);

			// search for shapes which pass the first test for merging
			qualified.reset();
			for (int j = 0; j < output.size(); j++) {
				if (i == j)
					continue;
				FoundShape shapeB = output.get(j);
				int count = 0;
				for (int k = 0; k < shapeB.points.size(); k++) {
					PointVectorNN pv = shapeB.points.get(k);
					if (member.data[pv.index]) {
						count++;
					}
				}
				// compute the fractional overlap in each shape
				double overlap = Math.max(count / (double) shapeB.points.size(), count / (double) shapeA.points.size());
				if (overlap >= commonPointsFraction) {
					qualified.add(j);
				}
			}

			// remove the marking
			markPoints(shapeA.points, false);

			// go through the qualified list and see if it can merge with any of them
			boolean anyMerges = false;
			for (int j = 0; j < qualified.size; j++) {
				int indexB = qualified.get(j);
				int mergeAction = checkMergeShapes(shapeA, output.get(indexB));

				if (mergeAction == 1) {
					if (modifiedDominant && refine != null)
						refineShape(output.get(i));

					anyMerges = true;
					output.remove(indexB);
					i = Math.min(i, indexB);
					break;
				} else if (mergeAction == 2) {
					if (modifiedDominant && refine != null)
						refineShape(output.get(indexB));

					anyMerges = true;
					output.remove(i);
					i = Math.min(i, indexB);
					break;
				}
			}
			if (!anyMerges) {
				i++;
			}
		}
	}

	/**
	 * Recomputes the shape parameters and neighbor list after merging
	 *
	 * @param shape
	 */
	protected void refineShape(FoundShape shape) {

		ShapeDescription shapeDesc = models.get(shape.whichShape);

		refine.configure(shapeDesc.modelFitter, shapeDesc.modelDistance, shapeDesc.modelCheck, shapeDesc.codec, shapeDesc.thresholdFit);
		refine.refine(shape.points, shape.modelParam, true);
		// todo handle if refine is false
	}

	/**
	 * Performs a more rigorous fractional membership check and if above a threshold.  The dominant
	 * one will then consume the other shape.
	 *
	 * @return 0 no merge.  1 = shapeA is dominant.  2 = shapeB is dominant.
	 */
	protected int checkMergeShapes(FoundShape shapeA, FoundShape shapeB) {
		membersAinB.clear();
		membersBinA.clear();

		// find points which are mutual members
		findMembersRigorous(shapeA, shapeB.points, membersBinA);
		findMembersRigorous(shapeB, shapeA.points, membersAinB);

		// see if one of the shapes has a bunch of points in the other
		double fracAinB = membersAinB.size() / (double) shapeA.points.size();
		double fracBinA = membersBinA.size() / (double) shapeB.points.size();

		if (Math.max(fracAinB, fracBinA) <= commonMembershipFraction) {
			return 0;
		}

		if (fracAinB > fracBinA) {
			// B is the dominant one
			modifiedDominant = mergeShape(shapeB, shapeA.points);
			return 2;
		} else {
			// A is the dominant one
			modifiedDominant = mergeShape(shapeA, shapeB.points);
			return 1;
		}
	}

	/**
	 * Finds members of 'shape' in 'points' using a distance test.
	 */
	protected void findMembersRigorous(FoundShape shape, List<PointVectorNN> points, List<PointVectorNN> members) {
		ShapeDescription desc = models.get(shape.whichShape);
		DistanceFromModel function = desc.modelDistance;
		double threshold = desc.thresholdFit;

		function.setModel(shape.modelParam);

		for (int i = 0; i < points.size(); i++) {
			PointVectorNN pv = points.get(i);
			double d = function.computeDistance(pv);
			if (d <= threshold) {
				members.add(pv);
			}
		}
	}

	/**
	 * Adds points in 'points' which are not already members of 'dominant'.
	 *
	 * @return true if points are added to the dominate shape
	 */
	protected boolean mergeShape(FoundShape dominant, List<PointVectorNN> points) {
		markPoints(dominant.points, true);

		boolean changed = false;

		// add points which are not already a member of dominant
		for (int i = 0; i < points.size(); i++) {
			PointVectorNN pv = points.get(i);
			if (!member.data[pv.index]) {
				dominant.points.add(pv);
				changed = true;
			}
		}

		markPoints(dominant.points, false);

		return changed;
	}

	protected void markPoints(List<PointVectorNN> points, boolean value) {
		for (int j = 0; j < points.size(); j++) {
			PointVectorNN pv = points.get(j);
			member.data[pv.index] = value;
		}
	}

	@Override
	public List<FoundShape> getOutput() {
		return output;
	}

}
