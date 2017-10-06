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
import java.util.Random;

import org.ddogleg.fitting.modelset.ransac.RansacMulti;
import org.ddogleg.struct.FastQueue;

import georegression.struct.point.Point3D_F64;
import georegression.struct.point.Vector3D_F64;
import georegression.struct.shapes.Box3D_F64;
import us.ihmc.sensorProcessing.bubo.clouds.detect.shape.CheckShapeAcceptAll;
import us.ihmc.sensorProcessing.bubo.construct.ConstructOctreeNumPoints_F64;
import us.ihmc.sensorProcessing.bubo.construct.Octree;
import us.ihmc.sensorProcessing.bubo.construct.Octree_F64;

/**
 * Finds shapes inside a point cloud by partitioning the space using an Octree and performing a modified version
 * of RANSAC using local points inside an Octree node.  By sampling locally, points belonging to the shape
 * shape are more likely to be together.  Based on the Schnabel et. al. 2007 paper [1].
 * <p></p>
 * Input: List of 3D points, surface normal tangent vector, and nearest neighbors.  The sign of the tangent
 * vector does not matter.<br>
 * Output: List of found shapes and points which match them.  A list of unmatched points can also be computed.. The
 * returned parameter for each shape is NOT the optimal set of parameters given the set of matched points.  The index
 * of the model refers to the index of the shape description in the models list provided in the constructor.
 * <p></p>
 * DEVIATIONS FROM PAPER:  The most important deviation is that a bitmap image is not constructed for finding points
 * belonging to a shape.  Instead the results from the nearest-neighbor search are used to find all points which
 * match the shape..  Other deviations are mentioned throughout the code in comments.
 * <p></p>
 * [1] Schnabel, Ruwen, Roland Wahl, and Reinhard Klein. "Efficient RANSAC for Point‐Cloud Shape Detection."
 * Computer Graphics Forum. Vol. 26. No. 2. Blackwell Publishing Ltd, 2007.
 *
 * @author Peter Abeles
 */
public class PointCloudShapeDetectionSchnabel2007 {

	// constructs and maintains the octree
	protected ConstructOctreeNumPoints_F64 managerOctree;
	// used to randomly sample regions in the octree
	private Random rand;
	// An object must have this many points before being accepted as valid
	private int minModelAccept;
	// the initial bounding cube of the point cloud.  used when constructing the octree
	private Box3D_F64 bounding = new Box3D_F64();

	// list of leafs in the Octree.  Allows quick finding of the path to the base node
	private FastQueue<Octree_F64> leafs = new FastQueue<Octree_F64>(Octree_F64.class, false);

	// found path from leaf to base
	private FastQueue<Octree_F64> path = new FastQueue<Octree_F64>(Octree_F64.class, false);

	// searches the NN graph to find points which match the model.
	// need to use the same instance for all searches since it modified the points by marking them.
	private FindMatchSetPointVectorNN matchFinder = new FindMatchSetPointVectorNN();

	// robust model estimation
	private RansacShapeDetection ransac;

	// refines the estimate provided by RANSAC
	private LocalFitShapeNN refineShape;

	// list of found objects
	private FastQueue<FoundShape> foundObjects = new FastQueue<FoundShape>(FoundShape.class, true);

	// points with normal vectors
	private FastQueue<PointVectorNN> pointsNormal = new FastQueue<PointVectorNN>(PointVectorNN.class, false);
	// points without normal vectors
	private FastQueue<PointVectorNN> pointsNoNormal = new FastQueue<PointVectorNN>(PointVectorNN.class, false);

	// list of shapes which can be fit
	private List<ShapeDescription> models;

	// the maximum number of RANSAC iterations.  Can be used to control how long the process can run for
	private int maximumAllowedIterations;

	/**
	 * Configures the algorithm
	 *
	 * @param config Specified tuning parameters
	 */
	public PointCloudShapeDetectionSchnabel2007(ConfigSchnabel2007 config) {
		config.checkConfig();

		this.models = config.models;
		this.refineShape = new LocalFitShapeNN(config.localFitMaxIterations, config.localFitChangeThreshold, matchFinder);
		this.minModelAccept = config.minModelAccept;
		this.rand = new Random(config.randomSeed);
		this.maximumAllowedIterations = config.maximumAllowedIterations;

		managerOctree = new ConstructOctreeNumPoints_F64(config.octreeSplit);

		// convert it into a description that RANSAC understands
		List<RansacMulti.ObjectType> modelsRansac = new ArrayList<RansacMulti.ObjectType>();
		for (int i = 0; i < models.size(); i++) {
			ShapeDescription s = models.get(i);
			RansacMulti.ObjectType o = new RansacMulti.ObjectType();

			o.modelManager = s.modelManager;
			o.thresholdFit = s.thresholdFit;
			o.modelDistance = s.modelDistance;
			o.modelGenerator = s.modelGenerator;

			modelsRansac.add(o);

			// if a check is specified, pass it along
			if (s.modelCheck == null) {
				s.modelCheck = new CheckShapeAcceptAll();
			}
			s.modelGenerator.setCheck(s.modelCheck);
		}

		ransac = new RansacShapeDetection(config.randomSeed, config.ransacExtension * 2, matchFinder, modelsRansac);
	}

	/**
	 * For debugging purposes.
	 */
	protected PointCloudShapeDetectionSchnabel2007() {
	}

	/**
	 * Searches for shapes inside the provided point cloud.  It will continue to search until the maximum
	 * number of RANSAC iterations has been reached or all the points have been assigned ot shapes.
	 *
	 * @param points      Points in the point cloud.  PointVectorNN.used MUST be set to false.
	 * @param boundingBox Bonding box for use in the Octree.
	 */
	public void process(FastQueue<PointVectorNN> points, Box3D_F64 boundingBox) {

		// initialize data structures
		this.bounding.set(boundingBox);
		ransac.reset();
		foundObjects.reset();
		pointsNoNormal.reset();
		pointsNormal.reset();
		matchFinder.reset();
		for (int i = 0; i < models.size(); i++) {
			models.get(i).reset();
		}

		// split input points into a list with and without normal vectors
		for (int i = 0; i < points.size; i++) {
			PointVectorNN pv = points.data[i];
			Vector3D_F64 v = pv.normal;
			if (v.x == 0 && v.y == 0 && v.z == 0) {
				pointsNoNormal.add(pv);
			} else {
				pointsNormal.add(pv);
			}
		}

		// Constructs the Octree and finds its leafs
		constructOctree(pointsNormal);

		List<PointVectorNN> sampleSet = new ArrayList<PointVectorNN>();

		// run untill there are no more iterations or that there are not enough points left to fit an object
		int totalIterations = 0;
		while (totalIterations < maximumAllowedIterations &&
				managerOctree.getTree().points.size > minModelAccept) {
			// select region to search for a shape inside
			Octree_F64 sampleNode = selectSampleNode();

			// create list of points which are not a member of a shape yet
			sampleSet.clear();

			for (int i = 0; i < sampleNode.points.size; i++) {
				PointVectorNN pv = (PointVectorNN) sampleNode.points.get(i).userData;

				if (!pv.used)
					sampleSet.add(pv);
			}

			// see if its possible to find a valid model with this data
			if (sampleNode.points.size != sampleSet.size()) {
				// Update the octree since it clearly needs to since its running into trouble here and can't
				// run RANSAC.  Next cycle this situation will be impossible
				// By constructing the Octree here it is only constructed as needed
				constructOctree(pointsNormal);
				continue;
			}

			// use RANSAC to find a shape
			if (ransac.process(sampleSet)) {
				List<PointVectorNN> inliers = ransac.getMatchSet();

				// see if there are enough points to be a valid shape
				if (inliers.size() >= minModelAccept) {
					refineRansacShape();
				}
			}

			// note how many iterations have been processed
			totalIterations += ransac.getIteration();
		}
	}

	/**
	 * Searches for a locally optimal set of model parameters and inlier points.  Save results to
	 * a new shape for output
	 */
	protected void refineRansacShape() {
		Object ransacParam = ransac.getModelParameters();
		int whichShape = ransac.getModelIndex();
		List<PointVectorNN> ransacInliers = ransac.getMatchSet();

		ShapeDescription shapeDesc = models.get(whichShape);

		// create a new shape for output
		FoundShape output = foundObjects.grow();
		output.points.clear();
		output.modelParam = shapeDesc.createModel();
		output.whichShape = ransac.getModelIndex();

		// refine the model
		shapeDesc.modelManager.copyModel(ransacParam, output.modelParam);
		output.points.addAll(ransacInliers);
		refineShape.configure(shapeDesc.modelFitter, shapeDesc.modelDistance,
				shapeDesc.modelCheck, shapeDesc.codec, shapeDesc.thresholdFit);
		if (!refineShape.refine(output.points, output.modelParam, true)) {
			// the shape became invalid
			foundObjects.removeTail();
		} else if (output.points.size() < minModelAccept) {
			// see if the shape still has enough points to be accepted.  if the total number of matching points dropped
			// it is highly likely to be a poor fit to the shape anyways
			foundObjects.removeTail();
		} else {
			// mark shape points as being used
			for (int i = 0; i < output.points.size(); i++) {
				PointVectorNN p = output.points.get(i);
				p.used = true;
			}
		}
	}

	/**
	 * Instead of selecting a random point then searching for it in the tree,
	 * selects a random leaf on the tree and traces backwards to the root
	 *
	 * @return Node it should draw samples from
	 */
	protected Octree_F64 selectSampleNode() {
		Octree_F64 node = leafs.get(rand.nextInt(leafs.size));

		path.reset();
		path.add(node);

		node = node.parent;
		while (node != null) {
			path.add(node);
			node = node.parent;
		}

		// randomly select one of the nodes in the path
		// TODO use a PDF
		return path.get(rand.nextInt(path.size));
	}

	/**
	 * Constructs an Octree for the set of points.  The initial bound box will be found from the
	 * input points or is provided by the user.
	 * \
	 */
	// NOTE: Could optimize by maintaining a list of points which not used the last time it
	// was called and only search that
	protected void constructOctree(FastQueue<PointVectorNN> points) {

		managerOctree.initialize(bounding);

		// add points to the Octree
		for (int i = 0; i < points.size; i++) {
			PointVectorNN p = points.data[i];

			if (!p.used)
				managerOctree.addPoint(p.p, p);
		}

		findLeafs();
	}

	/**
	 * Searches for all the leafs in the Octree
	 */
	protected void findLeafs() {
		// create a list of leafs
		leafs.reset();

		FastQueue<Octree_F64> nodes = managerOctree.getAllNodes();
		for (int i = 0; i < nodes.size; i++) {
			Octree_F64 n = nodes.get(i);
			if (n.isLeaf()) {
				leafs.add(n);
			}
		}
	}

	/**
	 * Returns a list of all the objects that it found
	 */
	public FastQueue<FoundShape> getFoundObjects() {
		return foundObjects;
	}

	/**
	 * Searches through the points for any points which have yet to be used.  To save time only points
	 * in the most recently constructed Octree are used. Point not in that Octree must have been used.
	 *
	 * @param unmatched Output. Where the unmatched points are stored.
	 */
	public void findUnmatchedPoints(List<PointVectorNN> unmatched) {
		FastQueue<Octree.Info<Point3D_F64>> treePts = managerOctree.getTree().points;

		for (int i = 0; i < treePts.size; i++) {
			Octree_F64.Info info = treePts.data[i];
			PointVectorNN pv = (PointVectorNN) info.userData;
			if (!pv.used) {
				unmatched.add(pv);
			}
		}
		// go through list of points with no normal and see if they are matched or not
		for (int i = 0; i < pointsNoNormal.size; i++) {
			PointVectorNN pv = pointsNoNormal.data[i];
			if (!pv.used) {
				unmatched.add(pv);
			}
		}
	}

	public LocalFitShapeNN getRefineShape() {
		return refineShape;
	}

	public Box3D_F64 getBounding() {
		return bounding;
	}

	public FastQueue<Octree_F64> getLeafs() {
		return leafs;
	}

	public ConstructOctreeNumPoints_F64 getManagerOctree() {
		return managerOctree;
	}
}
