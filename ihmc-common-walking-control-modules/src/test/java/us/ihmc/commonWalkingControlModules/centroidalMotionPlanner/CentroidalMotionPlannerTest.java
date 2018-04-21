package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner;

import static org.junit.Assert.assertTrue;

import org.junit.Test;

import afu.org.checkerframework.common.reflection.qual.GetClass;
import us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.RecycledLinkedListBuilder.RecycledLinkedListEntry;
import us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.trajectories.ForceTrajectory;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class CentroidalMotionPlannerTest
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final CentroidalMotionPlannerParameters parameters = new CentroidalMotionPlannerParameters();
   private static final String namePrefix = "CentroidalMotionPlannerTest";
   static
   {
      parameters.setRobotMass(1.0);
      parameters.setDeltaTMin(0.001);
      parameters.setGravityZ(-9.81);
   }

   @Test
   public void testConstructor()
   {
      YoVariableRegistry registry = new YoVariableRegistry("Dummy");
      CentroidalMotionPlanner motionPlanner = new CentroidalMotionPlanner(parameters, registry);
      assertTrue(motionPlanner != null);
   }

   @Test
   public void testNodeEntrySubmission()
   {
      YoVariableRegistry registry = new YoVariableRegistry("Dummy");
      CentroidalMotionNode node1 = new CentroidalMotionNode();
      node1.setTime(0.1);
      node1.setForceConstraint(new FrameVector3D(ReferenceFrame.getWorldFrame(), Double.NaN, Double.NaN, 100.0));
      CentroidalMotionNode node2 = new CentroidalMotionNode();
      node2.setTime(0.0);
      node2.setForceConstraint(new FrameVector3D(ReferenceFrame.getWorldFrame(), Double.NaN, Double.NaN, 123.0));
      CentroidalMotionNode node3 = new CentroidalMotionNode();
      node3.setTime(0.5);
      node3.setForceConstraint(new FrameVector3D(ReferenceFrame.getWorldFrame(), Double.NaN, Double.NaN, 0.0));

      CentroidalMotionPlanner motionPlanner = new CentroidalMotionPlanner(parameters, registry);
      motionPlanner.submitNode(node1);
      motionPlanner.submitNode(node2);
      motionPlanner.submitNode(node3);

      RecycledLinkedListBuilder<CentroidalMotionNode> nodeList = motionPlanner.getNodeList();
      assertTrue(nodeList.getSize() == 3);
      RecycledLinkedListBuilder<CentroidalMotionNode>.RecycledLinkedListEntry<CentroidalMotionNode> node = nodeList.getFirstEntry();
      assertTrue(node.element.getTime() == 0.0);
      node = node.getNext();
      assertTrue(node.element.getTime() == 0.1);
      node = node.getNext();
      assertTrue(node.element.getTime() == 0.5);
   }

   @Test
   public void testCentroidalTrajectoryGeneration()
   {
      CentroidalMotionNode node1 = new CentroidalMotionNode();
      CentroidalMotionNode node2 = new CentroidalMotionNode();
      CentroidalMotionNode node3 = new CentroidalMotionNode();
      double gravityZ = -9.81;
      double forceZValue = -1.0 * gravityZ;
      node1.setTime(0.0);
      node1.setForceConstraint(new FrameVector3D(worldFrame, 0.0, 0.0, forceZValue));
      node1.setForceRateConstraint(new FrameVector3D(worldFrame, 0.0, 0.0, 0.0));
      node1.setPositionObjective(new FramePoint3D(worldFrame, 0.0, 0.0, 0.75), new FrameVector3D(worldFrame, 10.0, 10.0, 10.0));
      node1.setLinearVelocityObjective(new FrameVector3D(worldFrame, 0.0, 0.0, 0.0), new FrameVector3D(worldFrame, 10.0, 10.0, 10.0));
      node2.setTime(0.1);
      node2.setForceObjective(new FrameVector3D(worldFrame, 0.0, 0.0, forceZValue), new FrameVector3D(worldFrame, 0.1, 0.1, 0.1));
      node2.setForceRateObjective(new FrameVector3D(worldFrame, 0.0, 0.0, 0.0), new FrameVector3D(worldFrame, 0.001, 0.001, 0.1));
      node3.setTime(0.8);
      node3.setForceConstraint(new FrameVector3D(worldFrame, 0.0, 0.0, forceZValue));
      node3.setForceRateConstraint(new FrameVector3D(worldFrame, 0.0, 0.0, 0.0));
      node3.setPositionObjective(new FramePoint3D(worldFrame, 0.0, 0.0, 0.75), new FrameVector3D(worldFrame, 10.0, 10.0, 10.0));
      node3.setLinearVelocityObjective(new FrameVector3D(worldFrame, 0.0, 0.0, 0.0), new FrameVector3D(worldFrame, 10.0, 10.0, 10.0));

      YoVariableRegistry registry = new YoVariableRegistry("Dummy");
      CentroidalMotionPlanner motionPlanner = new CentroidalMotionPlanner(parameters, registry);
      motionPlanner.reset();
      motionPlanner.submitNode(node1);
      motionPlanner.submitNode(node2);
      motionPlanner.submitNode(node3);

      RecycledLinkedListBuilder<CentroidalMotionNode> nodeList = motionPlanner.getNodeList();
      assertTrue(nodeList.getSize() == 3);

      RecycledLinkedListBuilder<CentroidalMotionNode>.RecycledLinkedListEntry<CentroidalMotionNode> entry = nodeList.getFirstEntry();
      assertTrue("Node time: " + entry.element.getTime(), node1.getTime() == entry.element.getTime());
      entry = entry.getNext();
      assertTrue("Node time: " + entry.element.getTime(), node2.getTime() == entry.element.getTime());
      entry = entry.getNext();
      assertTrue("Node time: " + entry.element.getTime(), node3.getTime() == entry.element.getTime());

      motionPlanner.compute();
      ForceTrajectory force = motionPlanner.getForceProfile();
      assertTrue(force != null);
      assertTrue(force.getNumberOfSegments() == 2);
   }
}