package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner;

import static org.junit.Assert.assertTrue;

import org.junit.Test;

import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;

public class CentroidalMotionPlannerTest
{
   private static final CentroidalMotionPlannerParameters parameters = new CentroidalMotionPlannerParameters();
   static
   {
      parameters.setRobotMass(18.0);
      parameters.setDeltaTMin(0.001);
   }

   @Test
   public void testConstructor()
   {
      CentroidalMotionPlanner motionPlanner = new CentroidalMotionPlanner(parameters);
      assertTrue(motionPlanner != null);
   }

   @Test
   public void testNodeEntrySubmission()
   {
      CentroidalMotionNode node1 = new CentroidalMotionNode();
      node1.setTime(0.1);
      node1.setForceAsHardConstraint(new FrameVector3D(ReferenceFrame.getWorldFrame(), Double.NaN, Double.NaN, 100.0));
      CentroidalMotionNode node2 = new CentroidalMotionNode();
      node2.setTime(0.0);
      node2.setForceAsHardConstraint(new FrameVector3D(ReferenceFrame.getWorldFrame(), Double.NaN, Double.NaN, 123.0));
      CentroidalMotionNode node3 = new CentroidalMotionNode();
      node3.setTime(0.5);
      node3.setForceAsHardConstraint(new FrameVector3D(ReferenceFrame.getWorldFrame(), Double.NaN, Double.NaN, 0.0));

      CentroidalMotionPlanner motionPlanner = new CentroidalMotionPlanner(parameters);
      motionPlanner.sumbitNode(node1);
      motionPlanner.sumbitNode(node2);
      motionPlanner.sumbitNode(node3);

      RecycledLinkedListBuilder<CentroidalMotionNode> nodeList = motionPlanner.getNodeList();
      assertTrue(nodeList.getSize() == 3);
      RecycledLinkedListBuilder<CentroidalMotionNode>.RecycledLinkedListEntry<CentroidalMotionNode> node = nodeList.getFirstEntry();
      assertTrue(node.element.getTime() == 0.0);
      node = node.getNext();
      assertTrue(node.element.getTime() == 0.1);
      node = node.getNext();
      assertTrue(node.element.getTime() == 0.5);
   }
}