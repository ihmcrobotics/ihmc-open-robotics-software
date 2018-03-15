package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.ejml.data.DenseMatrix64F;
import org.junit.Test;

import us.ihmc.commons.Epsilons;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.Axis;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;

public class OptimizationControlModuleHelperTest
{
   @Test
   public void testConstructor()
   {
      OptimizationControlModuleHelper helper = new OptimizationControlModuleHelper(0.1, 0.09, 10.25, 120.0);
      assertTrue(helper != null);
   }
   
   @Test
   public void testNodeSubmissionWithInvalidNodeList()
   {
      OptimizationControlModuleHelper helper = new OptimizationControlModuleHelper(0.0d, 0.0d, -9.81d, 18.0d);
      RecycledLinkedListBuilder<CentroidalMotionNode> nodeList = new RecycledLinkedListBuilder<>(CentroidalMotionNode.class);
      boolean success = false;
      try {
         helper.processNodeList(nodeList);
      }catch (RuntimeException e) {
         success = true;
      }
      assertTrue(success);
   }
   
   @Test 
   public void testNodeSubmissionWithValidNodeList()
   {
      OptimizationControlModuleHelper helper = new OptimizationControlModuleHelper(0.0d, 0.0d, -9.81d, 1.0/12.0);
      RecycledLinkedListBuilder<CentroidalMotionNode> nodeList = new RecycledLinkedListBuilder<>(CentroidalMotionNode.class);
      RecycledLinkedListBuilder<CentroidalMotionNode>.RecycledLinkedListEntry<CentroidalMotionNode> entry1 = nodeList.getOrCreateFirstEntry();
      RecycledLinkedListBuilder<CentroidalMotionNode>.RecycledLinkedListEntry<CentroidalMotionNode> entry2 = nodeList.insertAfter(entry1);
      RecycledLinkedListBuilder<CentroidalMotionNode>.RecycledLinkedListEntry<CentroidalMotionNode> entry3 = nodeList.insertAfter(entry2);
      CentroidalMotionNode node1 = entry1.element;
      CentroidalMotionNode node2 = entry2.element;
      CentroidalMotionNode node3 = entry3.element;
      
      node1.setTime(0.0);
      node1.setForceAsHardConstraint(new FrameVector3D(ReferenceFrame.getWorldFrame(), 0.0, 0.0, 9.81 / 6.0));
      node1.setRateOfChangeOfForceAsHardConstraint(new FrameVector3D(ReferenceFrame.getWorldFrame(), 0.0, 0.0, 0.0));
      node1.setPositionObjective(new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.0, 0.0, 0.75), new FrameVector3D(ReferenceFrame.getWorldFrame(), 0.0, 0.0, 0.0));
      node1.setLinearVeclocityObjective(new FrameVector3D(ReferenceFrame.getWorldFrame(), 0.0, 0.0, 0.1), new FrameVector3D(ReferenceFrame.getWorldFrame(), 0.0, 0.0, 0.0));
      node2.setTime(0.1);
      node2.setForceAsObjective(new FrameVector3D(ReferenceFrame.getWorldFrame(), 0.0, 0.0, 200.0), new FrameVector3D());
      node2.setRateOfChangeOfForceAsObjective(new FrameVector3D(ReferenceFrame.getWorldFrame(), 0.0, 0.0, 0.0), new FrameVector3D(ReferenceFrame.getWorldFrame(), 0.0, 0.0, 0.0));
      node3.setTime(0.4);
      node3.setForceAsHardConstraint(new FrameVector3D(ReferenceFrame.getWorldFrame(), 0.0, 0.0, 9.81 / 6.0));
      node3.setRateOfChangeOfForceAsHardConstraint(new FrameVector3D(ReferenceFrame.getWorldFrame(), 0.0, 0.0, 0.0));
      
      helper.processNodeList(nodeList);
      DenseMatrix64F dTMatrix = helper.getDeltaTMatrix();
      assertTrue(dTMatrix.getNumCols() == 1);
      assertTrue(dTMatrix.getNumRows() == 2);
      assertEquals(0.1, dTMatrix.get(0, 0), Epsilons.ONE_TEN_MILLIONTH);
      assertEquals(0.3, dTMatrix.get(1, 0), Epsilons.ONE_TEN_MILLIONTH);
      assertEquals(2, helper.getNumberOfDecisionVariables(Axis.X));
      assertEquals(2, helper.getNumberOfDecisionVariables(Axis.Y));
      assertEquals(2, helper.getNumberOfDecisionVariables(Axis.Z));
      
      PrintTools.debug(helper.getVelocityCoefficientMatrix(Axis.Z).toString());
      PrintTools.debug(helper.getVelocityBiasMatrix(Axis.Z).toString());
      
   }
   
   
}
