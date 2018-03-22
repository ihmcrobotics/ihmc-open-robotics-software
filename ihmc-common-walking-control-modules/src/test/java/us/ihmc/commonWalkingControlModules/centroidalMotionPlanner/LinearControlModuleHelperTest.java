package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.junit.Test;

import us.ihmc.commons.Epsilons;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.PrintTools;
import us.ihmc.convexOptimization.quadraticProgram.JavaQuadProgSolver;
import us.ihmc.euclid.Axis;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;

public class LinearControlModuleHelperTest
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final CentroidalMotionPlannerParameters parameters = new CentroidalMotionPlannerParameters();
   static
   {
      parameters.setRobotMass(1.0);
      parameters.setDeltaTMin(0.001);
      parameters.setGravityZ(-9.81);
   }

   @Test
   public void testConstructor()
   {
      LinearControlModuleHelper helper = new LinearControlModuleHelper(parameters);
      assertTrue(helper != null);
   }

   @Test
   public void testNodeSubmissionWithInvalidNodeList()
   {
      LinearControlModuleHelper helper = new LinearControlModuleHelper(parameters);
      RecycledLinkedListBuilder<CentroidalMotionNode> nodeList = new RecycledLinkedListBuilder<>(CentroidalMotionNode.class);
      boolean success = false;
      try
      {
         helper.processNodeList(nodeList);
      }
      catch (RuntimeException e)
      {
         success = true;
      }
      assertTrue(success);
   }

   @Test
   public void testNodeSubmissionWithValidNodeList()
   {
      LinearControlModuleHelper helper = new LinearControlModuleHelper(parameters);
      RecycledLinkedListBuilder<CentroidalMotionNode> nodeList = new RecycledLinkedListBuilder<>(CentroidalMotionNode.class);
      RecycledLinkedListBuilder<CentroidalMotionNode>.RecycledLinkedListEntry<CentroidalMotionNode> entry1 = nodeList.getOrCreateFirstEntry();
      RecycledLinkedListBuilder<CentroidalMotionNode>.RecycledLinkedListEntry<CentroidalMotionNode> entry2 = nodeList.insertAfter(entry1);
      RecycledLinkedListBuilder<CentroidalMotionNode>.RecycledLinkedListEntry<CentroidalMotionNode> entry3 = nodeList.insertAfter(entry2);
      CentroidalMotionNode node1 = entry1.element;
      CentroidalMotionNode node2 = entry2.element;
      CentroidalMotionNode node3 = entry3.element;

      node1.setTime(0.0);
      node1.setForceConstraint(new FrameVector3D(worldFrame, 0.0, 0.0, 9.81 / 6.0));
      node1.setForceRateConstraint(new FrameVector3D(worldFrame, 0.0, 0.0, 0.0));
      node1.setPositionObjective(new FramePoint3D(worldFrame, 0.0, 0.0, 0.75), new FrameVector3D(worldFrame, 0.0, 0.0, 0.0));
      node1.setLinearVelocityObjective(new FrameVector3D(worldFrame, 0.0, 0.0, 0.1), new FrameVector3D(worldFrame, 0.0, 0.0, 0.0));
      node2.setTime(0.1);
      node2.setForceObjective(new FrameVector3D(worldFrame, 0.0, 0.0, 200.0), new FrameVector3D());
      node2.setForceRateObjective(new FrameVector3D(worldFrame, 0.0, 0.0, 0.0), new FrameVector3D(worldFrame, 0.0, 0.0, 0.0));
      node3.setTime(0.4);
      node3.setForceConstraint(new FrameVector3D(worldFrame, 0.0, 0.0, 9.81 / 6.0));
      node3.setForceRateConstraint(new FrameVector3D(worldFrame, 0.0, 0.0, 0.0));

      helper.processNodeList(nodeList);
      DenseMatrix64F dTMatrix = helper.getDeltaTMatrix();
      assertTrue(dTMatrix.getNumCols() == 1);
      assertTrue(dTMatrix.getNumRows() == 2);
      assertEquals(0.1, dTMatrix.get(0, 0), Epsilons.ONE_TEN_MILLIONTH);
      assertEquals(0.3, dTMatrix.get(1, 0), Epsilons.ONE_TEN_MILLIONTH);
      assertEquals(2, helper.getNumberOfDecisionVariables(Axis.X));
      assertEquals(2, helper.getNumberOfDecisionVariables(Axis.Y));
      assertEquals(2, helper.getNumberOfDecisionVariables(Axis.Z));
   }

   @Test
   public void testCoefficientsValueCalculationWithForcesAsObjectives()
   {
      double gravityZ = -9.81d;
      double epsilon = Epsilons.ONE_TEN_MILLIONTH;

      LinearControlModuleHelper helper = new LinearControlModuleHelper(parameters);
      RecycledLinkedListBuilder<CentroidalMotionNode> nodeList = new RecycledLinkedListBuilder<>(CentroidalMotionNode.class);
      RecycledLinkedListBuilder<CentroidalMotionNode>.RecycledLinkedListEntry<CentroidalMotionNode> entry1 = nodeList.getOrCreateFirstEntry();
      RecycledLinkedListBuilder<CentroidalMotionNode>.RecycledLinkedListEntry<CentroidalMotionNode> entry2 = nodeList.insertAfter(entry1);
      RecycledLinkedListBuilder<CentroidalMotionNode>.RecycledLinkedListEntry<CentroidalMotionNode> entry3 = nodeList.insertAfter(entry2);
      CentroidalMotionNode node1 = entry1.element;
      CentroidalMotionNode node2 = entry2.element;
      CentroidalMotionNode node3 = entry3.element;

      node1.setTime(0.0);
      node1.setForceConstraint(new FrameVector3D(worldFrame, 0.0, 0.0, 0.0));
      node1.setForceRateConstraint(new FrameVector3D(worldFrame, 0.0, 0.0, 0.0));
      node1.setPositionObjective(new FramePoint3D(worldFrame, 0.0, 0.0, 0.75), new FrameVector3D(worldFrame, 0.0, 0.0, 0.0));
      node1.setLinearVelocityObjective(new FrameVector3D(worldFrame, 0.0, 0.0, 0.1), new FrameVector3D(worldFrame, 0.0, 0.0, 0.0));
      node2.setTime(0.1);
      node2.setForceConstraint(new FrameVector3D(worldFrame, 0.0, 0.0, 0.0));
      node2.setForceRateConstraint(new FrameVector3D(worldFrame, 0.0, 0.0, 0.0));
      node3.setTime(0.4);
      node3.setForceConstraint(new FrameVector3D(worldFrame, 0.0, 0.0, 0.0));
      node3.setForceRateConstraint(new FrameVector3D(worldFrame, 0.0, 0.0, 0.0));

      helper.processNodeList(nodeList);
      DenseMatrix64F dTMatrix = helper.getDeltaTMatrix();
      assertTrue(dTMatrix.getNumCols() == 1);
      assertTrue(dTMatrix.getNumRows() == 2);
      assertEquals(0.1, dTMatrix.get(0, 0), epsilon);
      assertEquals(0.3, dTMatrix.get(1, 0), epsilon);
      assertEquals(0, helper.getNumberOfDecisionVariables(Axis.X));
      assertEquals(0, helper.getNumberOfDecisionVariables(Axis.Y));
      assertEquals(0, helper.getNumberOfDecisionVariables(Axis.Z));
      DenseMatrix64F forceZBias = helper.getForceBiasMatrix(Axis.Z);
      assertEquals(0.0, forceZBias.get(0, 0), epsilon);
      assertEquals(0.0, forceZBias.get(1, 0), epsilon);
      assertEquals(0.0, forceZBias.get(2, 0), epsilon);
      DenseMatrix64F rateChangeOfForceZBias = helper.getForceRateBiasMatrix(Axis.Z);
      assertEquals(0.0, rateChangeOfForceZBias.get(0, 0), epsilon);
      assertEquals(0.0, rateChangeOfForceZBias.get(1, 0), epsilon);
      assertEquals(0.0, rateChangeOfForceZBias.get(2, 0), epsilon);
      DenseMatrix64F velocityZBias = helper.getVelocityBiasMatrix(Axis.Z);
      assertEquals(0.1, velocityZBias.get(0, 0), epsilon);
      assertEquals(0.1 + gravityZ * 0.1, velocityZBias.get(1, 0), epsilon);
      assertEquals(0.1 + gravityZ * 0.4, velocityZBias.get(2, 0), epsilon);
      DenseMatrix64F positionZBias = helper.getPositionBiasMatrix(Axis.Z);
      assertEquals(0.75, positionZBias.get(0, 0), epsilon);
      assertEquals(0.75 + 0.5 * 0.1 * 0.1 * gravityZ + 0.1 * 0.1, positionZBias.get(1, 0), epsilon);
      assertEquals(0.75 + 0.5 * 0.4 * 0.4 * gravityZ + 0.1 * 0.4, positionZBias.get(2, 0), epsilon);
   }

   @Test
   public void testBiasValueCalculationWithZeroGroundReactionForces()
   {
      double gravityZ = -9.81d;
      double epsilon = Epsilons.ONE_TEN_MILLIONTH;

      LinearControlModuleHelper helper = new LinearControlModuleHelper(parameters);
      RecycledLinkedListBuilder<CentroidalMotionNode> nodeList = new RecycledLinkedListBuilder<>(CentroidalMotionNode.class);
      RecycledLinkedListBuilder<CentroidalMotionNode>.RecycledLinkedListEntry<CentroidalMotionNode> entry1 = nodeList.getOrCreateFirstEntry();
      RecycledLinkedListBuilder<CentroidalMotionNode>.RecycledLinkedListEntry<CentroidalMotionNode> entry2 = nodeList.insertAfter(entry1);
      RecycledLinkedListBuilder<CentroidalMotionNode>.RecycledLinkedListEntry<CentroidalMotionNode> entry3 = nodeList.insertAfter(entry2);
      CentroidalMotionNode node1 = entry1.element;
      CentroidalMotionNode node2 = entry2.element;
      CentroidalMotionNode node3 = entry3.element;

      node1.setTime(0.0);
      node1.setForceConstraint(new FrameVector3D(worldFrame, 0.0, 0.0, 0.0));
      node1.setForceRateConstraint(new FrameVector3D(worldFrame, 0.0, 0.0, 0.0));
      node1.setPositionObjective(new FramePoint3D(worldFrame, 0.0, 0.0, 0.75), new FrameVector3D(worldFrame, 0.0, 0.0, 0.0));
      node1.setLinearVelocityObjective(new FrameVector3D(worldFrame, 0.0, 0.0, 0.1), new FrameVector3D(worldFrame, 0.0, 0.0, 0.0));
      node2.setTime(0.1);
      node2.setForceConstraint(new FrameVector3D(worldFrame, 0.0, 0.0, 0.0));
      node2.setForceRateConstraint(new FrameVector3D(worldFrame, 0.0, 0.0, 0.0));
      node3.setTime(0.4);
      node3.setForceConstraint(new FrameVector3D(worldFrame, 0.0, 0.0, 0.0));
      node3.setForceRateConstraint(new FrameVector3D(worldFrame, 0.0, 0.0, 0.0));

      helper.processNodeList(nodeList);
      DenseMatrix64F dTMatrix = helper.getDeltaTMatrix();
      assertTrue(dTMatrix.getNumCols() == 1);
      assertTrue(dTMatrix.getNumRows() == 2);
      assertEquals(0.1, dTMatrix.get(0, 0), epsilon);
      assertEquals(0.3, dTMatrix.get(1, 0), epsilon);
      assertEquals(0, helper.getNumberOfDecisionVariables(Axis.X));
      assertEquals(0, helper.getNumberOfDecisionVariables(Axis.Y));
      assertEquals(0, helper.getNumberOfDecisionVariables(Axis.Z));
      DenseMatrix64F forceZBias = helper.getForceBiasMatrix(Axis.Z);
      assertEquals(0.0, forceZBias.get(0, 0), epsilon);
      assertEquals(0.0, forceZBias.get(1, 0), epsilon);
      assertEquals(0.0, forceZBias.get(2, 0), epsilon);
      DenseMatrix64F rateChangeOfForceZBias = helper.getForceRateBiasMatrix(Axis.Z);
      assertEquals(0.0, rateChangeOfForceZBias.get(0, 0), epsilon);
      assertEquals(0.0, rateChangeOfForceZBias.get(1, 0), epsilon);
      assertEquals(0.0, rateChangeOfForceZBias.get(2, 0), epsilon);
      DenseMatrix64F velocityZBias = helper.getVelocityBiasMatrix(Axis.Z);
      assertEquals(0.1, velocityZBias.get(0, 0), epsilon);
      assertEquals(0.1 + gravityZ * 0.1, velocityZBias.get(1, 0), epsilon);
      assertEquals(0.1 + gravityZ * 0.4, velocityZBias.get(2, 0), epsilon);
      DenseMatrix64F positionZBias = helper.getPositionBiasMatrix(Axis.Z);
      assertEquals(0.75, positionZBias.get(0, 0), epsilon);
      assertEquals(0.75 + 0.5 * 0.1 * 0.1 * gravityZ + 0.1 * 0.1, positionZBias.get(1, 0), epsilon);
      assertEquals(0.75 + 0.5 * 0.4 * 0.4 * gravityZ + 0.1 * 0.4, positionZBias.get(2, 0), epsilon);
   }

   @Test
   public void testBiasValueCalculationWithConstantForceValue()
   {
      double gravityZ = -9.81d;
      double epsilon = Epsilons.ONE_TEN_MILLIONTH;

      LinearControlModuleHelper helper = new LinearControlModuleHelper(parameters);
      RecycledLinkedListBuilder<CentroidalMotionNode> nodeList = new RecycledLinkedListBuilder<>(CentroidalMotionNode.class);
      RecycledLinkedListBuilder<CentroidalMotionNode>.RecycledLinkedListEntry<CentroidalMotionNode> entry1 = nodeList.getOrCreateFirstEntry();
      RecycledLinkedListBuilder<CentroidalMotionNode>.RecycledLinkedListEntry<CentroidalMotionNode> entry2 = nodeList.insertAfter(entry1);
      RecycledLinkedListBuilder<CentroidalMotionNode>.RecycledLinkedListEntry<CentroidalMotionNode> entry3 = nodeList.insertAfter(entry2);
      CentroidalMotionNode node1 = entry1.element;
      CentroidalMotionNode node2 = entry2.element;
      CentroidalMotionNode node3 = entry3.element;

      node1.setTime(0.0);
      double forceZValue = -2.0 * gravityZ;
      node1.setForceConstraint(new FrameVector3D(worldFrame, 0.0, 0.0, forceZValue));
      node1.setForceRateConstraint(new FrameVector3D(worldFrame, 0.0, 0.0, 0.0));
      node1.setPositionObjective(new FramePoint3D(worldFrame, 0.0, 0.0, 0.75), new FrameVector3D(worldFrame, 0.0, 0.0, 0.0));
      node1.setLinearVelocityObjective(new FrameVector3D(worldFrame, 0.0, 0.0, 0.1), new FrameVector3D(worldFrame, 0.0, 0.0, 0.0));
      node2.setTime(0.1);
      node2.setForceConstraint(new FrameVector3D(worldFrame, 0.0, 0.0, forceZValue));
      node2.setForceRateConstraint(new FrameVector3D(worldFrame, 0.0, 0.0, 0.0));
      node3.setTime(0.4);
      node3.setForceConstraint(new FrameVector3D(worldFrame, 0.0, 0.0, forceZValue));
      node3.setForceRateConstraint(new FrameVector3D(worldFrame, 0.0, 0.0, 0.0));

      helper.processNodeList(nodeList);
      DenseMatrix64F dTMatrix = helper.getDeltaTMatrix();
      assertTrue(dTMatrix.getNumCols() == 1);
      assertTrue(dTMatrix.getNumRows() == 2);
      assertEquals(0.1, dTMatrix.get(0, 0), epsilon);
      assertEquals(0.3, dTMatrix.get(1, 0), epsilon);
      assertEquals(0, helper.getNumberOfDecisionVariables(Axis.X));
      assertEquals(0, helper.getNumberOfDecisionVariables(Axis.Y));
      assertEquals(0, helper.getNumberOfDecisionVariables(Axis.Z));
      DenseMatrix64F forceZBias = helper.getForceBiasMatrix(Axis.Z);
      assertEquals(forceZValue, forceZBias.get(0, 0), epsilon);
      assertEquals(forceZValue, forceZBias.get(1, 0), epsilon);
      assertEquals(forceZValue, forceZBias.get(2, 0), epsilon);
      DenseMatrix64F rateChangeOfForceZBias = helper.getForceRateBiasMatrix(Axis.Z);
      assertEquals(0.0, rateChangeOfForceZBias.get(0, 0), epsilon);
      assertEquals(0.0, rateChangeOfForceZBias.get(1, 0), epsilon);
      assertEquals(0.0, rateChangeOfForceZBias.get(2, 0), epsilon);
      DenseMatrix64F velocityZBias = helper.getVelocityBiasMatrix(Axis.Z);
      assertEquals(0.1, velocityZBias.get(0, 0), epsilon);
      assertEquals(0.1 - gravityZ * 0.1, velocityZBias.get(1, 0), epsilon);
      assertEquals(0.1 - gravityZ * 0.4, velocityZBias.get(2, 0), epsilon);
      DenseMatrix64F positionZBias = helper.getPositionBiasMatrix(Axis.Z);
      assertEquals(0.75, positionZBias.get(0, 0), epsilon);
      assertEquals(0.75 - 0.5 * 0.1 * 0.1 * gravityZ + 0.1 * 0.1, positionZBias.get(1, 0), epsilon);
      assertEquals(0.75 - 0.5 * 0.4 * 0.4 * gravityZ + 0.1 * 0.4, positionZBias.get(2, 0), epsilon);
   }

   @Test
   public void testOptimization()
   {
      double gravityZ = -9.81d;
      double epsilon = Epsilons.ONE_TEN_MILLIONTH;

      LinearControlModuleHelper helper = new LinearControlModuleHelper(parameters);
      RecycledLinkedListBuilder<CentroidalMotionNode> nodeList = new RecycledLinkedListBuilder<>(CentroidalMotionNode.class);
      RecycledLinkedListBuilder<CentroidalMotionNode>.RecycledLinkedListEntry<CentroidalMotionNode> entry1 = nodeList.getOrCreateFirstEntry();
      RecycledLinkedListBuilder<CentroidalMotionNode>.RecycledLinkedListEntry<CentroidalMotionNode> entry2 = nodeList.insertAfter(entry1);
      RecycledLinkedListBuilder<CentroidalMotionNode>.RecycledLinkedListEntry<CentroidalMotionNode> entry3 = nodeList.insertAfter(entry2);
      CentroidalMotionNode node1 = entry1.element;
      CentroidalMotionNode node2 = entry2.element;
      CentroidalMotionNode node3 = entry3.element;

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
      //node3.setPositionConstraint(new FramePoint3D(worldFrame, 0.0, 0.0, 0.75));
      //node3.setLinearVelocityConstraint(new FrameVector3D(worldFrame, 0.0, 0.0, 0.0));

      helper.processNodeList(nodeList);
      testAxisOptimization(epsilon, helper, Axis.X, 0.00, 0.00);
      testAxisOptimization(epsilon, helper, Axis.Y, 0.00, 0.00);
      testAxisOptimization(epsilon, helper, Axis.Z, 9.81, 0.00);
   }

   private void testAxisOptimization(double epsilon, LinearControlModuleHelper helper, Axis axisToTest, double expectedForceValue, double expectedForceRateValue)
   {
//      PrintTools.debug("Testing axis: " + axisToTest.toString());
      JavaQuadProgSolver qpSolver = new JavaQuadProgSolver();
      DenseMatrix64F H = helper.getObjectiveHMatrix(axisToTest);
      DenseMatrix64F f = helper.getObjectivefMatrix(axisToTest);
      DenseMatrix64F Aeq = helper.getConstraintAeqMatrix(axisToTest);
      DenseMatrix64F beq = helper.getConstraintbeqMatrix(axisToTest);
      qpSolver.setQuadraticCostFunction(H, f, 0.0);
      qpSolver.setLinearEqualityConstraints(Aeq, beq);
//      PrintTools.debug("H: " + H.toString());
//      PrintTools.debug("f: " + f.toString());
//      PrintTools.debug("Aeq: " + Aeq.toString());
//      PrintTools.debug("beq: " + beq.toString());

      DenseMatrix64F soln = new DenseMatrix64F(helper.getNumberOfDecisionVariables(axisToTest), 1);
      try
      {
         qpSolver.solve(soln);
      }
      catch (Exception e)
      {
         assertFalse("Got exception: " + e.getMessage(), true);
      }
      //PrintTools.debug(soln.toString());
      assertTrue("Should have been a finite value: " + soln.get(0, 0), Double.isFinite(soln.get(0, 0)));
      assertTrue("Should have been a finite value: " + soln.get(1, 0), Double.isFinite(soln.get(1, 0)));
      assertEquals(expectedForceValue, soln.get(0, 0), epsilon);
      assertEquals(expectedForceRateValue, soln.get(1, 0), epsilon);
   }

}
