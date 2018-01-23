package us.ihmc.commonWalkingControlModules.capturePoint.optimization;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.ejml.ops.MatrixFeatures;
import org.junit.Test;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.footstepPlanning.polygonWiggling.PolygonWiggler;
import us.ihmc.humanoidRobotics.footstep.FootSpoof;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.referenceFrames.MidFrameZUpFrame;
import us.ihmc.robotics.referenceFrames.ZUpFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

import java.util.ArrayList;
import java.util.List;

import static org.fest.assertions.Fail.fail;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

@ContinuousIntegrationPlan(categories = {IntegrationCategory.FAST})
public class ICPOptimizationCoPConstraintHandlerTest
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private static final double footLength = 0.25;
   private static final double stanceWidth = 0.35;

   private final SideDependentList<FramePose3D> footPosesAtTouchdown = new SideDependentList<>(new FramePose3D(), new FramePose3D());
   private final SideDependentList<ReferenceFrame> ankleFrames = new SideDependentList<>();

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 21000)
   public void testDoubleSupportWithBipedSupportPolygonsAndAngularMomentum()
   {
      YoVariableRegistry registry = new YoVariableRegistry("robert");

      SideDependentList<FootSpoof> contactableFeet = setupContactableFeet(footLength, 0.1, stanceWidth);
      BipedSupportPolygons bipedSupportPolygons = setupBipedSupportPolygons(contactableFeet, registry);
      YoBoolean useControlPolygons = new YoBoolean("useControlPolygons", registry);
      ICPOptimizationCoPConstraintHandler constraintHandler = new ICPOptimizationCoPConstraintHandler(bipedSupportPolygons, null, useControlPolygons, registry);
      ICPOptimizationParameters parameters = new TestICPOptimizationParameters();
      ICPOptimizationQPSolver solver = new ICPOptimizationQPSolver(parameters, 5, false);

      constraintHandler.updateCoPConstraintForDoubleSupport(solver);
      solver.setMaxCMPDistanceFromEdge(0.05);
      solver.setCopSafeDistanceToEdge(0.01);

      solver.setFeedbackConditions(0.2, 2.0, 10000.0);
      solver.setAngularMomentumConditions(10.0, true);
      FrameVector2D currentICPError = new FrameVector2D(worldFrame, 0.01, 0.02);
      FramePoint2D perfectCMP = new FramePoint2D(bipedSupportPolygons.getFootPolygonInWorldFrame(RobotSide.LEFT).getCentroid());
      try
      {
         solver.compute(currentICPError, perfectCMP);
      }
      catch (Exception e)
      {
         fail();
      }

      FrameConvexPolygon2d copConstraint = new FrameConvexPolygon2d();
      FrameConvexPolygon2d cmpConstraint = new FrameConvexPolygon2d();

      for (RobotSide robotSide : RobotSide.values)
      {
         copConstraint.addVertices(bipedSupportPolygons.getFootPolygonInWorldFrame(robotSide));
         cmpConstraint.addVertices(bipedSupportPolygons.getFootPolygonInWorldFrame(robotSide));
      }

      copConstraint.update();
      cmpConstraint.update();

      DenseMatrix64F copAin = new DenseMatrix64F(copConstraint.getNumberOfVertices(), 2);
      DenseMatrix64F copBin = new DenseMatrix64F(copConstraint.getNumberOfVertices(), 1);

      DenseMatrix64F cmpAin = new DenseMatrix64F(copConstraint.getNumberOfVertices(), 2);
      DenseMatrix64F cmpBin = new DenseMatrix64F(copConstraint.getNumberOfVertices(), 1);

      DenseMatrix64F perfectCMPMatrix = new DenseMatrix64F(2, 1);
      perfectCMP.get(perfectCMPMatrix);

      PolygonWiggler.convertToInequalityConstraints(copConstraint.getConvexPolygon2d(), copAin, copBin, 0.01);
      PolygonWiggler.convertToInequalityConstraints(cmpConstraint.getConvexPolygon2d(), cmpAin, cmpBin, -0.05);
      CommonOps.multAdd(-1.0, copAin, perfectCMPMatrix, copBin);
      CommonOps.multAdd(-1.0, cmpAin, perfectCMPMatrix, cmpBin);

      assertTrue(MatrixFeatures.isEquals(copAin, solver.getCoPLocationConstraint().Aineq, 1e-7));
      assertTrue(MatrixFeatures.isEquals(copBin, solver.getCoPLocationConstraint().bineq, 1e-7));

      assertTrue(MatrixFeatures.isEquals(cmpAin, solver.getCMPLocationConstraint().Aineq, 1e-7));
      assertTrue(MatrixFeatures.isEquals(cmpBin, solver.getCMPLocationConstraint().bineq, 1e-7));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 21000)
   public void testSingleSupportWithBipedSupportPolygonsAndAngularMomentum()
   {
      YoVariableRegistry registry = new YoVariableRegistry("robert");

      SideDependentList<FootSpoof> contactableFeet = setupContactableFeet(footLength, 0.1, stanceWidth);
      BipedSupportPolygons bipedSupportPolygons = setupBipedSupportPolygons(contactableFeet, registry);
      YoBoolean useControlPolygons = new YoBoolean("useControlPolygons", registry);
      ICPOptimizationCoPConstraintHandler constraintHandler = new ICPOptimizationCoPConstraintHandler(bipedSupportPolygons, null, useControlPolygons, registry);
      ICPOptimizationParameters parameters = new TestICPOptimizationParameters();
      ICPOptimizationQPSolver solver = new ICPOptimizationQPSolver(parameters, 5, false);

      // test left support
      constraintHandler.updateCoPConstraintForSingleSupport(RobotSide.LEFT, solver);
      solver.setMaxCMPDistanceFromEdge(0.05);
      solver.setCopSafeDistanceToEdge(0.01);

      solver.setFeedbackConditions(0.2, 2.0, 10000.0);
      solver.setAngularMomentumConditions(10.0, true);
      FrameVector2D currentICPError = new FrameVector2D(worldFrame, 0.01, 0.02);
      FramePoint2D perfectCMP = new FramePoint2D(bipedSupportPolygons.getFootPolygonInWorldFrame(RobotSide.LEFT).getCentroid());
      try
      {
         solver.compute(currentICPError, perfectCMP);
      }
      catch (Exception e)
      {
         fail();
      }

      FrameConvexPolygon2d copConstraint = new FrameConvexPolygon2d();
      FrameConvexPolygon2d cmpConstraint = new FrameConvexPolygon2d();

      copConstraint.addVertices(bipedSupportPolygons.getFootPolygonInWorldFrame(RobotSide.LEFT));
      cmpConstraint.addVertices(bipedSupportPolygons.getFootPolygonInWorldFrame(RobotSide.LEFT));

      copConstraint.update();
      cmpConstraint.update();

      DenseMatrix64F copAin = new DenseMatrix64F(copConstraint.getNumberOfVertices(), 2);
      DenseMatrix64F copBin = new DenseMatrix64F(copConstraint.getNumberOfVertices(), 1);

      DenseMatrix64F cmpAin = new DenseMatrix64F(copConstraint.getNumberOfVertices(), 2);
      DenseMatrix64F cmpBin = new DenseMatrix64F(copConstraint.getNumberOfVertices(), 1);

      DenseMatrix64F perfectCMPMatrix = new DenseMatrix64F(2, 1);
      perfectCMP.get(perfectCMPMatrix);

      PolygonWiggler.convertToInequalityConstraints(copConstraint.getConvexPolygon2d(), copAin, copBin, 0.01);
      PolygonWiggler.convertToInequalityConstraints(cmpConstraint.getConvexPolygon2d(), cmpAin, cmpBin, -0.05);
      CommonOps.multAdd(-1.0, copAin, perfectCMPMatrix, copBin);
      CommonOps.multAdd(-1.0, cmpAin, perfectCMPMatrix, cmpBin);

      assertTrue(MatrixFeatures.isEquals(copAin, solver.getCoPLocationConstraint().Aineq, 1e-7));
      assertTrue(MatrixFeatures.isEquals(copBin, solver.getCoPLocationConstraint().bineq, 1e-7));

      assertTrue(MatrixFeatures.isEquals(cmpAin, solver.getCMPLocationConstraint().Aineq, 1e-7));
      assertTrue(MatrixFeatures.isEquals(cmpBin, solver.getCMPLocationConstraint().bineq, 1e-7));


      // test right support
      constraintHandler.updateCoPConstraintForSingleSupport(RobotSide.RIGHT, solver);
      solver.setFeedbackConditions(0.2, 2.0, 10000.0);
      solver.setAngularMomentumConditions(10.0, true);
      currentICPError = new FrameVector2D(worldFrame, 0.01, 0.02);
      perfectCMP = new FramePoint2D(bipedSupportPolygons.getFootPolygonInWorldFrame(RobotSide.RIGHT).getCentroid());
      try
      {
         solver.compute(currentICPError, perfectCMP);
      }
      catch (Exception e)
      {
         fail();
      }

      copConstraint = new FrameConvexPolygon2d();
      cmpConstraint = new FrameConvexPolygon2d();

      copConstraint.addVertices(bipedSupportPolygons.getFootPolygonInWorldFrame(RobotSide.RIGHT));
      cmpConstraint.addVertices(bipedSupportPolygons.getFootPolygonInWorldFrame(RobotSide.RIGHT));

      copConstraint.update();
      cmpConstraint.update();

      copAin = new DenseMatrix64F(copConstraint.getNumberOfVertices(), 2);
      copBin = new DenseMatrix64F(copConstraint.getNumberOfVertices(), 1);

      cmpAin = new DenseMatrix64F(copConstraint.getNumberOfVertices(), 2);
      cmpBin = new DenseMatrix64F(copConstraint.getNumberOfVertices(), 1);

      perfectCMPMatrix = new DenseMatrix64F(2, 1);
      perfectCMP.get(perfectCMPMatrix);

      PolygonWiggler.convertToInequalityConstraints(copConstraint.getConvexPolygon2d(), copAin, copBin, 0.01);
      PolygonWiggler.convertToInequalityConstraints(cmpConstraint.getConvexPolygon2d(), cmpAin, cmpBin, -0.05);
      CommonOps.multAdd(-1.0, copAin, perfectCMPMatrix, copBin);
      CommonOps.multAdd(-1.0, cmpAin, perfectCMPMatrix, cmpBin);

      assertTrue(MatrixFeatures.isEquals(copAin, solver.getCoPLocationConstraint().Aineq, 1e-7));
      assertTrue(MatrixFeatures.isEquals(copBin, solver.getCoPLocationConstraint().bineq, 1e-7));

      assertTrue(MatrixFeatures.isEquals(cmpAin, solver.getCMPLocationConstraint().Aineq, 1e-7));
      assertTrue(MatrixFeatures.isEquals(cmpBin, solver.getCMPLocationConstraint().bineq, 1e-7));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 21000)
   public void testDoubleSupportWithBipedSupportPolygonsNoAngularMomentum()
   {
      YoVariableRegistry registry = new YoVariableRegistry("robert");

      SideDependentList<FootSpoof> contactableFeet = setupContactableFeet(footLength, 0.1, stanceWidth);
      BipedSupportPolygons bipedSupportPolygons = setupBipedSupportPolygons(contactableFeet, registry);
      YoBoolean useControlPolygons = new YoBoolean("useControlPolygons", registry);
      ICPOptimizationCoPConstraintHandler constraintHandler = new ICPOptimizationCoPConstraintHandler(bipedSupportPolygons, null, useControlPolygons, registry);
      ICPOptimizationParameters parameters = new TestICPOptimizationParameters();
      ICPOptimizationQPSolver solver = new ICPOptimizationQPSolver(parameters, 5, false);

      constraintHandler.updateCoPConstraintForDoubleSupport(solver);
      solver.setMaxCMPDistanceFromEdge(0.05);
      solver.setCopSafeDistanceToEdge(0.01);

      solver.setFeedbackConditions(0.2, 2.0, 10000.0);
      FrameVector2D currentICPError = new FrameVector2D(worldFrame, 0.01, 0.02);
      FramePoint2D perfectCMP = new FramePoint2D(bipedSupportPolygons.getFootPolygonInWorldFrame(RobotSide.LEFT).getCentroid());
      try
      {
         solver.compute(currentICPError, perfectCMP);
      }
      catch (Exception e)
      {
         fail();
      }

      FrameConvexPolygon2d copConstraint = new FrameConvexPolygon2d();
      FrameConvexPolygon2d cmpConstraint = new FrameConvexPolygon2d();

      for (RobotSide robotSide : RobotSide.values)
      {
         copConstraint.addVertices(bipedSupportPolygons.getFootPolygonInWorldFrame(robotSide));
         cmpConstraint.addVertices(bipedSupportPolygons.getFootPolygonInWorldFrame(robotSide));
      }

      copConstraint.update();
      cmpConstraint.update();

      DenseMatrix64F copAin = new DenseMatrix64F(copConstraint.getNumberOfVertices(), 2);
      DenseMatrix64F copBin = new DenseMatrix64F(copConstraint.getNumberOfVertices(), 1);

      DenseMatrix64F perfectCMPMatrix = new DenseMatrix64F(2, 1);
      perfectCMP.get(perfectCMPMatrix);

      PolygonWiggler.convertToInequalityConstraints(copConstraint.getConvexPolygon2d(), copAin, copBin, 0.01);
      CommonOps.multAdd(-1.0, copAin, perfectCMPMatrix, copBin);

      assertTrue(MatrixFeatures.isEquals(copAin, solver.getCoPLocationConstraint().Aineq, 1e-7));
      assertTrue(MatrixFeatures.isEquals(copBin, solver.getCoPLocationConstraint().bineq, 1e-7));

      assertEquals(CommonOps.elementSum(solver.getCMPLocationConstraint().Aineq), 0.0, 1e-7);
      assertEquals(CommonOps.elementSum(solver.getCMPLocationConstraint().bineq), 0.0, 1e-7);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 21000)
   public void testSingleSupportWithBipedSupportPolygonsNoAngularMomentum()
   {
      YoVariableRegistry registry = new YoVariableRegistry("robert");

      SideDependentList<FootSpoof> contactableFeet = setupContactableFeet(footLength, 0.1, stanceWidth);
      BipedSupportPolygons bipedSupportPolygons = setupBipedSupportPolygons(contactableFeet, registry);
      YoBoolean useControlPolygons = new YoBoolean("useControlPolygons", registry);
      ICPOptimizationCoPConstraintHandler constraintHandler = new ICPOptimizationCoPConstraintHandler(bipedSupportPolygons, null, useControlPolygons, registry);
      ICPOptimizationParameters parameters = new TestICPOptimizationParameters();
      ICPOptimizationQPSolver solver = new ICPOptimizationQPSolver(parameters, 5, false);

      // test left support
      constraintHandler.updateCoPConstraintForSingleSupport(RobotSide.LEFT, solver);
      solver.setMaxCMPDistanceFromEdge(0.05);
      solver.setCopSafeDistanceToEdge(0.01);

      solver.setFeedbackConditions(0.2, 2.0, 10000.0);
      FrameVector2D currentICPError = new FrameVector2D(worldFrame, 0.01, 0.02);
      FramePoint2D perfectCMP = new FramePoint2D(bipedSupportPolygons.getFootPolygonInWorldFrame(RobotSide.LEFT).getCentroid());
      try
      {
         solver.compute(currentICPError, perfectCMP);
      }
      catch (Exception e)
      {
         fail();
      }

      FrameConvexPolygon2d copConstraint = new FrameConvexPolygon2d();
      FrameConvexPolygon2d cmpConstraint = new FrameConvexPolygon2d();

      copConstraint.addVertices(bipedSupportPolygons.getFootPolygonInWorldFrame(RobotSide.LEFT));
      cmpConstraint.addVertices(bipedSupportPolygons.getFootPolygonInWorldFrame(RobotSide.LEFT));

      copConstraint.update();
      cmpConstraint.update();

      DenseMatrix64F copAin = new DenseMatrix64F(copConstraint.getNumberOfVertices(), 2);
      DenseMatrix64F copBin = new DenseMatrix64F(copConstraint.getNumberOfVertices(), 1);

      DenseMatrix64F perfectCMPMatrix = new DenseMatrix64F(2, 1);
      perfectCMP.get(perfectCMPMatrix);

      PolygonWiggler.convertToInequalityConstraints(copConstraint.getConvexPolygon2d(), copAin, copBin, 0.01);
      CommonOps.multAdd(-1.0, copAin, perfectCMPMatrix, copBin);

      assertTrue(MatrixFeatures.isEquals(copAin, solver.getCoPLocationConstraint().Aineq, 1e-7));
      assertTrue(MatrixFeatures.isEquals(copBin, solver.getCoPLocationConstraint().bineq, 1e-7));

      assertEquals(CommonOps.elementSum(solver.getCMPLocationConstraint().Aineq), 0.0, 1e-7);
      assertEquals(CommonOps.elementSum(solver.getCMPLocationConstraint().bineq), 0.0, 1e-7);


      // test right support
      constraintHandler.updateCoPConstraintForSingleSupport(RobotSide.RIGHT, solver);
      solver.setFeedbackConditions(0.2, 2.0, 10000.0);
      currentICPError = new FrameVector2D(worldFrame, 0.01, 0.02);
      perfectCMP = new FramePoint2D(bipedSupportPolygons.getFootPolygonInWorldFrame(RobotSide.RIGHT).getCentroid());
      try
      {
         solver.compute(currentICPError, perfectCMP);
      }
      catch (Exception e)
      {
         fail();
      }

      copConstraint = new FrameConvexPolygon2d();
      cmpConstraint = new FrameConvexPolygon2d();

      copConstraint.addVertices(bipedSupportPolygons.getFootPolygonInWorldFrame(RobotSide.RIGHT));
      cmpConstraint.addVertices(bipedSupportPolygons.getFootPolygonInWorldFrame(RobotSide.RIGHT));

      copConstraint.update();
      cmpConstraint.update();

      copAin = new DenseMatrix64F(copConstraint.getNumberOfVertices(), 2);
      copBin = new DenseMatrix64F(copConstraint.getNumberOfVertices(), 1);

      perfectCMPMatrix = new DenseMatrix64F(2, 1);
      perfectCMP.get(perfectCMPMatrix);

      PolygonWiggler.convertToInequalityConstraints(copConstraint.getConvexPolygon2d(), copAin, copBin, 0.01);
      CommonOps.multAdd(-1.0, copAin, perfectCMPMatrix, copBin);

      assertTrue(MatrixFeatures.isEquals(copAin, solver.getCoPLocationConstraint().Aineq, 1e-7));
      assertTrue(MatrixFeatures.isEquals(copBin, solver.getCoPLocationConstraint().bineq, 1e-7));

      assertEquals(CommonOps.elementSum(solver.getCMPLocationConstraint().Aineq), 0.0, 1e-7);
      assertEquals(CommonOps.elementSum(solver.getCMPLocationConstraint().bineq), 0.0, 1e-7);
   }

   private SideDependentList<FootSpoof> setupContactableFeet(double footLength, double footWidth, double totalWidth)
   {
      SideDependentList<FootSpoof> contactableFeet = new SideDependentList<>();

      for (RobotSide robotSide : RobotSide.values)
      {
         String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
         double xToAnkle = 0.0;
         double yToAnkle = 0.0;
         double zToAnkle = 0.084;

         List<Point2D> contactPointsInSoleFrame = new ArrayList<>();
         contactPointsInSoleFrame.add(new Point2D(footLength / 2.0, footWidth / 2.0));
         contactPointsInSoleFrame.add(new Point2D(footLength / 2.0, -footWidth / 2.0));
         contactPointsInSoleFrame.add(new Point2D(-footLength / 2.0, -footWidth / 2.0));
         contactPointsInSoleFrame.add(new Point2D(-footLength / 2.0, footWidth / 2.0));

         FootSpoof contactableFoot = new FootSpoof(sidePrefix + "Foot", xToAnkle, yToAnkle, zToAnkle, contactPointsInSoleFrame, 0.0);
         FramePose3D startingPose = footPosesAtTouchdown.get(robotSide);
         startingPose.setToZero(worldFrame);
         startingPose.setY(robotSide.negateIfRightSide(0.5 * (totalWidth - footWidth)));
         contactableFoot.setSoleFrame(startingPose);

         contactableFeet.put(robotSide, contactableFoot);
      }

      return contactableFeet;
   }

   private BipedSupportPolygons setupBipedSupportPolygons(SideDependentList<FootSpoof> contactableFeet, YoVariableRegistry registry)
   {
      SideDependentList<ReferenceFrame> ankleZUpFrames = new SideDependentList<>();
      SideDependentList<YoPlaneContactState> contactStates = new SideDependentList<>();

      for (RobotSide robotSide : RobotSide.values)
      {
         FootSpoof contactableFoot = contactableFeet.get(robotSide);
         ReferenceFrame ankleFrame = contactableFoot.getFrameAfterParentJoint();
         ankleFrames.put(robotSide, ankleFrame);
         ankleZUpFrames.put(robotSide, new ZUpFrame(worldFrame, ankleFrame, robotSide.getCamelCaseNameForStartOfExpression() + "ZUp"));

         String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
         RigidBody foot = contactableFoot.getRigidBody();
         ReferenceFrame soleFrame = contactableFoot.getSoleFrame();
         List<FramePoint2D> contactFramePoints = contactableFoot.getContactPoints2d();
         double coefficientOfFriction = contactableFoot.getCoefficientOfFriction();
         YoPlaneContactState yoPlaneContactState = new YoPlaneContactState(sidePrefix + "Foot", foot, soleFrame, contactFramePoints, coefficientOfFriction, registry);
         yoPlaneContactState.setFullyConstrained();
         contactStates.put(robotSide, yoPlaneContactState);
      }

      ReferenceFrame midFeetZUpFrame = new MidFrameZUpFrame("midFeetZupFrame", worldFrame, ankleZUpFrames.get(RobotSide.LEFT), ankleZUpFrames.get(RobotSide.RIGHT));
      midFeetZUpFrame.update();

      BipedSupportPolygons bipedSupportPolygons = new BipedSupportPolygons(ankleZUpFrames, midFeetZUpFrame, ankleZUpFrames, registry, null);
      bipedSupportPolygons.updateUsingContactStates(contactStates);

      return bipedSupportPolygons;
   }

   private class TestICPOptimizationParameters extends ICPOptimizationParameters
   {
      @Override
      public double getForwardFootstepWeight()
      {
         return 10.0;
      }

      @Override
      public double getLateralFootstepWeight()
      {
         return 10.0;
      }

      @Override
      public double getFootstepRateWeight()
      {
         return 0.0001;
      }

      @Override
      public double getFeedbackForwardWeight()
      {
         return 0.5;
      }

      @Override
      public double getFeedbackLateralWeight()
      {
         return 0.5;
      }

      @Override
      public double getFeedbackRateWeight()
      {
         return 0.0001;
      }

      @Override
      public double getFeedbackParallelGain()
      {
         return 3.0;
      }

      @Override
      public double getFeedbackOrthogonalGain()
      {
         return 2.5;
      }

      @Override
      public double getDynamicsObjectiveWeight()
      {
         return 500.0;
      }

      @Override
      public double getDynamicsObjectiveDoubleSupportWeightModifier()
      {
         return 1.0;
      }

      @Override
      public double getAngularMomentumMinimizationWeight()
      {
         return 50;
      }

      @Override
      public boolean scaleStepRateWeightWithTime()
      {
         return false;
      }

      @Override
      public boolean scaleFeedbackWeightWithGain()
      {
         return false;
      }

      @Override
      public boolean useFeedbackRate()
      {
         return false;
      }

      @Override
      public boolean useStepAdjustment()
      {
         return false;
      }

      @Override
      public boolean useAngularMomentum()
      {
         return false;
      }

      @Override
      public boolean useFootstepRate()
      {
         return false;
      }

      @Override
      public double getMinimumFootstepWeight()
      {
         return 0.0001;
      }

      @Override
      public double getMinimumFeedbackWeight()
      {
         return 0.0001;
      }


      @Override
      public double getMinimumTimeRemaining()
      {
         return 0.0001;
      }

      @Override
      public double getAdjustmentDeadband()
      {
         return 0.03;
      }

      @Override
      public double getSafeCoPDistanceToEdge()
      {
         return 0.0;
      }
   }
}
