package us.ihmc.commonWalkingControlModules.capturePoint.optimization;

import static org.fest.assertions.Fail.fail;
import static us.ihmc.robotics.Assert.*;

import java.util.ArrayList;
import java.util.List;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.ejml.ops.MatrixFeatures;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.polygonWiggling.PolygonWiggler;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.humanoidRobotics.footstep.FootSpoof;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.referenceFrames.MidFrameZUpFrame;
import us.ihmc.robotics.referenceFrames.ZUpFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

public class ICPOptimizationCoPConstraintHandlerTest
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private static final double footLength = 0.25;
   private static final double stanceWidth = 0.35;

   private final SideDependentList<FramePose3D> footPosesAtTouchdown = new SideDependentList<>(new FramePose3D(), new FramePose3D());
   private final SideDependentList<ReferenceFrame> ankleFrames = new SideDependentList<>();

   @AfterEach
   public void tearDown()
   {
      ReferenceFrameTools.clearWorldFrameTree();
   }

   @Test
   public void testDoubleSupportWithBipedSupportPolygonsAndAngularMomentum()
   {
      YoVariableRegistry registry = new YoVariableRegistry("robert");

      SideDependentList<FootSpoof> contactableFeet = setupContactableFeet(footLength, 0.1, stanceWidth);
      BipedSupportPolygons bipedSupportPolygons = setupBipedSupportPolygons(contactableFeet, registry);
      YoBoolean useControlPolygons = new YoBoolean("useControlPolygons", registry);
      ICPOptimizationCoPConstraintHandler constraintHandler = new ICPOptimizationCoPConstraintHandler(bipedSupportPolygons, null, useControlPolygons, false, registry);
      ICPOptimizationQPSolver solver = new ICPOptimizationQPSolver(5, false);

      solver.resetCoPLocationConstraint();
      solver.addSupportPolygon(constraintHandler.updateCoPConstraintForDoubleSupport());

      solver.setMaxCMPDistanceFromEdge(0.05);
      solver.setCopSafeDistanceToEdge(0.01);

      solver.setFeedbackConditions(0.2, 2.0, 10000.0);
      solver.setCMPFeedbackConditions(10.0, true);
      FrameVector2D currentICPError = new FrameVector2D(worldFrame, 0.01, 0.02);
      FramePoint2D perfectCMP = new FramePoint2D(bipedSupportPolygons.getFootPolygonInWorldFrame(RobotSide.LEFT).getCentroid());
      assertTrue(solver.compute(currentICPError, perfectCMP));

      FrameConvexPolygon2D copConstraint = new FrameConvexPolygon2D();
      FrameConvexPolygon2D cmpConstraint = new FrameConvexPolygon2D();

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

      PolygonWiggler.convertToInequalityConstraints(copConstraint, copAin, copBin, 0.01);
      PolygonWiggler.convertToInequalityConstraints(cmpConstraint, cmpAin, cmpBin, -0.05);
      CommonOps.multAdd(-1.0, copAin, perfectCMPMatrix, copBin);
      CommonOps.multAdd(-1.0, cmpAin, perfectCMPMatrix, cmpBin);

      assertTrue(MatrixFeatures.isEquals(copAin, solver.getCoPLocationConstraint().Aineq, 1e-7));
      assertTrue(MatrixFeatures.isEquals(copBin, solver.getCoPLocationConstraint().bineq, 1e-7));

      assertTrue(MatrixFeatures.isEquals(cmpAin, solver.getCMPLocationConstraint().Aineq, 1e-7));
      assertTrue(MatrixFeatures.isEquals(cmpBin, solver.getCMPLocationConstraint().bineq, 1e-7));
   }

   @Test
   public void testSingleSupportWithBipedSupportPolygonsAndAngularMomentum()
   {
      YoVariableRegistry registry = new YoVariableRegistry("robert");

      SideDependentList<FootSpoof> contactableFeet = setupContactableFeet(footLength, 0.1, stanceWidth);
      BipedSupportPolygons bipedSupportPolygons = setupBipedSupportPolygons(contactableFeet, registry);
      YoBoolean useControlPolygons = new YoBoolean("useControlPolygons", registry);
      ICPOptimizationCoPConstraintHandler constraintHandler = new ICPOptimizationCoPConstraintHandler(bipedSupportPolygons, null, useControlPolygons, false, registry);
      ICPOptimizationQPSolver solver = new ICPOptimizationQPSolver(5, false);

      // test left support
      solver.resetCoPLocationConstraint();
      solver.addSupportPolygon(constraintHandler.updateCoPConstraintForSingleSupport(RobotSide.LEFT));

      solver.setMaxCMPDistanceFromEdge(0.05);
      solver.setCopSafeDistanceToEdge(0.01);

      solver.setFeedbackConditions(0.2, 2.0, 10000.0);
      solver.setCMPFeedbackConditions(10.0, true);
      FrameVector2D currentICPError = new FrameVector2D(worldFrame, 0.01, 0.02);
      FramePoint2D perfectCMP = new FramePoint2D(bipedSupportPolygons.getFootPolygonInWorldFrame(RobotSide.LEFT).getCentroid());
      assertTrue(solver.compute(currentICPError, perfectCMP));

      FrameConvexPolygon2D copConstraint = new FrameConvexPolygon2D();
      FrameConvexPolygon2D cmpConstraint = new FrameConvexPolygon2D();

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

      PolygonWiggler.convertToInequalityConstraints(copConstraint, copAin, copBin, 0.01);
      PolygonWiggler.convertToInequalityConstraints(cmpConstraint, cmpAin, cmpBin, -0.05);
      CommonOps.multAdd(-1.0, copAin, perfectCMPMatrix, copBin);
      CommonOps.multAdd(-1.0, cmpAin, perfectCMPMatrix, cmpBin);

      assertTrue(MatrixFeatures.isEquals(copAin, solver.getCoPLocationConstraint().Aineq, 1e-7));
      assertTrue(MatrixFeatures.isEquals(copBin, solver.getCoPLocationConstraint().bineq, 1e-7));

      assertTrue(MatrixFeatures.isEquals(cmpAin, solver.getCMPLocationConstraint().Aineq, 1e-7));
      assertTrue(MatrixFeatures.isEquals(cmpBin, solver.getCMPLocationConstraint().bineq, 1e-7));


      // test right support
      solver.resetCoPLocationConstraint();
      solver.addSupportPolygon(constraintHandler.updateCoPConstraintForSingleSupport(RobotSide.RIGHT));

      solver.setFeedbackConditions(0.2, 2.0, 10000.0);
      solver.setCMPFeedbackConditions(10.0, true);
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

      copConstraint = new FrameConvexPolygon2D();
      cmpConstraint = new FrameConvexPolygon2D();

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

      PolygonWiggler.convertToInequalityConstraints(copConstraint, copAin, copBin, 0.01);
      PolygonWiggler.convertToInequalityConstraints(cmpConstraint, cmpAin, cmpBin, -0.05);
      CommonOps.multAdd(-1.0, copAin, perfectCMPMatrix, copBin);
      CommonOps.multAdd(-1.0, cmpAin, perfectCMPMatrix, cmpBin);

      assertTrue(MatrixFeatures.isEquals(copAin, solver.getCoPLocationConstraint().Aineq, 1e-7));
      assertTrue(MatrixFeatures.isEquals(copBin, solver.getCoPLocationConstraint().bineq, 1e-7));

      assertTrue(MatrixFeatures.isEquals(cmpAin, solver.getCMPLocationConstraint().Aineq, 1e-7));
      assertTrue(MatrixFeatures.isEquals(cmpBin, solver.getCMPLocationConstraint().bineq, 1e-7));
   }

   @Test
   public void testDoubleSupportWithBipedSupportPolygonsNoAngularMomentum()
   {
      YoVariableRegistry registry = new YoVariableRegistry("robert");

      SideDependentList<FootSpoof> contactableFeet = setupContactableFeet(footLength, 0.1, stanceWidth);
      BipedSupportPolygons bipedSupportPolygons = setupBipedSupportPolygons(contactableFeet, registry);
      YoBoolean useControlPolygons = new YoBoolean("useControlPolygons", registry);
      ICPOptimizationCoPConstraintHandler constraintHandler = new ICPOptimizationCoPConstraintHandler(bipedSupportPolygons, null, useControlPolygons, false, registry);
      ICPOptimizationQPSolver solver = new ICPOptimizationQPSolver(5, false);

      solver.resetCoPLocationConstraint();
      solver.addSupportPolygon(constraintHandler.updateCoPConstraintForDoubleSupport());

      solver.setMaxCMPDistanceFromEdge(0.05);
      solver.setCopSafeDistanceToEdge(0.01);

      solver.setFeedbackConditions(0.2, 2.0, 10000.0);
      FrameVector2D currentICPError = new FrameVector2D(worldFrame, 0.01, 0.02);
      FramePoint2D perfectCMP = new FramePoint2D(bipedSupportPolygons.getFootPolygonInWorldFrame(RobotSide.LEFT).getCentroid());
      assertTrue(solver.compute(currentICPError, perfectCMP));

      FrameConvexPolygon2D copConstraint = new FrameConvexPolygon2D();
      FrameConvexPolygon2D cmpConstraint = new FrameConvexPolygon2D();

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

      PolygonWiggler.convertToInequalityConstraints(copConstraint, copAin, copBin, 0.01);
      CommonOps.multAdd(-1.0, copAin, perfectCMPMatrix, copBin);

      assertTrue(MatrixFeatures.isEquals(copAin, solver.getCoPLocationConstraint().Aineq, 1e-7));
      assertTrue(MatrixFeatures.isEquals(copBin, solver.getCoPLocationConstraint().bineq, 1e-7));

      assertEquals(CommonOps.elementSum(solver.getCMPLocationConstraint().Aineq), 0.0, 1e-7);
      assertEquals(CommonOps.elementSum(solver.getCMPLocationConstraint().bineq), 0.0, 1e-7);
   }

   @Test
   public void testSingleSupportWithBipedSupportPolygonsNoAngularMomentum()
   {
      YoVariableRegistry registry = new YoVariableRegistry("robert");

      SideDependentList<FootSpoof> contactableFeet = setupContactableFeet(footLength, 0.1, stanceWidth);
      BipedSupportPolygons bipedSupportPolygons = setupBipedSupportPolygons(contactableFeet, registry);
      YoBoolean useControlPolygons = new YoBoolean("useControlPolygons", registry);
      ICPOptimizationCoPConstraintHandler constraintHandler = new ICPOptimizationCoPConstraintHandler(bipedSupportPolygons, null, useControlPolygons, false, registry);
      ICPOptimizationQPSolver solver = new ICPOptimizationQPSolver(5, false);

      // test left support
      solver.resetCoPLocationConstraint();
      solver.addSupportPolygon(constraintHandler.updateCoPConstraintForSingleSupport(RobotSide.LEFT));

      solver.setMaxCMPDistanceFromEdge(0.05);
      solver.setCopSafeDistanceToEdge(0.01);

      solver.setFeedbackConditions(0.2, 2.0, 10000.0);
      FrameVector2D currentICPError = new FrameVector2D(worldFrame, 0.01, 0.02);
      FramePoint2D perfectCMP = new FramePoint2D(bipedSupportPolygons.getFootPolygonInWorldFrame(RobotSide.LEFT).getCentroid());
      assertTrue(solver.compute(currentICPError, perfectCMP));

      FrameConvexPolygon2D copConstraint = new FrameConvexPolygon2D();
      FrameConvexPolygon2D cmpConstraint = new FrameConvexPolygon2D();

      copConstraint.addVertices(bipedSupportPolygons.getFootPolygonInWorldFrame(RobotSide.LEFT));
      cmpConstraint.addVertices(bipedSupportPolygons.getFootPolygonInWorldFrame(RobotSide.LEFT));

      copConstraint.update();
      cmpConstraint.update();

      DenseMatrix64F copAin = new DenseMatrix64F(copConstraint.getNumberOfVertices(), 2);
      DenseMatrix64F copBin = new DenseMatrix64F(copConstraint.getNumberOfVertices(), 1);

      DenseMatrix64F perfectCMPMatrix = new DenseMatrix64F(2, 1);
      perfectCMP.get(perfectCMPMatrix);

      PolygonWiggler.convertToInequalityConstraints(copConstraint, copAin, copBin, 0.01);
      CommonOps.multAdd(-1.0, copAin, perfectCMPMatrix, copBin);

      assertTrue(MatrixFeatures.isEquals(copAin, solver.getCoPLocationConstraint().Aineq, 1e-7));
      assertTrue(MatrixFeatures.isEquals(copBin, solver.getCoPLocationConstraint().bineq, 1e-7));

      assertEquals(CommonOps.elementSum(solver.getCMPLocationConstraint().Aineq), 0.0, 1e-7);
      assertEquals(CommonOps.elementSum(solver.getCMPLocationConstraint().bineq), 0.0, 1e-7);


      // test right support
      solver.resetCoPLocationConstraint();
      solver.addSupportPolygon(constraintHandler.updateCoPConstraintForSingleSupport(RobotSide.RIGHT));

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

      copConstraint = new FrameConvexPolygon2D();
      cmpConstraint = new FrameConvexPolygon2D();

      copConstraint.addVertices(bipedSupportPolygons.getFootPolygonInWorldFrame(RobotSide.RIGHT));
      cmpConstraint.addVertices(bipedSupportPolygons.getFootPolygonInWorldFrame(RobotSide.RIGHT));

      copConstraint.update();
      cmpConstraint.update();

      copAin = new DenseMatrix64F(copConstraint.getNumberOfVertices(), 2);
      copBin = new DenseMatrix64F(copConstraint.getNumberOfVertices(), 1);

      perfectCMPMatrix = new DenseMatrix64F(2, 1);
      perfectCMP.get(perfectCMPMatrix);

      PolygonWiggler.convertToInequalityConstraints(copConstraint, copAin, copBin, 0.01);
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
      SideDependentList<ReferenceFrame> soleFrames = new SideDependentList<>();
      SideDependentList<YoPlaneContactState> contactStates = new SideDependentList<>();

      for (RobotSide robotSide : RobotSide.values)
      {
         FootSpoof contactableFoot = contactableFeet.get(robotSide);
         ReferenceFrame ankleFrame = contactableFoot.getFrameAfterParentJoint();
         ankleFrames.put(robotSide, ankleFrame);
         ankleZUpFrames.put(robotSide, new ZUpFrame(worldFrame, ankleFrame, robotSide.getCamelCaseNameForStartOfExpression() + "ZUp"));

         String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
         RigidBodyBasics foot = contactableFoot.getRigidBody();
         ReferenceFrame soleFrame = contactableFoot.getSoleFrame();
         soleFrames.put(robotSide, soleFrame);
         List<FramePoint2D> contactFramePoints = contactableFoot.getContactPoints2d();
         double coefficientOfFriction = contactableFoot.getCoefficientOfFriction();
         YoPlaneContactState yoPlaneContactState = new YoPlaneContactState(sidePrefix + "Foot", foot, soleFrame, contactFramePoints, coefficientOfFriction, registry);
         yoPlaneContactState.setFullyConstrained();
         contactStates.put(robotSide, yoPlaneContactState);
      }

      ReferenceFrame midFeetZUpFrame = new MidFrameZUpFrame("midFeetZupFrame", worldFrame, ankleZUpFrames.get(RobotSide.LEFT), ankleZUpFrames.get(RobotSide.RIGHT));
      midFeetZUpFrame.update();

      BipedSupportPolygons bipedSupportPolygons = new BipedSupportPolygons(midFeetZUpFrame, ankleZUpFrames, soleFrames, registry, null);
      bipedSupportPolygons.updateUsingContactStates(contactStates);

      return bipedSupportPolygons;
   }

}
