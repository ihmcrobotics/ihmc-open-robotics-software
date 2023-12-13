package us.ihmc.commonWalkingControlModules.capturePoint.controller;

import static org.fest.assertions.Fail.fail;
import static us.ihmc.robotics.Assert.assertEquals;
import static us.ihmc.robotics.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;

import org.ejml.EjmlUnitTests;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.MatrixFeatures_DDRM;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.polygonWiggling.PolygonWiggler;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.humanoidRobotics.footstep.FootSpoof;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.geometry.ConvexPolygonScaler;
import us.ihmc.robotics.referenceFrames.MidFrameZUpFrame;
import us.ihmc.robotics.referenceFrames.ZUpFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

public class ICPCoPConstraintHandlerTest
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
      YoRegistry registry = new YoRegistry("robert");

      SideDependentList<FootSpoof> contactableFeet = setupContactableFeet(footLength, 0.1, stanceWidth);
      SideDependentList<YoPlaneContactState> contactStates = new SideDependentList<>();
      BipedSupportPolygons bipedSupportPolygons = setupBipedSupportPolygons(contactableFeet, registry, contactStates);
      YoBoolean useControlPolygons = new YoBoolean("useControlPolygons", registry);
      ICPCoPConstraintHandler constraintHandler = new ICPCoPConstraintHandler(null, useControlPolygons, false, registry);
      ICPControllerQPSolver solver = new ICPControllerQPSolver(5, false, null);

      solver.resetCoPLocationConstraint();
      solver.addSupportPolygon(constraintHandler.updateCoPConstraint(bipedSupportPolygons.getSupportPolygonInWorld()));

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

      DMatrixRMaj copAin = new DMatrixRMaj(copConstraint.getNumberOfVertices(), 2);
      DMatrixRMaj copBin = new DMatrixRMaj(copConstraint.getNumberOfVertices(), 1);

      DMatrixRMaj cmpAin = new DMatrixRMaj(copConstraint.getNumberOfVertices(), 2);
      DMatrixRMaj cmpBin = new DMatrixRMaj(copConstraint.getNumberOfVertices(), 1);

      ConvexPolygonScaler scaler = new ConvexPolygonScaler();
      ConvexPolygon2D scaledCoPConstraint = new ConvexPolygon2D();
      ConvexPolygon2D scaledCMPConstraint = new ConvexPolygon2D();

      scaler.scaleConvexPolygon(copConstraint, 0.01, scaledCoPConstraint);
      scaler.scaleConvexPolygon(cmpConstraint, -0.05, scaledCMPConstraint);

      DMatrixRMaj perfectCMPMatrix = new DMatrixRMaj(2, 1);
      perfectCMP.get(perfectCMPMatrix);

      PolygonWiggler.convertToInequalityConstraints(scaledCoPConstraint, copAin, copBin, 0.0);
      PolygonWiggler.convertToInequalityConstraints(scaledCMPConstraint, cmpAin, cmpBin, 0.0);
      CommonOps_DDRM.multAdd(-1.0, copAin, perfectCMPMatrix, copBin);
      CommonOps_DDRM.multAdd(-1.0, cmpAin, perfectCMPMatrix, cmpBin);

      EjmlUnitTests.assertEquals(copAin, solver.getCoPLocationConstraint().Aineq, 1e-7);
      EjmlUnitTests.assertEquals(copBin, solver.getCoPLocationConstraint().bineq, 1e-7);

      EjmlUnitTests.assertEquals(cmpAin, solver.getCMPLocationConstraint().Aineq, 1e-7);
      EjmlUnitTests.assertEquals(cmpBin, solver.getCMPLocationConstraint().bineq, 1e-7);
   }

   @Test
   public void testSingleSupportWithBipedSupportPolygonsAndAngularMomentum()
   {
      YoRegistry registry = new YoRegistry("robert");

      SideDependentList<FootSpoof> contactableFeet = setupContactableFeet(footLength, 0.1, stanceWidth);
      SideDependentList<YoPlaneContactState> contactStates = new SideDependentList<>();
      BipedSupportPolygons bipedSupportPolygons = setupBipedSupportPolygons(contactableFeet, registry, contactStates);
      contactStates.get(RobotSide.RIGHT).getContactPoints().forEach(point -> point.setInContact(false));
      bipedSupportPolygons.updateUsingContactStates(contactStates);

      YoBoolean useControlPolygons = new YoBoolean("useControlPolygons", registry);
      ICPCoPConstraintHandler constraintHandler = new ICPCoPConstraintHandler(null, useControlPolygons, false, registry);
      ICPControllerQPSolver solver = new ICPControllerQPSolver(5, false, null);

      // test left support
      solver.resetCoPLocationConstraint();
      solver.addSupportPolygon(constraintHandler.updateCoPConstraint(bipedSupportPolygons.getSupportPolygonInWorld()));

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

      DMatrixRMaj copAin = new DMatrixRMaj(copConstraint.getNumberOfVertices(), 2);
      DMatrixRMaj copBin = new DMatrixRMaj(copConstraint.getNumberOfVertices(), 1);

      DMatrixRMaj cmpAin = new DMatrixRMaj(copConstraint.getNumberOfVertices(), 2);
      DMatrixRMaj cmpBin = new DMatrixRMaj(copConstraint.getNumberOfVertices(), 1);

      DMatrixRMaj perfectCMPMatrix = new DMatrixRMaj(2, 1);
      perfectCMP.get(perfectCMPMatrix);

      PolygonWiggler.convertToInequalityConstraints(copConstraint, copAin, copBin, 0.01);
      PolygonWiggler.convertToInequalityConstraints(cmpConstraint, cmpAin, cmpBin, -0.05);
      CommonOps_DDRM.multAdd(-1.0, copAin, perfectCMPMatrix, copBin);
      CommonOps_DDRM.multAdd(-1.0, cmpAin, perfectCMPMatrix, cmpBin);

      EjmlUnitTests.assertEquals(copAin, solver.getCoPLocationConstraint().Aineq, 1e-7);
      EjmlUnitTests.assertEquals(copBin, solver.getCoPLocationConstraint().bineq, 1e-7);

      EjmlUnitTests.assertEquals(cmpAin, solver.getCMPLocationConstraint().Aineq, 1e-7);
      EjmlUnitTests.assertEquals(cmpBin, solver.getCMPLocationConstraint().bineq, 1e-7);

      // test right support
      contactStates.get(RobotSide.RIGHT).setFullyConstrained();
      contactStates.get(RobotSide.LEFT).getContactPoints().forEach(point -> point.setInContact(false));
      bipedSupportPolygons.updateUsingContactStates(contactStates);

      solver.resetCoPLocationConstraint();
      solver.addSupportPolygon(constraintHandler.updateCoPConstraint(bipedSupportPolygons.getSupportPolygonInWorld()));

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

      copAin = new DMatrixRMaj(copConstraint.getNumberOfVertices(), 2);
      copBin = new DMatrixRMaj(copConstraint.getNumberOfVertices(), 1);

      cmpAin = new DMatrixRMaj(copConstraint.getNumberOfVertices(), 2);
      cmpBin = new DMatrixRMaj(copConstraint.getNumberOfVertices(), 1);

      perfectCMPMatrix = new DMatrixRMaj(2, 1);
      perfectCMP.get(perfectCMPMatrix);

      PolygonWiggler.convertToInequalityConstraints(copConstraint, copAin, copBin, 0.01);
      PolygonWiggler.convertToInequalityConstraints(cmpConstraint, cmpAin, cmpBin, -0.05);
      CommonOps_DDRM.multAdd(-1.0, copAin, perfectCMPMatrix, copBin);
      CommonOps_DDRM.multAdd(-1.0, cmpAin, perfectCMPMatrix, cmpBin);

      EjmlUnitTests.assertEquals(copAin, solver.getCoPLocationConstraint().Aineq, 1e-7);
      EjmlUnitTests.assertEquals(copBin, solver.getCoPLocationConstraint().bineq, 1e-7);

      EjmlUnitTests.assertEquals(cmpAin, solver.getCMPLocationConstraint().Aineq, 1e-7);
      EjmlUnitTests.assertEquals(cmpBin, solver.getCMPLocationConstraint().bineq, 1e-7);
   }

   @Test
   public void testDoubleSupportWithBipedSupportPolygonsNoAngularMomentum()
   {
      YoRegistry registry = new YoRegistry("robert");

      SideDependentList<FootSpoof> contactableFeet = setupContactableFeet(footLength, 0.1, stanceWidth);
      SideDependentList<YoPlaneContactState> contactStates = new SideDependentList<>();
      BipedSupportPolygons bipedSupportPolygons = setupBipedSupportPolygons(contactableFeet, registry, contactStates);
      YoBoolean useControlPolygons = new YoBoolean("useControlPolygons", registry);
      ICPCoPConstraintHandler constraintHandler = new ICPCoPConstraintHandler(null, useControlPolygons, false, registry);
      ICPControllerQPSolver solver = new ICPControllerQPSolver(5, false, null);

      solver.resetCoPLocationConstraint();
      solver.addSupportPolygon(constraintHandler.updateCoPConstraint(bipedSupportPolygons.getSupportPolygonInWorld()));

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

      ConvexPolygonScaler scaler = new ConvexPolygonScaler();
      ConvexPolygon2D scaledCoPConstraint = new ConvexPolygon2D();

      DMatrixRMaj copAin = new DMatrixRMaj(copConstraint.getNumberOfVertices(), 2);
      DMatrixRMaj copBin = new DMatrixRMaj(copConstraint.getNumberOfVertices(), 1);

      DMatrixRMaj perfectCMPMatrix = new DMatrixRMaj(2, 1);
      perfectCMP.get(perfectCMPMatrix);

      scaler.scaleConvexPolygon(copConstraint, 0.01, scaledCoPConstraint);
      PolygonWiggler.convertToInequalityConstraints(scaledCoPConstraint, copAin, copBin, 0.0);
      CommonOps_DDRM.multAdd(-1.0, copAin, perfectCMPMatrix, copBin);

      EjmlUnitTests.assertEquals(copAin, solver.getCoPLocationConstraint().Aineq, 1e-7);
      EjmlUnitTests.assertEquals(copBin, solver.getCoPLocationConstraint().bineq, 1e-7);

      assertEquals(CommonOps_DDRM.elementSum(solver.getCMPLocationConstraint().Aineq), 0.0, 1e-7);
      assertEquals(CommonOps_DDRM.elementSum(solver.getCMPLocationConstraint().bineq), 0.0, 1e-7);
   }

   @Test
   public void testSingleSupportWithBipedSupportPolygonsNoAngularMomentum()
   {
      YoRegistry registry = new YoRegistry("robert");

      SideDependentList<FootSpoof> contactableFeet = setupContactableFeet(footLength, 0.1, stanceWidth);
      SideDependentList<YoPlaneContactState> contactStates = new SideDependentList<>();
      BipedSupportPolygons bipedSupportPolygons = setupBipedSupportPolygons(contactableFeet, registry, contactStates);
      YoBoolean useControlPolygons = new YoBoolean("useControlPolygons", registry);
      ICPCoPConstraintHandler constraintHandler = new ICPCoPConstraintHandler(null, useControlPolygons, false, registry);
      ICPControllerQPSolver solver = new ICPControllerQPSolver(5, false, null);

      contactStates.get(RobotSide.RIGHT).getContactPoints().forEach(point -> point.setInContact(false));
      bipedSupportPolygons.updateUsingContactStates(contactStates);

      // test left support
      solver.resetCoPLocationConstraint();
      solver.addSupportPolygon(constraintHandler.updateCoPConstraint(bipedSupportPolygons.getSupportPolygonInWorld()));

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

      DMatrixRMaj copAin = new DMatrixRMaj(copConstraint.getNumberOfVertices(), 2);
      DMatrixRMaj copBin = new DMatrixRMaj(copConstraint.getNumberOfVertices(), 1);

      DMatrixRMaj perfectCMPMatrix = new DMatrixRMaj(2, 1);
      perfectCMP.get(perfectCMPMatrix);

      PolygonWiggler.convertToInequalityConstraints(copConstraint, copAin, copBin, 0.01);
      CommonOps_DDRM.multAdd(-1.0, copAin, perfectCMPMatrix, copBin);

      EjmlUnitTests.assertEquals(copAin, solver.getCoPLocationConstraint().Aineq, 1e-7);
      EjmlUnitTests.assertEquals(copBin, solver.getCoPLocationConstraint().bineq, 1e-7);

      assertEquals(CommonOps_DDRM.elementSum(solver.getCMPLocationConstraint().Aineq), 0.0, 1e-7);
      assertEquals(CommonOps_DDRM.elementSum(solver.getCMPLocationConstraint().bineq), 0.0, 1e-7);

      // test right support
      contactStates.get(RobotSide.RIGHT).setFullyConstrained();
      contactStates.get(RobotSide.LEFT).getContactPoints().forEach(point -> point.setInContact(false));
      bipedSupportPolygons.updateUsingContactStates(contactStates);

      solver.resetCoPLocationConstraint();
      solver.addSupportPolygon(constraintHandler.updateCoPConstraint(bipedSupportPolygons.getSupportPolygonInWorld()));

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

      copAin = new DMatrixRMaj(copConstraint.getNumberOfVertices(), 2);
      copBin = new DMatrixRMaj(copConstraint.getNumberOfVertices(), 1);

      perfectCMPMatrix = new DMatrixRMaj(2, 1);
      perfectCMP.get(perfectCMPMatrix);

      PolygonWiggler.convertToInequalityConstraints(copConstraint, copAin, copBin, 0.01);
      CommonOps_DDRM.multAdd(-1.0, copAin, perfectCMPMatrix, copBin);

      assertTrue(MatrixFeatures_DDRM.isEquals(copAin, solver.getCoPLocationConstraint().Aineq, 1e-7));
      assertTrue(MatrixFeatures_DDRM.isEquals(copBin, solver.getCoPLocationConstraint().bineq, 1e-7));

      assertEquals(CommonOps_DDRM.elementSum(solver.getCMPLocationConstraint().Aineq), 0.0, 1e-7);
      assertEquals(CommonOps_DDRM.elementSum(solver.getCMPLocationConstraint().bineq), 0.0, 1e-7);
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

   private BipedSupportPolygons setupBipedSupportPolygons(SideDependentList<FootSpoof> contactableFeet,
                                                          YoRegistry registry,
                                                          SideDependentList<YoPlaneContactState> contactStatesToPack)
   {
      SideDependentList<ReferenceFrame> ankleZUpFrames = new SideDependentList<>();
      SideDependentList<ReferenceFrame> soleFrames = new SideDependentList<>();

      for (RobotSide robotSide : RobotSide.values)
      {
         FootSpoof contactableFoot = contactableFeet.get(robotSide);
         ReferenceFrame ankleFrame = contactableFoot.getFrameAfterParentJoint();
         ankleFrames.put(robotSide, ankleFrame);
         ankleZUpFrames.put(robotSide, new ZUpFrame(ankleFrame, robotSide.getCamelCaseNameForStartOfExpression() + "ZUp"));

         String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
         RigidBodyBasics foot = contactableFoot.getRigidBody();
         ReferenceFrame soleFrame = contactableFoot.getSoleFrame();
         soleFrames.put(robotSide, soleFrame);
         List<FramePoint2D> contactFramePoints = contactableFoot.getContactPoints2d();
         double coefficientOfFriction = contactableFoot.getCoefficientOfFriction();
         YoPlaneContactState yoPlaneContactState = new YoPlaneContactState(sidePrefix
               + "Foot", foot, soleFrame, contactFramePoints, coefficientOfFriction, registry);
         yoPlaneContactState.setFullyConstrained();
         contactStatesToPack.put(robotSide, yoPlaneContactState);
      }

      ReferenceFrame midFeetZUpFrame = new MidFrameZUpFrame("midFeetZupFrame",
                                                            worldFrame,
                                                            ankleZUpFrames.get(RobotSide.LEFT),
                                                            ankleZUpFrames.get(RobotSide.RIGHT));
      midFeetZUpFrame.update();

      BipedSupportPolygons bipedSupportPolygons = new BipedSupportPolygons(midFeetZUpFrame, ankleZUpFrames, soleFrames, registry, null);
      bipedSupportPolygons.updateUsingContactStates(contactStatesToPack);

      return bipedSupportPolygons;
   }

}
