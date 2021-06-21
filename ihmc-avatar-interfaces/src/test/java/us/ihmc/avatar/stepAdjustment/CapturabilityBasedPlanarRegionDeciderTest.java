package us.ihmc.avatar.stepAdjustment;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import us.ihmc.commonWalkingControlModules.captureRegion.CapturabilityBasedPlanarRegionDecider;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.StepConstraintRegion;
import us.ihmc.robotics.geometry.concaveHull.GeometryPolygonTestTools;
import us.ihmc.robotics.geometry.concavePolygon2D.ConcavePolygon2D;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertTrue;

public class CapturabilityBasedPlanarRegionDeciderTest
{
   @AfterEach
   public void tearDown()
   {
      ReferenceFrameTools.clearWorldFrameTree();
   }

   @Test
   public void testFlatGround()
   {
      double stanceWidth = 0.2;
      double stepLength = 0.3;

      ConvexPolygon2D ground = new ConvexPolygon2D();
      ground.addVertex(10.0, 10.0);
      ground.addVertex(10.0, -10.0);
      ground.addVertex(-10.0, -10.0);
      ground.addVertex(-10.0, 10.0);
      ground.update();
      StepConstraintRegion groundPlane = new StepConstraintRegion(new RigidBodyTransform(), ground);

      List<StepConstraintRegion> constraintRegions = new ArrayList<>();
      constraintRegions.add(groundPlane);

      PoseReferenceFrame centerOfMassFrame = new PoseReferenceFrame("centerOfMassFrame", ReferenceFrame.getWorldFrame());
      centerOfMassFrame.translateAndUpdate(0.0, 0.0, 1.0);
      CapturabilityBasedPlanarRegionDecider<StepConstraintRegion> constraintCalculator = new CapturabilityBasedPlanarRegionDecider<>(centerOfMassFrame,
                                                                                                             9.81,
                                                                                                             StepConstraintRegion::new,
                                                                                                             new YoRegistry("Dummy"),
                                                                                                             null);
      constraintCalculator.setSwitchPlanarRegionConstraintsAutomatically(true);

      FramePose3D stepPose = new FramePose3D();
      stepPose.getPosition().set(stepLength, 0.5 * stanceWidth, 0.0);

      FrameConvexPolygon2D captureRegion = new FrameConvexPolygon2D();
      captureRegion.addVertex(stepLength + 0.05, 0.5 * stanceWidth + 0.05);
      captureRegion.addVertex(stepLength - 0.05, 0.5 * stanceWidth + 0.05);
      captureRegion.addVertex(stepLength - 0.05, 0.5 * stanceWidth - 0.05);
      captureRegion.addVertex(stepLength + 0.05, 0.5 * stanceWidth - 0.05);
      captureRegion.update();

      constraintCalculator.setOmega0(3.0);
      constraintCalculator.setConstraintRegions(constraintRegions);
      constraintCalculator.setReachableCaptureRegion(captureRegion);

      constraintCalculator.updatePlanarRegionConstraintForStep(stepPose.getPosition());

      assertTrue(groundPlane.epsilonEquals(constraintCalculator.getConstraintRegion(), 1e-8));
   }

   @Test
   public void testFlatGroundWithOtherRegion()
   {
      double stanceWidth = 0.2;
      double stepLength = 0.3;
      double stepWidth = 0.15;

      double comHeight = 1.0;
      double gravity = 9.81;
      double omega = Math.sqrt(gravity / comHeight);

      FramePose3D stepPose = new FramePose3D();
      stepPose.getPosition().set(stepLength, stepWidth, 0.0);

      ConvexPolygon2D ground = new ConvexPolygon2D();
      ground.addVertex(10.0, 10.0);
      ground.addVertex(10.0, -10.0);
      ground.addVertex(-10.0, -10.0);
      ground.addVertex(-10.0, 10.0);
      ground.update();
      StepConstraintRegion groundPlane = new StepConstraintRegion(new RigidBodyTransform(), ground);

      ConvexPolygon2D smallRegion = new ConvexPolygon2D();
      smallRegion.addVertex(0.2, 0.2);
      smallRegion.addVertex(0.2, -0.2);
      smallRegion.addVertex(-0.2, -0.2);
      smallRegion.addVertex(-0.2, 0.2);
      smallRegion.update();
      RigidBodyTransform smallRegionTransform = new RigidBodyTransform();
      smallRegionTransform.getTranslation().set(stepPose.getPosition());
      smallRegionTransform.getTranslation().setZ(0.1);
      StepConstraintRegion smallRegionPlane = new StepConstraintRegion(smallRegionTransform, smallRegion);

      List<StepConstraintRegion> constraintRegions = new ArrayList<>();
      constraintRegions.add(groundPlane);
      constraintRegions.add(smallRegionPlane);

      FrameConvexPolygon2D captureRegion = new FrameConvexPolygon2D();
      captureRegion.addVertex(stepLength + 0.05, 0.5 * stanceWidth + 0.05);
      captureRegion.addVertex(stepLength - 0.05, 0.5 * stanceWidth + 0.05);
      captureRegion.addVertex(stepLength - 0.05, 0.5 * stanceWidth - 0.05);
      captureRegion.addVertex(stepLength + 0.05, 0.5 * stanceWidth - 0.05);
      captureRegion.update();

      PoseReferenceFrame centerOfMassFrame = new PoseReferenceFrame("centerOfMassFrame", ReferenceFrame.getWorldFrame());
      CapturabilityBasedPlanarRegionDecider<StepConstraintRegion> constraintCalculator = new CapturabilityBasedPlanarRegionDecider<>(centerOfMassFrame,
                                                                                                             9.81,
                                                                                                             StepConstraintRegion::new,
                                                                                                             new YoRegistry("Dummy"),
                                                                                                             null);
      constraintCalculator.setSwitchPlanarRegionConstraintsAutomatically(true);

      centerOfMassFrame.setPositionAndUpdate(new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.1, -0.05, comHeight));

      constraintCalculator.setOmega0(omega);
      constraintCalculator.setConstraintRegions(constraintRegions);
      constraintCalculator.setReachableCaptureRegion(captureRegion);
      constraintCalculator.updatePlanarRegionConstraintForStep(stepPose.getPosition());

      assertTrue(smallRegionPlane.epsilonEquals(constraintCalculator.getConstraintRegion(), 1e-8));

      captureRegion.clear();
      captureRegion.addVertex(stepLength + 0.05 + 0.05, 0.5 * stanceWidth + 0.05 - 0.2);
      captureRegion.addVertex(stepLength + 0.05 - 0.05, 0.5 * stanceWidth + 0.05 - 0.2);
      captureRegion.addVertex(stepLength + 0.05 - 0.05, 0.5 * stanceWidth - 0.05 - 0.2);
      captureRegion.addVertex(stepLength + 0.05 + 0.05, 0.5 * stanceWidth - 0.05 - 0.2);
      captureRegion.update();

      constraintCalculator.setReachableCaptureRegion(captureRegion);

      constraintCalculator.updatePlanarRegionConstraintForStep(stepPose.getPosition());

      ConcavePolygon2D concaveHullExpected = new ConcavePolygon2D(Vertex2DSupplier.asVertex2DSupplier(groundPlane.getConcaveHull()));
      ConcavePolygon2D concaveHull = new ConcavePolygon2D(Vertex2DSupplier.asVertex2DSupplier(constraintCalculator.getConstraintRegion().getConcaveHull()));
      GeometryPolygonTestTools.assertConcavePolygon2DEquals(concaveHullExpected, concaveHull, 1e-8);
      assertTrue(groundPlane.epsilonEquals(constraintCalculator.getConstraintRegion(), 1e-8));
   }
}
