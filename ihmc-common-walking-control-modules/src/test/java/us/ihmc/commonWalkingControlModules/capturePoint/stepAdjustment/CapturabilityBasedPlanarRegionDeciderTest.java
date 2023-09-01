package us.ihmc.commonWalkingControlModules.capturePoint.stepAdjustment;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import us.ihmc.commonWalkingControlModules.capturePoint.stepAdjustment.CapturabilityBasedPlanarRegionDecider;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.StepConstraintListConverter;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.StepConstraintRegion;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.concaveHull.GeometryPolygonTestTools;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.TranslationReferenceFrame;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

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
      CapturabilityBasedPlanarRegionDecider constraintCalculator = new CapturabilityBasedPlanarRegionDecider(new YoRegistry("Dummy"), null);
      constraintCalculator.setSwitchPlanarRegionConstraintsAutomatically(true);

      FramePose3D stepPose = new FramePose3D();
      stepPose.getPosition().set(stepLength, 0.5 * stanceWidth, 0.0);

      FrameConvexPolygon2D captureRegion = new FrameConvexPolygon2D();
      captureRegion.addVertex(stepLength + 0.05, 0.5 * stanceWidth + 0.05);
      captureRegion.addVertex(stepLength - 0.05, 0.5 * stanceWidth + 0.05);
      captureRegion.addVertex(stepLength - 0.05, 0.5 * stanceWidth - 0.05);
      captureRegion.addVertex(stepLength + 0.05, 0.5 * stanceWidth - 0.05);
      captureRegion.update();

      constraintCalculator.setConstraintRegions(constraintRegions);
      constraintCalculator.setCaptureRegion(captureRegion);

      constraintCalculator.updatePlanarRegionConstraintForStep(stepPose, null);

      assertTrue(groundPlane.epsilonEquals(constraintCalculator.getConstraintRegion(), 1e-8));

      constraintCalculator.reset();
      constraintCalculator.setCaptureRegion(captureRegion);

      constraintCalculator.updatePlanarRegionConstraintForStep(stepPose, null);
      assertNull(constraintCalculator.getConstraintRegion());
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
      CapturabilityBasedPlanarRegionDecider constraintCalculator = new CapturabilityBasedPlanarRegionDecider(new YoRegistry("Dummy"), null);
      constraintCalculator.setSwitchPlanarRegionConstraintsAutomatically(true);

      centerOfMassFrame.setPositionAndUpdate(new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.1, -0.05, comHeight));

      constraintCalculator.setConstraintRegions(constraintRegions);
      constraintCalculator.setCaptureRegion(captureRegion);
      constraintCalculator.updatePlanarRegionConstraintForStep(stepPose, null);

      assertTrue(smallRegionPlane.epsilonEquals(constraintCalculator.getConstraintRegion(), 1e-8));

      captureRegion.clear();
      captureRegion.addVertex(stepLength + 0.05 + 0.05, 0.5 * stanceWidth + 0.05 - 0.2);
      captureRegion.addVertex(stepLength + 0.05 - 0.05, 0.5 * stanceWidth + 0.05 - 0.2);
      captureRegion.addVertex(stepLength + 0.05 - 0.05, 0.5 * stanceWidth - 0.05 - 0.2);
      captureRegion.addVertex(stepLength + 0.05 + 0.05, 0.5 * stanceWidth - 0.05 - 0.2);
      captureRegion.update();

      constraintCalculator.setCaptureRegion(captureRegion);

      constraintCalculator.updatePlanarRegionConstraintForStep(stepPose, null);

      GeometryPolygonTestTools.assertConcavePolygon2DEquals(groundPlane.getConcaveHull(), constraintCalculator.getConstraintRegion().getConcaveHull(), 1e-8);
      assertTrue(groundPlane.epsilonEquals(constraintCalculator.getConstraintRegion(), 1e-8));
   }

   @Test
   public void testOverSteppingStones()
   {
      FramePose3D stepPose = new FramePose3D();
      stepPose.getPosition().set(-10.230, -1.001, 0.3);
      stepPose.getOrientation().set(0.0, 0.001, 1.0, 0.004);

      FrameConvexPolygon2D captureRegion = new FrameConvexPolygon2D();
      captureRegion.addVertex(-010.983, -0.894);
      captureRegion.addVertex(-10.920, -0.863);
      captureRegion.addVertex(-10.109, -0.707);
      captureRegion.addVertex(-9.732, -0.743);
      captureRegion.addVertex(-9.688, -0.752);
      captureRegion.addVertex(-8.958, -1.446);
      captureRegion.addVertex(-8.939, -1.468);
      captureRegion.addVertex(-9.163, -1.625);
      captureRegion.addVertex(-9.479, -1.748);
      captureRegion.addVertex(-9.749, -1.791);
      captureRegion.addVertex(-10.022, -1.771);
      captureRegion.addVertex(-10.284, -1.690);
      captureRegion.addVertex(-10.346, -1.671);
      captureRegion.addVertex(-10.567, -1.544);
      captureRegion.addVertex(-10.736, -1.379);
      captureRegion.addVertex(-10.776, -1.335);
//      captureRegion.addVertex(-10.464, -0.516);
      captureRegion.update();

      List<PlanarRegion> environmentRegions = createSteppingStonesPlanarRegionsList();
      List<StepConstraintRegion> constraintRegions = StepConstraintListConverter.convertPlanarRegionListToStepConstraintRegion(environmentRegions);
      StepConstraintRegion lastRegion = StepConstraintListConverter.convertPlanarRegionToStepConstraintRegion(environmentRegions.get(environmentRegions.size() - 1));

      CapturabilityBasedPlanarRegionDecider constraintCalculator = new CapturabilityBasedPlanarRegionDecider(new YoRegistry("Dummy"), null);
      constraintCalculator.setSwitchPlanarRegionConstraintsAutomatically(true);


      constraintCalculator.setConstraintRegions(constraintRegions);
      constraintCalculator.setCaptureRegion(captureRegion);
      constraintCalculator.updatePlanarRegionConstraintForStep(stepPose, null);


      assertTrue(lastRegion.epsilonEquals(constraintCalculator.getConstraintRegion(), 1e-8));

   }

   private List<PlanarRegion> createSteppingStonesPlanarRegionsList()
   {
      List<Point3D> locations = new ArrayList<>();
      locations.add(new Point3D(-7.75, -0.55, 0.3));
      locations.add(new Point3D(-8.25, -0.95, 0.3));
      locations.add(new Point3D(-8.75, -0.55, 0.3));
      locations.add(new Point3D(-9.25, -0.95, 0.3));
      locations.add(new Point3D(-9.75, -0.55, 0.3));
      locations.add(new Point3D(-10.25, -1.0, 0.3));
      locations.add(new Point3D(-10.25, -0.65, 0.3));

      List<PlanarRegion> planarRegions = new ArrayList<>();
      int idStart = 10;
      for (int i = 0; i < locations.size() - 2; i++)
      {
         PlanarRegion planarRegion = createSteppingStonePlanarRegion(locations.get(i));
         planarRegion.setRegionId(idStart + i);
         planarRegions.add(planarRegion);
      }

      PlanarRegion platform = createEndPlanarRegion(locations.get(locations.size() - 2));
      platform.setRegionId(idStart + locations.size() - 2);

      planarRegions.add(platform);

      return planarRegions;
   }

   private PlanarRegion createSteppingStonePlanarRegion(Point3D centered)
   {
      ConvexPolygon2D convexPolygon2D = new ConvexPolygon2D();
      List<Point2D> points = createSteppingStoneFace();
      for (Point2D point : points)
      {
         point.add(centered.getX(), centered.getY());
         convexPolygon2D.addVertex(point);
      }
      convexPolygon2D.update();

      TranslationReferenceFrame planarRegionFrame = new TranslationReferenceFrame("planarRegionFrame", ReferenceFrame.getWorldFrame());
      planarRegionFrame.updateTranslation(new Vector3D(0.0, 0.0, 0.3));

      PlanarRegion planarRegion = new PlanarRegion(planarRegionFrame.getTransformToWorldFrame(), convexPolygon2D);
      return planarRegion;
   }

   private PlanarRegion createEndPlanarRegion(Point3D centered)
   {
      ConvexPolygon2D convexPolygon2D = new ConvexPolygon2D();
      List<Point2D> points = createPlatformFace();
      for (Point2D point : points)
      {
         point.add(centered.getX(), centered.getY());
         convexPolygon2D.addVertex(point);
      }
      convexPolygon2D.update();

      TranslationReferenceFrame planarRegionFrame = new TranslationReferenceFrame("planarRegionFrame", ReferenceFrame.getWorldFrame());
      planarRegionFrame.updateTranslation(new Vector3D(0.0, 0.0, 0.3));

      PlanarRegion planarRegion = new PlanarRegion(planarRegionFrame.getTransformToWorldFrame(), convexPolygon2D);
      return planarRegion;
   }

   private List<Point2D> createSteppingStoneFace()
   {
      ArrayList<Point2D> points = new ArrayList<>();
      points.add(new Point2D(0.25, 0.25));
      points.add(new Point2D(-0.25, 0.25));
      points.add(new Point2D(-0.25, -0.25));
      points.add(new Point2D(0.25, -0.25));

      return points;
   }

   private List<Point2D> createPlatformFace()
   {
      ArrayList<Point2D> points = new ArrayList<>();
      points.add(new Point2D(0.25, 1.0));
      points.add(new Point2D(-0.25, 1.0));
      points.add(new Point2D(-0.25, -1.0));
      points.add(new Point2D(0.25, -1.0));

      return points;
   }

}
