package us.ihmc.avatar.networkProcessor.stepConstraintToolboxModule;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.stream.Collectors;

import static org.junit.jupiter.api.Assertions.assertTrue;

public class StepConstraintCalculatorTest
{
   private final PoseReferenceFrame leftAnkleZUpFrame = new PoseReferenceFrame("leftAnkleZUp", ReferenceFrame.getWorldFrame());
   private final PoseReferenceFrame rightAnkleZUpFrame = new PoseReferenceFrame("rightAnkleZUp", ReferenceFrame.getWorldFrame());
   private final SideDependentList<ReferenceFrame> ankleZUpFrames = new SideDependentList<ReferenceFrame>(leftAnkleZUpFrame, rightAnkleZUpFrame);

   @AfterEach
   public void tearDown()
   {
      ReferenceFrameTools.clearWorldFrameTree();
   }

   @Test
   public void testFlatGround()
   {
      SideDependentList<ConvexPolygon2D> footPolygons = PlannerTools.createDefaultFootPolygons();
      double stanceWidth = 0.2;
      double stepLength = 0.3;

      ConvexPolygon2D ground = new ConvexPolygon2D();
      ground.addVertex(10.0, 10.0);
      ground.addVertex(10.0, -10.0);
      ground.addVertex(-10.0, -10.0);
      ground.addVertex(-10.0, 10.0);
      ground.update();
      PlanarRegion groundPlane = new PlanarRegion(new RigidBodyTransform(), ground);

      PlanarRegionsList planarRegionsList = new PlanarRegionsList();
      planarRegionsList.addPlanarRegion(groundPlane);

      double maxStepLength = 0.75;
      double footWidth = 0.15;

      YoDouble time = new YoDouble("time", null);

      PoseReferenceFrame centerOfMassFrame = new PoseReferenceFrame("centerOfMassFrame", ReferenceFrame.getWorldFrame());
      StepConstraintCalculator constraintCalculator = new StepConstraintCalculator(ankleZUpFrames, centerOfMassFrame, footWidth, maxStepLength, time, 9.81);

      SimpleStep step = new SimpleStep();
      step.setSwingDuration(1.0);
      step.setStartTime(0.0);
      step.setSwingSide(RobotSide.LEFT);
      step.setStepPosition(new Point3D(stepLength, 0.5 * stanceWidth, 0.0));

      rightAnkleZUpFrame.setPositionAndUpdate(new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.0, -0.5 * stanceWidth, 0.0));

      time.set(0.50);

      Point3D capturePoint = new Point3D(0.15, -0.3 * stanceWidth, 0.0);

      constraintCalculator.setOmega(3.0);
      constraintCalculator.setCurrentStep(step);
      constraintCalculator.setPlanarRegions(planarRegionsList);
      constraintCalculator.setCapturePoint(capturePoint);
      for (RobotSide robotSide : RobotSide.values)
         constraintCalculator.setFootSupportPolygon(robotSide, footPolygons.get(robotSide).getVertexBufferView().stream().map(Point3D::new).collect(Collectors.toList()));

      constraintCalculator.update();

      assertTrue(groundPlane.epsilonEquals(constraintCalculator.getConstraintRegion(), 1e-8));
   }

   @Test
   public void testFlatGroundWithOtherRegion()
   {
      SideDependentList<ConvexPolygon2D> footPolygons = PlannerTools.createDefaultFootPolygons();
      double stanceWidth = 0.2;
      double stepLength = 0.3;
      double stepWidth = 0.15;

      double comHeight = 1.0;
      double gravity = 9.81;
      double omega = Math.sqrt(gravity / comHeight);

      Point3D stepPosition = new Point3D(stepLength, stepWidth, 0.0);

      ConvexPolygon2D ground = new ConvexPolygon2D();
      ground.addVertex(10.0, 10.0);
      ground.addVertex(10.0, -10.0);
      ground.addVertex(-10.0, -10.0);
      ground.addVertex(-10.0, 10.0);
      ground.update();
      PlanarRegion groundPlane = new PlanarRegion(new RigidBodyTransform(), ground);

      ConvexPolygon2D smallRegion = new ConvexPolygon2D();
      smallRegion.addVertex(0.2, 0.2);
      smallRegion.addVertex(0.2, -0.2);
      smallRegion.addVertex(-0.2, -0.2);
      smallRegion.addVertex(-0.2, 0.2);
      smallRegion.update();
      RigidBodyTransform smallRegionTransform = new RigidBodyTransform();
      smallRegionTransform.getTranslation().set(stepPosition);
      smallRegionTransform.getTranslation().setZ(0.1);
      PlanarRegion smallRegionPlane = new PlanarRegion(smallRegionTransform, smallRegion);

      PlanarRegionsList planarRegionsList = new PlanarRegionsList();
      planarRegionsList.addPlanarRegion(groundPlane);
      planarRegionsList.addPlanarRegion(smallRegionPlane);

      double maxStepLength = 0.75;
      double footWidth = 0.15;

      YoDouble time = new YoDouble("time", null);

      PoseReferenceFrame centerOfMassFrame = new PoseReferenceFrame("centerOfMassFrame", ReferenceFrame.getWorldFrame());
      StepConstraintCalculator constraintCalculator = new StepConstraintCalculator(ankleZUpFrames, centerOfMassFrame, footWidth, maxStepLength, time, gravity);

      SimpleStep step = new SimpleStep();
      step.setSwingDuration(1.0);
      step.setStartTime(0.0);
      step.setSwingSide(RobotSide.LEFT);
      step.setStepPosition(stepPosition);

      rightAnkleZUpFrame.setPositionAndUpdate(new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.0, -0.5 * stanceWidth, 0.0));

      time.set(0.50);

      centerOfMassFrame.setPositionAndUpdate(new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.1, -0.05, comHeight));
      Point3D capturePoint = new Point3D(0.15, -0.03, 0.0);

      constraintCalculator.setOmega(omega);
      constraintCalculator.setCurrentStep(step);
      constraintCalculator.setPlanarRegions(planarRegionsList);
      constraintCalculator.setCapturePoint(capturePoint);
      for (RobotSide robotSide : RobotSide.values)
         constraintCalculator.setFootSupportPolygon(robotSide, footPolygons.get(robotSide).getVertexBufferView().stream().map(Point3D::new).collect(Collectors.toList()));

      constraintCalculator.update();

      assertTrue(smallRegionPlane.epsilonEquals(constraintCalculator.getConstraintRegion(), 1e-8));

      Point3D disturbedCapturePoint = new Point3D(0.2, -0.04, 0.0);

      constraintCalculator.setCapturePoint(disturbedCapturePoint);

      constraintCalculator.update();
      assertTrue(groundPlane.epsilonEquals(constraintCalculator.getConstraintRegion(), 1e-8));
   }
}
