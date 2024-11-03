package us.ihmc.commonWalkingControlModules.captureRegion;

import org.junit.jupiter.api.Test;
import us.ihmc.commonWalkingControlModules.capturePoint.stepAdjustment.StepAdjustmentParameters;
import us.ihmc.commonWalkingControlModules.capturePoint.stepAdjustment.StepAdjustmentReachabilityConstraint;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.geometry.FrameGeometry2dPlotter;
import us.ihmc.robotics.geometry.FrameGeometryTestFrame;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ZUpFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.parameters.DefaultParameterReader;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.awt.*;
import java.util.Random;

import static us.ihmc.robotics.Assert.assertTrue;

public class MultiStepWithHeuristicsTest
{
   private static final boolean VISUALIZE = true;

   @Test
   public void testHardwareBug20221207_091601_NadiaControllerFactory()
   {
      FramePose3D leftFootPose = new FramePose3D();
      leftFootPose.getPosition().set(0.359635, 0.023577, 0.042572);
      leftFootPose.getOrientation().set(-0.019835, 0.000640, 0.111174, 0.993603);
      PoseReferenceFrame leftSoleFrame = new PoseReferenceFrame("leftSoleFrame", ReferenceFrame.getWorldFrame());
      ZUpFrame leftSoleZUpFrame = new ZUpFrame(leftSoleFrame, "leftSoleZUpFrame");
      leftSoleFrame.setPoseAndUpdate(leftFootPose);
      leftSoleZUpFrame.update();
      FrameConvexPolygon2D solePolygon = new FrameConvexPolygon2D(leftSoleZUpFrame);
      solePolygon.addVertex(-0.057507737057184344, 0.01246372307278952);
      solePolygon.addVertex(0.087493, 0.0);
      solePolygon.addVertex(0.087504, -0.009977);
      solePolygon.addVertex(-0.057490, -0.027464);
      solePolygon.update();
      StepAdjustmentParameters.CrossOverReachabilityParameters crossOverReachabilityParameters = new StepAdjustmentParameters.CrossOverReachabilityParameters();

      SideDependentList<ReferenceFrame> soleZUpFrames = new SideDependentList<>(leftSoleZUpFrame, leftSoleZUpFrame);
      double footWidth = 0.095;
      double maxLength = 0.6;
      YoRegistry registry = new YoRegistry("test");
      OneStepCaptureRegionCalculator captureRegionCalculator = new OneStepCaptureRegionCalculator(footWidth,
                                                                                                  () -> 1.5 * maxLength,
                                                                                                  soleZUpFrames,
                                                                                                  false,
                                                                                                  "controller",
                                                                                                  registry,
                                                                                                  null);;
      CaptureRegionSafetyHeuristics heuristics = new CaptureRegionSafetyHeuristics(() -> maxLength, registry);
      StepAdjustmentReachabilityConstraint reachabilityConstraint = new StepAdjustmentReachabilityConstraint(soleZUpFrames,
                                                                                                             () -> maxLength,
                                                                                                             () -> maxLength,
                                                                                                             () -> 0.075,
                                                                                                             () -> maxLength,
                                                                                                             () -> 0.25,
                                                                                                             crossOverReachabilityParameters,
                                                                                                             "controller",
                                                                                                             false,
                                                                                                             registry,
                                                                                                             null);
      MultiStepCaptureRegionCalculator mutliStepCalculator = new MultiStepCaptureRegionCalculator(reachabilityConstraint, () -> false, registry);
      ((YoDouble) registry.findVariable("distanceIntoCaptureRegionForInside")).set(0.05);
      ((YoDouble) registry.findVariable("distanceIntoCaptureRegionForEverywhere")).set(0.02);
      ((YoDouble) registry.findVariable("extraDistanceToStepFromStanceFoot")).set(0.05);

      new DefaultParameterReader().readParametersInRegistry(registry);
      double timeRemaining = 0.52;
      double omega = 3.2;
      int stepsInQueue = 2;
      double swingDuration = 0.993899;
      double transferDuration = 0.2;
      FramePoint2D capturePoint = new FramePoint2D(ReferenceFrame.getWorldFrame(), 0.362380, 0.000811);
      RobotSide swingSide = RobotSide.RIGHT;

      FramePoint2D unadjustedFootstepPositionInWorld = new FramePoint2D(ReferenceFrame.getWorldFrame(), 0.461538, -0.22989);
      FramePoint2D unadjustedFootstepPosition = new FramePoint2D(unadjustedFootstepPositionInWorld);
      unadjustedFootstepPosition.changeFrameAndProjectToXYPlane(leftSoleZUpFrame);

      double maxCapturePointPerturbation = 0.01;
      FramePoint2D perterbedCapturePoint = new FramePoint2D(capturePoint);
      FramePose3D perturbedFootPose = new FramePose3D(leftFootPose);
      FrameConvexPolygon2D perturbedSolePolygon = new FrameConvexPolygon2D(solePolygon);

      Random random = new Random(1738L);


      for (int iter = 0; iter < 1000; iter++)
      {
         leftSoleFrame.setPoseAndUpdate(leftFootPose);
         leftSoleZUpFrame.update();

         FrameConvexPolygon2D supportInWorld = new FrameConvexPolygon2D(perturbedSolePolygon);
         supportInWorld.changeFrameAndProjectToXYPlane(ReferenceFrame.getWorldFrame());


         captureRegionCalculator.calculateCaptureRegion(swingSide, timeRemaining, perterbedCapturePoint, omega, perturbedSolePolygon);
         heuristics.computeCaptureRegionWithSafetyHeuristics(swingSide.getOppositeSide(), perterbedCapturePoint, supportInWorld.getCentroid(), captureRegionCalculator.getCaptureRegion());
         mutliStepCalculator.compute(swingSide.getOppositeSide(),
                                     heuristics.getCaptureRegionWithSafetyMargin(),
                                     swingDuration + transferDuration,
                                     omega,
                                     stepsInQueue);
         double heuristicDistance = heuristics.getCaptureRegionWithSafetyMargin().signedDistance(unadjustedFootstepPositionInWorld);

         boolean allGood = captureRegionCalculator.getCaptureRegion().signedDistance(unadjustedFootstepPosition) < -0.05 &&
                           heuristicDistance < -0.05 &&
                           mutliStepCalculator.getCaptureRegion().signedDistance(unadjustedFootstepPositionInWorld) < -0.05;
         if (!allGood && VISUALIZE)
         {
            FrameConvexPolygon2D captureRegionInWorld = new FrameConvexPolygon2D(captureRegionCalculator.getCaptureRegion());
            captureRegionInWorld.changeFrame(ReferenceFrame.getWorldFrame());

            LogTools.info("Heuristic distance = " + heuristicDistance);
            FrameGeometryTestFrame testFrame = new FrameGeometryTestFrame(-5, 5, -5, 5);
            FrameGeometry2dPlotter plotter = testFrame.getFrameGeometry2dPlotter();
            plotter.setDrawPointsLarge();
            plotter.addPolygon(supportInWorld, Color.black);
            plotter.addFramePoint2d(perterbedCapturePoint, Color.blue);
            plotter.addFramePoint2d(unadjustedFootstepPositionInWorld, Color.yellow);
            plotter.addPolygon(captureRegionInWorld, Color.yellow);
            plotter.addPolygon(heuristics.getCaptureRegionWithSafetyMargin(), Color.red);
//            plotter.addPolygon(mutliStepCalculator.getCaptureRegion(), Color.green);

            for (int i = 0; i < captureRegionInWorld.getNumberOfVertices(); i++)
               plotter.addFramePoint2d(captureRegionInWorld.getVertex(i), Color.yellow);
//            for (int i = 0; i < mutliStepCalculator.getCaptureRegion().getNumberOfVertices(); i++)
//               plotter.addFramePoint2d(mutliStepCalculator.getCaptureRegion().getVertex(i), Color.green);
            for (int i = 0; i < heuristics.getCaptureRegionWithSafetyMargin().getNumberOfVertices(); i++)
               plotter.addFramePoint2d(heuristics.getCaptureRegionWithSafetyMargin().getVertex(i), Color.red);

            ThreadTools.sleepForever();
         }

         assertTrue("capture region doesn't include the foot pose, which it should",
                    captureRegionCalculator.getCaptureRegion().signedDistance(unadjustedFootstepPosition) < -0.05);

         assertTrue("heuristic capture region doesn't include the foot pose, which it should. Distance is " + heuristicDistance,
                     heuristicDistance < -0.05);

         assertTrue("multi-step capture region doesn't include the foot pose, which it should",
                    mutliStepCalculator.getCaptureRegion().signedDistance(unadjustedFootstepPositionInWorld) < -0.05);



         perterbedCapturePoint.set(capturePoint);
         perterbedCapturePoint.add(RandomNumbers.nextDouble(random, maxCapturePointPerturbation), RandomNumbers.nextDouble(random, maxCapturePointPerturbation));

         perturbedFootPose.set(leftFootPose);
         perturbedSolePolygon.set(solePolygon);

      }

   }
}
