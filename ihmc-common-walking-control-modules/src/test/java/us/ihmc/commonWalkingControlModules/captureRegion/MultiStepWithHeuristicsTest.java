package us.ihmc.commonWalkingControlModules.captureRegion;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ZUpFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.parameters.DefaultParameterReader;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.Random;

import static us.ihmc.robotics.Assert.assertTrue;

public class MultiStepWithHeuristicsTest
{
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
      FrameConvexPolygon2D solePolygon = new FrameConvexPolygon2D(leftSoleFrame);
      solePolygon.addVertex(-0.057508, 0.012464);
      solePolygon.addVertex(0.087493, 0.0);
      solePolygon.addVertex(0.087504, -0.009977);
      solePolygon.addVertex(-0.057490, -0.027464);
      solePolygon.update();

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
//      MultiStepCaptureRegionCalculator mutliStepCalculator = new MultiStepCaptureRegionCalculator();

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

      FramePoint2D modifiedCapturePoint = new FramePoint2D(capturePoint);

      Random random = new Random(1738L);
      FrameConvexPolygon2D supportInWorld = new FrameConvexPolygon2D(solePolygon);
      supportInWorld.changeFrameAndProjectToXYPlane(ReferenceFrame.getWorldFrame());

      for (int i = 0; i < 1000; i++)
      {
         captureRegionCalculator.calculateCaptureRegion(swingSide, timeRemaining, capturePoint, omega, solePolygon);
         heuristics.computeCaptureRegionWithSafetyHeuristics(swingSide.getOppositeSide(), capturePoint, supportInWorld.getCentroid(), captureRegionCalculator.getCaptureRegion());
//         mutliStepCalculator.compute(swingSide.getOppositeSide(),
//                                     heuristics.getCaptureRegionWithSafetyMargin(),
//                                     swingDuration + transferDuration,
//                                     omega,
//                                     stepsInQueue);

         assertTrue("capture region doesn't include the foot pose, which it should",
                    captureRegionCalculator.getCaptureRegion().signedDistance(unadjustedFootstepPosition) < -0.05);

         assertTrue("capture region doesn't include the foot pose, which it should",
                    heuristics.getCaptureRegionWithSafetyMargin().signedDistance(unadjustedFootstepPositionInWorld) < -0.05);

         modifiedCapturePoint.set(capturePoint);
         modifiedCapturePoint.add(RandomNumbers.nextDouble(random, 0.005), RandomNumbers.nextDouble(random, 0.005));
      }

   }
}
