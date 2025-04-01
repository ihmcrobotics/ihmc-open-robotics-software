package us.ihmc.commonWalkingControlModules.desiredFootStep;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameTestTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.scs2.SimulationConstructionSet2;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicPoint2DDefinition;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint2D;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;

public class EllipticalStepPositionLimiterTest
{
   @Test
   public void testMinWidths()
   {
      EllipticalStepPositionLimiter stepPositionLimiter = new EllipticalStepPositionLimiter();

      FramePoint3D stepPositionUnlimited = new FramePoint3D();
      FramePoint3D stepPositionLimited = new FramePoint3D();
      FramePose3D stanceFootPose = new FramePose3D();
      PoseReferenceFrame stanceFootFrame = new PoseReferenceFrame("StanceFootFrame", ReferenceFrame.getWorldFrame());

      double stanceWidth = 0.25;
      double maxLength = 1.0;
      double maxWidth = 0.75;

      Random random = new Random(1738L);
      for (int i = 0; i < 10000; i++)
      {
         // wrap this in a loop so we test both side
         for (RobotSide swingSide : RobotSide.values)
         {
            // test a position that shouldn't be limited. The desired step width here is 0.2, and the min is 0.1, so there should be no adjustment
            double minWidth = 0.1;
            stanceFootPose.getPosition().set(0.0, swingSide.negateIfLeftSide(0.1), 0.0);
            // apply a random foot yaw to make sure that doesn't mess things up
            stanceFootPose.getOrientation().setToYawOrientation(RandomNumbers.nextDouble(random, Math.PI));
            stanceFootFrame.setPoseAndUpdate(stanceFootPose);

            stepPositionUnlimited.set(0.0, swingSide.negateIfRightSide(0.1), 0.0);

            stepPositionLimiter.enforceFootPositionConstraint(stepPositionUnlimited,
                                                              stepPositionLimited,
                                                              ReferenceFrame.getWorldFrame(),
                                                              stanceFootFrame,
                                                              stanceWidth,
                                                              maxLength,
                                                              minWidth,
                                                              maxWidth,
                                                              0.0,
                                                              swingSide);

            EuclidCoreTestTools.assertEquals(stepPositionUnlimited, stepPositionLimited, 1e-5);

            // increase the minimum width so that it forces the limited position out by 5 cm so that the robot is the minimum width from the stance foot
            minWidth = 0.25;
            stepPositionLimiter.enforceFootPositionConstraint(stepPositionUnlimited,
                                                              stepPositionLimited,
                                                              ReferenceFrame.getWorldFrame(),
                                                              stanceFootFrame,
                                                              stanceWidth,
                                                              maxLength,
                                                              minWidth,
                                                              maxWidth,
                                                              0.0,
                                                              swingSide);

            FramePoint3D stepPositionExpected = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.0, swingSide.negateIfRightSide(0.15), 0.0);
            EuclidCoreTestTools.assertEquals(stepPositionExpected, stepPositionLimited, 1e-5);

            // Move the stance foot to the left, so that the width is enforced by the constraint frame
            stanceFootPose.getPosition().set(0.0, swingSide.negateIfLeftSide(0.5), 0.0);
            stanceFootPose.getOrientation().setToYawOrientation(RandomNumbers.nextDouble(random, Math.PI));
            stanceFootFrame.setPoseAndUpdate(stanceFootPose);

            stepPositionLimiter.enforceFootPositionConstraint(stepPositionUnlimited,
                                                              stepPositionLimited,
                                                              ReferenceFrame.getWorldFrame(),
                                                              stanceFootFrame,
                                                              stanceWidth,
                                                              maxLength,
                                                              minWidth,
                                                              maxWidth,
                                                              0.0,
                                                              swingSide);

            stepPositionExpected.set(0.0, 0.5 * swingSide.negateIfRightSide(minWidth), 0.0);
            EuclidCoreTestTools.assertEquals(stepPositionExpected, stepPositionLimited, 1e-5);

            // decrease the desired position, such that the actual gets cropped
            stepPositionUnlimited.set(0.0, swingSide.negateIfRightSide(0.0), 0.0);
            stepPositionLimiter.enforceFootPositionConstraint(stepPositionUnlimited,
                                                              stepPositionLimited,
                                                              ReferenceFrame.getWorldFrame(),
                                                              stanceFootFrame,
                                                              stanceWidth,
                                                              maxLength,
                                                              0.15,
                                                              maxWidth,
                                                              0.0,
                                                              swingSide);

            // This is 0.075, becuase we know that the minimum width about the pelvis is HALF the min width vvalue
            stepPositionExpected = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.0, swingSide.negateIfRightSide(0.075), 0.0);
            EuclidCoreTestTools.assertEquals(stepPositionExpected, stepPositionLimited, 1e-5);
         }
      }
   }

   @Test
   public void testEnforceEllipticConstraint()
   {
      Random random = new Random(1738L);

      for (int i = 0; i < 100; i++)
      {
         ReferenceFrame constraintFrame = EuclidFrameRandomTools.nextReferenceFrame("constraintFrame", random);
         ReferenceFrame originalFrame = EuclidFrameRandomTools.nextReferenceFrame("originalFrame", random);

         double nominalWidth = RandomNumbers.nextDouble(random, 0.1, 0.4);
         double maxWidth = RandomNumbers.nextDouble(random, nominalWidth, 0.8);
         double maxForward = RandomNumbers.nextDouble(random, 0.0, 1.0);

         RobotSide stepSide = RandomNumbers.nextEnum(random, RobotSide.class);
         PoseReferenceFrame ellipseFrame = new PoseReferenceFrame("ellipseFrame", constraintFrame);
         ellipseFrame.setPositionAndUpdate(new FramePoint3D(constraintFrame, 0.0, stepSide.negateIfRightSide(0.5 * nominalWidth), 0.0));

         for (int pointIdx = 0; pointIdx < 5000; pointIdx++)
         {
            FramePoint3D pointToConstrain = EuclidFrameRandomTools.nextFramePoint3D(random, constraintFrame);
            pointToConstrain.changeFrame(originalFrame);
            FramePoint3D originalPoint = new FramePoint3D(pointToConstrain);

            boolean wasModified = EllipticalStepPositionLimiter.enforceOuterEllipticalBoundInConstraintFrame(pointToConstrain,
                                                                                                             constraintFrame,
                                                                                                             nominalWidth,
                                                                                                             maxForward,
                                                                                                             maxWidth,
                                                                                                             stepSide);

            // make sure the frames never changed
            assertEquals(originalFrame, pointToConstrain.getReferenceFrame());

            // check that points that were outside were changed, and points that were inside did not.
            boolean isOriginalPointInside = isPointInsideEllipse(originalPoint, ellipseFrame, maxForward, maxWidth - nominalWidth, 1e-16);
            boolean isNewPointInside = isPointInsideEllipse(pointToConstrain, ellipseFrame, maxForward, maxWidth - nominalWidth, 1e-6);
            if (isOriginalPointInside)
            {
               pointToConstrain.changeFrame(constraintFrame);
               originalPoint.changeFrame(constraintFrame);
               if (wasModified)
                  LogTools.info("crap");

               // point should not have been modified
               assertFalse(wasModified);
               // original point is inside, so the after point should be the exact same
               assertTrue(isNewPointInside);
               EuclidFrameTestTools.assertEquals(originalPoint, pointToConstrain, 1e-12);
            }
            else
            {
               pointToConstrain.changeFrame(ellipseFrame);
               originalPoint.changeFrame(ellipseFrame);
               if (!isNewPointInside)
                  LogTools.info("crap");

               double ellipseNumber = getEllipseNumber(pointToConstrain, ellipseFrame, maxForward, maxWidth - nominalWidth);
               // poitn should have been modified
               assertTrue(wasModified);
               // point should be on edge
               assertEquals(ellipseNumber, 1.0, 1e-9);
               assertTrue(isNewPointInside);
               // we should never project to opposite sides of the ellipse, as that wouldn't be the closest point
               assertEquals(Math.signum(originalPoint.getX()), Math.signum(pointToConstrain.getX()));
               assertEquals(Math.signum(originalPoint.getY()), Math.signum(pointToConstrain.getY()));
               // make sure the points arent' equal
               assertTrue(originalPoint.distance(pointToConstrain) > 1e-6);
               assertTrue(originalPoint.distanceXY(pointToConstrain) > 1e-6);
            }
         }
      }
   }

   private boolean isPointInsideEllipse(FramePoint3DReadOnly pointToCheck, ReferenceFrame ellipseFrame, double rx, double ry, double epsilon)
   {
      return getEllipseNumber(pointToCheck, ellipseFrame, rx, ry) <= 1.0 + epsilon;
   }

   private static double getEllipseNumber(FramePoint3DReadOnly pointToCheck, ReferenceFrame ellipseFrame, double rx, double ry)
   {
      FramePoint3D pointCopy = new FramePoint3D(pointToCheck);
      pointCopy.changeFrame(ellipseFrame);

      return MathTools.square(pointCopy.getX() / rx) + MathTools.square(pointCopy.getY() / ry);
   }

   public static void main(String[] args)
   {
      SimulationConstructionSet2 scs = new SimulationConstructionSet2();

      EllipticalStepPositionLimiter stepPositionLimiter = new EllipticalStepPositionLimiter();

      PoseReferenceFrame stanceFootFrame = new PoseReferenceFrame("StanceFootFrame", ReferenceFrame.getWorldFrame());

      double maxLength = 0.6;
      double maxWidth = 0.5;
      double minWidth = -0.2;
      double nominalWidth = 0.2;
      double distanceFromStance = 0.15;

      stanceFootFrame.setPositionAndUpdate(new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.0, -0.5 * nominalWidth, 0.0));

      YoRegistry registry = new YoRegistry(EllipticalStepPositionLimiter.class.getSimpleName());
      int index = 0;
      for (double x = -maxLength - 0.2; x <= maxLength + 0.2; x += 0.025)
      {
         for (double y = minWidth - 0.1; y <= maxWidth + 0.1; y += 0.025)
         {
            FramePoint3D stepPosition = new FramePoint3D();
            FramePoint3D constrainedStepPosition = new FramePoint3D();
            stepPosition.set(x, y, 0.0);

            stepPositionLimiter.enforceFootPositionConstraint(stepPosition,
                                                              constrainedStepPosition,
                                                              ReferenceFrame.getWorldFrame(),
                                                              stanceFootFrame,
                                                              nominalWidth,
                                                              maxLength,
                                                              minWidth,
                                                              maxWidth,
                                                              distanceFromStance,
                                                              RobotSide.LEFT);

            YoFramePoint2D unconstrainedPosition = new YoFramePoint2D("unconstrainedPosition" + index, ReferenceFrame.getWorldFrame(), registry);
            YoFramePoint2D constrainedPosition = new YoFramePoint2D("constrainedPosition" + index, ReferenceFrame.getWorldFrame(), registry);
            index++;

            unconstrainedPosition.set(stepPosition);
            constrainedPosition.set(constrainedStepPosition);

            YoGraphicPoint2DDefinition originGraphic = YoGraphicDefinitionFactory.newYoGraphicPoint2D(unconstrainedPosition.getNamePrefix(),
                                                                                                      unconstrainedPosition,
                                                                                                      0.0075,
                                                                                                      ColorDefinitions.Blue(),
                                                                                                      YoGraphicDefinitionFactory.DefaultPoint2DGraphic.CIRCLE);
            YoGraphicPoint2DDefinition goalGraphic = YoGraphicDefinitionFactory.newYoGraphicPoint2D(constrainedPosition.getNamePrefix(),
                                                                                                    constrainedPosition,
                                                                                                    0.006,
                                                                                                    ColorDefinitions.Red(),
                                                                                                    YoGraphicDefinitionFactory.DefaultPoint2DGraphic.CIRCLE_FILLED);
            scs.addYoGraphic(originGraphic);
            scs.addYoGraphic(goalGraphic);
         }
      }

      scs.addRegistry(registry);
      scs.startSimulationThread();

      ThreadTools.sleepForever();
   }
}
