package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.manual;

import controller_msgs.msg.dds.QuadrupedTimedStepListMessage;
import controller_msgs.msg.dds.QuadrupedTimedStepMessage;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.sensorProcessing.frames.CommonQuadrupedReferenceFrames;

public class QuadrupedManualFootstepPlanGenerator
{
   public QuadrupedTimedStepListMessage generateSteps(boolean trot,
                                                      RobotQuadrant firstFoot,
                                                      double swingHeight,
                                                      double stepHeight,
                                                      double stepLength,
                                                      double stepWidth,
                                                      double stepDuration,
                                                      double dwellTime,
                                                      int numberOfSteps,
                                                      QuadrupedXGaitSettingsReadOnly xGaitSettings,
                                                      CommonQuadrupedReferenceFrames referenceFrames)
   {
      if (trot)
      {
         return generateTrotSteps(firstFoot,
                                  swingHeight,
                                  stepHeight,
                                  stepLength,
                                  stepWidth,
                                  stepDuration,
                                  dwellTime,
                                  numberOfSteps,
                                  xGaitSettings,
                                  referenceFrames);
      }
      else
      {
         return generateCrawlSteps(firstFoot,
                                   swingHeight,
                                   stepHeight,
                                   stepLength,
                                   stepWidth,
                                   stepDuration,
                                   dwellTime,
                                   numberOfSteps,
                                   xGaitSettings,
                                   referenceFrames);
      }
   }

   public QuadrupedTimedStepListMessage generateTrotSteps(RobotQuadrant firstFoot,
                                                          double swingHeight,
                                                          double stepHeight,
                                                          double stepLength,
                                                          double stepWidth,
                                                          double stepDuration,
                                                          double dwellTime,
                                                          int numberOfSteps,
                                                          QuadrupedXGaitSettingsReadOnly xGaitSettings,
                                                          CommonQuadrupedReferenceFrames referenceFrames)
   {
      QuadrupedTimedStepListMessage stepListMessage = new QuadrupedTimedStepListMessage();

      stepListMessage.setIsExpressedInAbsoluteTime(false);
      stepListMessage.getQuadrupedStepList().clear();

      double stanceLength = xGaitSettings.getStanceLength();
      double stanceWidth = xGaitSettings.getStanceWidth();

      ReferenceFrame supportFrame = referenceFrames.getCenterOfFeetZUpFrameAveragingLowestZHeightsAcrossEnds();
      FramePoint3D centerPoint = new FramePoint3D(supportFrame);
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         FramePoint3D solePosition = new FramePoint3D(referenceFrames.getSoleFrame(robotQuadrant));
         solePosition.changeFrame(supportFrame);
         centerPoint.add(solePosition);
      }
      centerPoint.scale(0.25);

      QuadrantDependentList<FramePoint3D> nominalPositions = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         FramePoint3D nominalPosition = new FramePoint3D(centerPoint);
         nominalPosition.addX(0.5 * robotQuadrant.getEnd().negateIfHindEnd(stanceLength));
         nominalPosition.addY(0.5 * robotQuadrant.getSide().negateIfRightSide(stanceWidth));
         nominalPositions.put(robotQuadrant, nominalPosition);
      }

      double timeDelay = 0.0;
      double lengthOffset = 0.0;
      double widthOffset = 0.0;

      for (int i = 0; i < numberOfSteps; i++)
      {
         lengthOffset += stepLength;
         widthOffset += stepWidth;

         RobotQuadrant frontQuadrant = firstFoot;
         RobotQuadrant rearQuadrant = firstFoot.getDiagonalOppositeQuadrant();

         QuadrupedTimedStepMessage frontFootMessage = stepListMessage.getQuadrupedStepList().add();
         QuadrupedTimedStepMessage hindFootMessage = stepListMessage.getQuadrupedStepList().add();

         nominalPositions.get(frontQuadrant).changeFrame(supportFrame);
         nominalPositions.get(rearQuadrant).changeFrame(supportFrame);

         FramePoint3D frontPosition = new FramePoint3D(nominalPositions.get(frontQuadrant));
         FramePoint3D rearPosition = new FramePoint3D(nominalPositions.get(rearQuadrant));
         frontPosition.addX(lengthOffset);
         frontPosition.addY(widthOffset);
         rearPosition.addX(lengthOffset);
         rearPosition.addY(widthOffset);

         frontPosition.changeFrame(ReferenceFrame.getWorldFrame());
         rearPosition.changeFrame(ReferenceFrame.getWorldFrame());
         frontPosition.addZ(stepHeight);
         rearPosition.addZ(stepHeight);

         frontFootMessage.getTimeInterval().setStartTime(timeDelay);
         frontFootMessage.getTimeInterval().setEndTime(timeDelay + stepDuration);
         frontFootMessage.getQuadrupedStepMessage().setGroundClearance(swingHeight);
         frontFootMessage.getQuadrupedStepMessage().setRobotQuadrant(frontQuadrant.toByte());
         frontFootMessage.getQuadrupedStepMessage().getGoalPosition().set(frontPosition);

         hindFootMessage.getTimeInterval().setStartTime(timeDelay);
         hindFootMessage.getTimeInterval().setEndTime(timeDelay + stepDuration);
         hindFootMessage.getQuadrupedStepMessage().setGroundClearance(swingHeight);
         hindFootMessage.getQuadrupedStepMessage().setRobotQuadrant(rearQuadrant.toByte());
         hindFootMessage.getQuadrupedStepMessage().getGoalPosition().set(rearPosition);


         firstFoot = firstFoot.getAcrossBodyQuadrant();
         timeDelay += stepDuration + dwellTime;
      }

      return stepListMessage;
   }

   public QuadrupedTimedStepListMessage generateCrawlSteps(RobotQuadrant firstFoot,
                                                           double swingHeight,
                                                           double stepHeight,
                                                           double stepLength,
                                                           double stepWidth,
                                                           double stepDuration,
                                                           double dwellTime,
                                                           int numberOfSteps,
                                                           QuadrupedXGaitSettingsReadOnly xGaitSettings,
                                                           CommonQuadrupedReferenceFrames referenceFrames)
   {

      QuadrupedTimedStepListMessage stepListMessage = new QuadrupedTimedStepListMessage();

      stepListMessage.setIsExpressedInAbsoluteTime(false);
      stepListMessage.getQuadrupedStepList().clear();

      double currentTime = 0.0;

      double stanceLength = xGaitSettings.getStanceLength();
      double stanceWidth = xGaitSettings.getStanceWidth();

      ReferenceFrame supportFrame = referenceFrames.getCenterOfFeetZUpFrameAveragingLowestZHeightsAcrossEnds();
      FramePoint3D centerPoint = new FramePoint3D(supportFrame);
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         FramePoint3D solePosition = new FramePoint3D(referenceFrames.getSoleFrame(robotQuadrant));
         solePosition.changeFrame(supportFrame);
         centerPoint.add(solePosition);
      }
      centerPoint.scale(0.25);

      QuadrantDependentList<FramePoint3D> nominalPositions = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         FramePoint3D nominalPosition = new FramePoint3D(centerPoint);
         nominalPosition.addX(0.5 * robotQuadrant.getEnd().negateIfHindEnd(stanceLength));
         nominalPosition.addY(0.5 * robotQuadrant.getSide().negateIfRightSide(stanceWidth));
         nominalPositions.put(robotQuadrant, nominalPosition);
      }

      double lengthOffset = 0.0;
      double widthOffset = 0.0;

      for (int i = 0; i < Math.ceil(numberOfSteps / 2.0); i++)
      {
         lengthOffset += 0.5 * stepLength;
         widthOffset += 0.5 * stepWidth;

         QuadrupedTimedStepMessage firstFootMessage = stepListMessage.getQuadrupedStepList().add();

         nominalPositions.get(firstFoot).changeFrame(supportFrame);

         FramePoint3D firstPosition = new FramePoint3D(nominalPositions.get(firstFoot));
         firstPosition.addX(lengthOffset);
         firstPosition.addY(widthOffset);

         firstPosition.changeFrame(ReferenceFrame.getWorldFrame());
         firstPosition.addZ(stepHeight);

         firstFootMessage.getTimeInterval().setStartTime(currentTime);
         firstFootMessage.getTimeInterval().setEndTime(currentTime + stepDuration);
         firstFootMessage.getQuadrupedStepMessage().setGroundClearance(swingHeight);
         firstFootMessage.getQuadrupedStepMessage().setRobotQuadrant(firstFoot.toByte());
         firstFootMessage.getQuadrupedStepMessage().getGoalPosition().set(firstPosition);


         if (2 * i + 1 < numberOfSteps)
         {
            lengthOffset += 0.5 * stepLength;
            widthOffset += 0.5 * stepWidth;

            QuadrupedTimedStepMessage oppositeFootMessage = stepListMessage.getQuadrupedStepList().add();

            currentTime += 0.5 * stepDuration + dwellTime;
            RobotQuadrant oppositeQuadrant = firstFoot.getDiagonalOppositeQuadrant();

            nominalPositions.get(oppositeQuadrant).changeFrame(supportFrame);

            FramePoint3D secondPosition = new FramePoint3D(nominalPositions.get(oppositeQuadrant));
            secondPosition.addX(lengthOffset);
            secondPosition.addY(widthOffset);

            secondPosition.changeFrame(ReferenceFrame.getWorldFrame());
            secondPosition.addZ(stepHeight);

            oppositeFootMessage.getTimeInterval().setStartTime(currentTime);
            oppositeFootMessage.getTimeInterval().setEndTime(currentTime + stepDuration);
            oppositeFootMessage.getQuadrupedStepMessage().setGroundClearance(swingHeight);
            oppositeFootMessage.getQuadrupedStepMessage().setRobotQuadrant(oppositeQuadrant.toByte());
            oppositeFootMessage.getQuadrupedStepMessage().getGoalPosition().set(secondPosition);
         }
         else
         {
            break;
         }

         firstFoot = firstFoot.getAcrossBodyQuadrant();

         currentTime += 0.5 * stepDuration + dwellTime;
      }

      return stepListMessage;
   }
}
