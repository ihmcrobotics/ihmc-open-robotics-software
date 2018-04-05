package us.ihmc.quadrupedRobotics.communication;

import controller_msgs.msg.dds.QuadrupedTimedStepListMessage;
import controller_msgs.msg.dds.QuadrupedTimedStepMessage;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.quadrupedRobotics.planning.QuadrupedTimedStep;
import us.ihmc.quadrupedRobotics.util.TimeInterval;
import us.ihmc.robotics.robotSide.RobotQuadrant;

import java.util.ArrayList;
import java.util.List;

public class QuadrupedMessageTools
{

   public static QuadrupedTimedStepMessage createQuadrupedTimedStepMessage(RobotQuadrant robotQuadrant, Point3D goalPosition, double groundClearance,
                                                                           TimeInterval timeInterval)
   {
      QuadrupedTimedStepMessage message = new QuadrupedTimedStepMessage();
      message.getQuadrupedStepMessage().setRobotQuadrant(robotQuadrant.toByte());
      message.getQuadrupedStepMessage().getGoalPosition().set(goalPosition);
      message.getQuadrupedStepMessage().setGroundClearance(groundClearance);
      message.getTimeInterval().setEndTime(timeInterval.getEndTime());
      message.getTimeInterval().setStartTime(timeInterval.getStartTime());

      return message;
   }

   public static QuadrupedTimedStepMessage createQuadrupedTimedStepMessage(RobotQuadrant robotQuadrant, Point3D goalPosition, double groundClearance,
                                                                           double startTime, double endTime)
   {
      QuadrupedTimedStepMessage message = new QuadrupedTimedStepMessage();
      message.getQuadrupedStepMessage().setRobotQuadrant(robotQuadrant.toByte());
      message.getQuadrupedStepMessage().getGoalPosition().set(goalPosition);
      message.getQuadrupedStepMessage().setGroundClearance(groundClearance);
      message.getTimeInterval().setStartTime(startTime);
      message.getTimeInterval().setEndTime(endTime);

      return message;
   }

   public static QuadrupedTimedStepMessage createQuadrupedTimedStepMessage(QuadrupedTimedStep step)
   {
      QuadrupedTimedStepMessage message = new QuadrupedTimedStepMessage();
      message.getQuadrupedStepMessage().setRobotQuadrant(step.getRobotQuadrant().toByte());
      step.getGoalPosition(message.getQuadrupedStepMessage().getGoalPosition());
      message.getQuadrupedStepMessage().setGroundClearance(step.getGroundClearance());
      message.getTimeInterval().setStartTime(step.getTimeInterval().getStartTime());
      message.getTimeInterval().setEndTime(step.getTimeInterval().getEndTime());

      return message;
   }

   public static QuadrupedTimedStepListMessage createQuadrupedTimedStepListMessage(List<QuadrupedTimedStepMessage> stepMessages)
   {
      return createQuadrupedTimedStepListMessage(stepMessages, true);
   }


   public static QuadrupedTimedStepListMessage createQuadrupedTimedStepListMessage(List<QuadrupedTimedStepMessage> stepMessages,
                                                                                   boolean isExpressedInAbsoluteTime)
   {
      QuadrupedTimedStepListMessage message = new QuadrupedTimedStepListMessage();
      MessageTools.copyData(stepMessages, message.getQuadrupedStepList());
      message.setIsExpressedInAbsoluteTime(isExpressedInAbsoluteTime);

      return message;
   }
}
