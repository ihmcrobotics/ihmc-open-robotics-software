package us.ihmc.quadrupedRobotics.communication;

import controller_msgs.msg.dds.*;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.quadrupedRobotics.planning.QuadrupedTimedOrientedStep;
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

   public static QuadrupedBodyOrientationMessage createQuadrupedWorldFrameYawMessage(List<QuadrupedTimedOrientedStep> steps, double desiredYawVelocity)
   {
      QuadrupedBodyOrientationMessage message = new QuadrupedBodyOrientationMessage();
      message.setIsAnOffsetOrientation(false);
      message.setIsExpressedInAbsoluteTime(true);

      SO3TrajectoryMessage trajectoryMessage = new SO3TrajectoryMessage();
      for (int i = 0; i < steps.size(); i++)
      {
         QuadrupedTimedOrientedStep step = steps.get(i);
         double yawVelocity = (i == steps.size() - 1) ? 0.0 : desiredYawVelocity;
         Quaternion orientation = new Quaternion(step.getStepYaw(), 0.0, 0.0);
         Vector3D velocity = new Vector3D(0.0, 0.0, yawVelocity);

         SO3TrajectoryPointMessage orientationWaypoint = HumanoidMessageTools.createSO3TrajectoryPointMessage(step.getTimeInterval().getEndTime(), orientation, velocity);
         trajectoryMessage.getTaskspaceTrajectoryPoints().add().set(orientationWaypoint);
      }
      trajectoryMessage.getSelectionMatrix().setXSelected(false);
      trajectoryMessage.getSelectionMatrix().setYSelected(false);
      trajectoryMessage.getSelectionMatrix().setZSelected(true);
      message.getSo3Trajectory().set(trajectoryMessage);

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

   /**
    * Use this constructor to go straight to the given end point. Set the id of the message to
    * {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    *
    * @param trajectoryTime how long it takes to reach the desired height.
    * @param desiredHeight desired pelvis height expressed in data frame
    * @param trajectoryReferenceFrame the frame in which the height will be executed
    * @param dataReferenceFrame the frame the desiredHeight is expressed in, the height will be
    *           changed to the trajectory frame on the controller side
    */
   public static QuadrupedBodyHeightMessage createQuadrupedBodyHeightMessage(double trajectoryTime, double desiredHeight,
                                                                             ReferenceFrame trajectoryReferenceFrame, ReferenceFrame dataReferenceFrame)
   {
      QuadrupedBodyHeightMessage message = new QuadrupedBodyHeightMessage();
      message.getEuclideanTrajectory().set(HumanoidMessageTools.createEuclideanTrajectoryMessage(trajectoryTime, new Point3D(0.0, 0.0, desiredHeight),
                                                                                                 trajectoryReferenceFrame.hashCode()));
      message.getEuclideanTrajectory().getFrameInformation().setDataReferenceFrameId(MessageTools.toFrameId(dataReferenceFrame));
      message.getEuclideanTrajectory().getSelectionMatrix().setXSelected(false);
      message.getEuclideanTrajectory().getSelectionMatrix().setYSelected(false);
      message.getEuclideanTrajectory().getSelectionMatrix().setZSelected(true);
      return message;
   }

   /**
    * Use this constructor to go straight to the given end point. The trajectory and data frame are
    * set to world frame Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    *
    * @param trajectoryTime how long it takes to reach the desired height.
    * @param desiredHeight desired pelvis height expressed in world frame.
    */
   public static QuadrupedBodyHeightMessage createQuadrupedBodyHeightMessage(double trajectoryTime, double desiredHeight)
   {
      QuadrupedBodyHeightMessage message = new QuadrupedBodyHeightMessage();
      message.getEuclideanTrajectory().set(HumanoidMessageTools.createEuclideanTrajectoryMessage(trajectoryTime, new Point3D(0.0, 0.0, desiredHeight),
                                                                                          ReferenceFrame.getWorldFrame()));
      message.getEuclideanTrajectory().getFrameInformation().setDataReferenceFrameId(MessageTools.toFrameId(ReferenceFrame.getWorldFrame()));
      message.getEuclideanTrajectory().getSelectionMatrix().setXSelected(false);
      message.getEuclideanTrajectory().getSelectionMatrix().setYSelected(false);
      message.getEuclideanTrajectory().getSelectionMatrix().setZSelected(true);
      return message;
   }

   /**
    * Use this constructor to build a message with more than one trajectory point. This constructor
    * only allocates memory for the trajectory points, you need to call {@link #setTrajectoryPoint}
    * for each trajectory point afterwards. Set the id of the message to
    * {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    */
   public static QuadrupedBodyHeightMessage createPelvisHeightTrajectoryMessage()
   {
      QuadrupedBodyHeightMessage message = new QuadrupedBodyHeightMessage();
      message.getEuclideanTrajectory().getSelectionMatrix().setXSelected(false);
      message.getEuclideanTrajectory().getSelectionMatrix().setYSelected(false);
      message.getEuclideanTrajectory().getSelectionMatrix().setZSelected(true);
      return message;
   }
}
