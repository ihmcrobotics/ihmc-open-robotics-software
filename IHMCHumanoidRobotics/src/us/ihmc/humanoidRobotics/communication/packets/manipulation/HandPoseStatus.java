package us.ihmc.humanoidRobotics.communication.packets.manipulation;

import java.util.Random;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import us.ihmc.communication.packets.Packet;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandPosePacket.DataType;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.robotSide.RobotSide;

public class HandPoseStatus extends Packet<HandPoseStatus>
{
   public Point3d currentPosePosition;
   public Quat4d currentPoseOrientationInWorldFrame;

   public Point3d desiredPosePosition;
   public Quat4d desiredPoseOrientationInWorldFrame;

   public int robotSide;

   public int dataType;

   public boolean desiredFramePoseIsReached;
   public boolean timeOutIsReached;

   public enum Status
   {
      STARTED, COMPLETED, TIME_OUT;
   }

   public int status;

   public HandPoseStatus(Random random)
   {
      double maxValue = Double.MAX_VALUE / 2;
      currentPosePosition = RandomTools.generateRandomPoint(random, maxValue, maxValue, maxValue);
      currentPoseOrientationInWorldFrame = RandomTools.generateRandomQuaternion(random);
      desiredPosePosition = RandomTools.generateRandomPoint(random, maxValue, maxValue, maxValue);
      desiredPoseOrientationInWorldFrame = RandomTools.generateRandomQuaternion(random);
      
      robotSide = random.nextInt();
      dataType = random.nextInt();
      desiredFramePoseIsReached = random.nextBoolean(); 
      timeOutIsReached = random.nextBoolean(); 
   }
   
   public HandPoseStatus()
   {
      // Empty constructor for serialization
   }

   public HandPoseStatus(Point3d currentPosePosition, Quat4d currentPoseOrientationInWorldFrame, Point3d desiredPosePosition,
         Quat4d desiredPoseOrientationInWorldFrame, RobotSide robotSide)
   {
      this.currentPosePosition = currentPosePosition;
      this.currentPoseOrientationInWorldFrame = currentPoseOrientationInWorldFrame;

      this.desiredPosePosition = desiredPosePosition;
      this.desiredPoseOrientationInWorldFrame = desiredPoseOrientationInWorldFrame;

      if (robotSide == RobotSide.LEFT)
         this.robotSide = 0;
      else
         this.robotSide = 1;
   }

   public static HandPoseStatus createHandPoseIsStarted(RobotSide robotSide)
   {
      HandPoseStatus handPoseStatus = new HandPoseStatus();
      handPoseStatus.timeOutIsReached = false;
      handPoseStatus.desiredFramePoseIsReached = false;
      handPoseStatus.currentPosePosition = null;
      handPoseStatus.currentPoseOrientationInWorldFrame = null;
      handPoseStatus.desiredPosePosition = null;
      handPoseStatus.desiredPoseOrientationInWorldFrame = null;

      handPoseStatus.status = 0;
      if (robotSide == RobotSide.LEFT)
         handPoseStatus.robotSide = 0;
      else
         handPoseStatus.robotSide = 1;

      return handPoseStatus;
   }

   public static HandPoseStatus createPositionIsReachedPacket(RobotSide robotSide)
   {
      HandPoseStatus handPoseStatus = new HandPoseStatus();
      handPoseStatus.timeOutIsReached = false;
      handPoseStatus.desiredFramePoseIsReached = true;
      handPoseStatus.status = 1;
      handPoseStatus.currentPosePosition = null;
      handPoseStatus.currentPoseOrientationInWorldFrame = null;
      handPoseStatus.desiredPosePosition = null;
      handPoseStatus.desiredPoseOrientationInWorldFrame = null;

      if (robotSide == RobotSide.LEFT)
         handPoseStatus.robotSide = 0;
      else
         handPoseStatus.robotSide = 1;

      return handPoseStatus;
   }

   public static HandPoseStatus createTimeOutIsReachedPacket(RobotSide robotSide)
   {
      HandPoseStatus handPoseStatus = new HandPoseStatus();
      handPoseStatus.timeOutIsReached = true;
      handPoseStatus.desiredFramePoseIsReached = false;
      handPoseStatus.status = 2;
      handPoseStatus.currentPosePosition = null;
      handPoseStatus.currentPoseOrientationInWorldFrame = null;
      handPoseStatus.desiredPosePosition = null;
      handPoseStatus.desiredPoseOrientationInWorldFrame = null;

      if (robotSide == RobotSide.LEFT)
         handPoseStatus.robotSide = 0;
      else
         handPoseStatus.robotSide = 1;

      return handPoseStatus;
   }

   public Point3d getCurrentPosePosition()
   {
      return currentPosePosition;
   }

   public Point3d getDesiredPosePosition()
   {
      return desiredPosePosition;
   }

   public Quat4d getCurrentPoseOrientationInWorldFrame()
   {
      return currentPoseOrientationInWorldFrame;
   }

   public Quat4d getDesiredPoseOrientationInWorldFrame()
   {
      return desiredPoseOrientationInWorldFrame;
   }

   public Status getStatus()
   {
      if (status == 0)
         return Status.STARTED;
      if (status == 1)
         return Status.COMPLETED;
      if (status == 2)
         return Status.TIME_OUT;
      return null;
   }

   public RobotSide getRobotSide()
   {
      if (robotSide == 0)
         return RobotSide.LEFT;
      else
         return RobotSide.RIGHT;
   }

   public DataType getDataType()
   {
      if (dataType == 0)
         return DataType.HAND_POSE;
      if (dataType == 1)
         return DataType.JOINT_ANGLES;
      return null;
   }

   @Override
   public boolean epsilonEquals(HandPoseStatus other, double epsilon)
   {
      boolean currentPosePositionEquals = currentPosePosition.epsilonEquals(other.currentPosePosition, epsilon);
      boolean currentPoseOrientationInWorldFrameEquals = currentPoseOrientationInWorldFrame.epsilonEquals(other.currentPoseOrientationInWorldFrame, epsilon);

      boolean desiredPosePositionEquals = desiredPosePosition.epsilonEquals(other.desiredPosePosition, epsilon);
      boolean desiredPoseOrientationInWorldFrameEquals = desiredPoseOrientationInWorldFrame.epsilonEquals(other.desiredPoseOrientationInWorldFrame, epsilon);

      boolean robotSideEquals = robotSide == other.robotSide;
      boolean dataTypeEquals = dataType == other.dataType;
      boolean statusEquals = status == other.status;

      boolean desiredFramePoseIsReachedEquals = desiredFramePoseIsReached == other.desiredFramePoseIsReached;
      boolean timeOutIsReachedEquals = timeOutIsReached == other.timeOutIsReached;

      return currentPoseOrientationInWorldFrameEquals && currentPosePositionEquals && desiredPosePositionEquals && desiredPoseOrientationInWorldFrameEquals
            && robotSideEquals && dataTypeEquals && statusEquals && desiredFramePoseIsReachedEquals && timeOutIsReachedEquals;
   }

}
