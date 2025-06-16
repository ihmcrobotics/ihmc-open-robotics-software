package us.ihmc.avatar.psyonicAbilityHand;

import us.ihmc.abilityhand.AHWrapper;
import us.ihmc.abilityhand.FloatArray6;
import us.ihmc.abilityhand.global.abilityhand;
import us.ihmc.robotics.robotSide.RobotSide;

public class ROS2AbilityHand implements AbilityHand
{
   private final RobotSide handSide;
   private AHWrapper wrapperHand;
   private float indexFingerPosition;
   private float middleFingerPosition;
   private float ringFingerPosition;
   private float pinkyFingerPosition;
   private float thumbFlexorPosition;
   private float thumbRotatorPosition;

   private final FloatArray6 command = new FloatArray6();

   public ROS2AbilityHand(RobotSide handSide)
   {
      this.handSide = handSide;
   }

   public void connect()
   {
      if (handSide == RobotSide.LEFT)
      {
         wrapperHand = new AHWrapper((byte) 0x50, 1000000);
         wrapperHand.connect();
      }
      else
      {
         wrapperHand = new AHWrapper((byte) 0x51, 1000000);
      }
   }

   @Override
   public void update()
   {
      command.put(0, indexFingerPosition);
      command.put(1, middleFingerPosition);
      command.put(2, ringFingerPosition);
      command.put(3, pinkyFingerPosition);
      command.put(4, thumbFlexorPosition);
      command.put(5, thumbRotatorPosition);
      wrapperHand.read_write_once(command, abilityhand.POSITION, (byte) 0);
   }

   @Override
   public void setIndexFingerPositionCommand(float indexFingerPositionCommand)
   {
      command.put(0, indexFingerPositionCommand);
      indexFingerPosition = indexFingerPositionCommand;
   }

   @Override
   public void setMiddleFingerPositionCommand(float middleFingerPositionCommand)
   {
      command.put(1, middleFingerPositionCommand);
      middleFingerPosition = middleFingerPositionCommand;
   }

   @Override
   public void setRingFingerPositionCommand(float ringFingerPositionCommand)
   {
      command.put(2, ringFingerPositionCommand);
      ringFingerPosition = ringFingerPositionCommand;
   }

   @Override
   public void setPinkyFingerPositionCommand(float pinkyFingerPositionCommand)
   {
      command.put(3, pinkyFingerPositionCommand);
      pinkyFingerPosition = pinkyFingerPositionCommand;
   }

   @Override
   public void setThumbFlexorPositionCommand(float thumbFlexorPositionCommand)
   {
      command.put(4, thumbFlexorPositionCommand);
      thumbFlexorPosition = thumbFlexorPositionCommand;
   }

   @Override
   public void setThumbRotatorPositionCommand(float thumbRotatorPositionCommand)
   {
      command.put(5, thumbRotatorPositionCommand);
      thumbRotatorPosition = thumbRotatorPositionCommand;
   }

   @Override
   public float getIndexFingerPositionStatus()
   {
      return indexFingerPosition;
   }

   @Override
   public float getMiddleFingerPositionStatus()
   {
      return middleFingerPosition;
   }

   @Override
   public float getRingFingerPositionStatus()
   {
      return ringFingerPosition;
   }

   @Override
   public float getPinkyFingerPositionStatus()
   {
      return pinkyFingerPosition;
   }

   @Override
   public float getThumbFlexorPositionStatus()
   {
      return thumbFlexorPosition;
   }

   @Override
   public float getThumbRotatorPositionStatus()
   {
      return thumbRotatorPosition;
   }
}
