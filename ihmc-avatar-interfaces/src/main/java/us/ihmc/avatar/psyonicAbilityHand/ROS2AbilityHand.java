package us.ihmc.avatar.psyonicAbilityHand;

import us.ihmc.abilityhand.AHWrapper;
import us.ihmc.abilityhand.FloatArray6;
import us.ihmc.abilityhand.global.abilityhand;
import us.ihmc.robotics.robotSide.RobotSide;

public class ROS2AbilityHand implements AbilityHand
{
   private final RobotSide handSide;
   private AHWrapper wrapperHand;
   private float indexFingerPosition = 30.0f;
   private float middleFingerPosition = 30.0f;
   private float ringFingerPosition = 30.0f;
   private float pinkyFingerPosition = 30.0f;
   private float thumbFlexorPosition = 30.0f;
   private float thumbRotatorPosition = -30.0f;

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
      // Sets hand to starting position
      for(int i = 0; i <5; i++)
      {
         command.put(i, 30.0f);
      }
      command.put(5, -30.0f);
      wrapperHand.read_write_once(command, abilityhand.POSITION, (byte) 0);
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
      if(command.get(5) > 0.0f)
      {
         command.put(5, -command.get(5));
      }
      wrapperHand.write_once(command, abilityhand.POSITION, (byte) 0);
      wrapperHand.read_once((byte) 0);
   }

   @Override
   public void setIndexFingerPositionCommand(float indexFingerPositionCommand)
   {
      indexFingerPosition = indexFingerPositionCommand;
   }

   @Override
   public void setMiddleFingerPositionCommand(float middleFingerPositionCommand)
   {
      middleFingerPosition = middleFingerPositionCommand;
   }

   @Override
   public void setRingFingerPositionCommand(float ringFingerPositionCommand)
   {
      ringFingerPosition = ringFingerPositionCommand;
   }

   @Override
   public void setPinkyFingerPositionCommand(float pinkyFingerPositionCommand)
   {
      pinkyFingerPosition = pinkyFingerPositionCommand;
   }

   @Override
   public void setThumbFlexorPositionCommand(float thumbFlexorPositionCommand)
   {
      thumbFlexorPosition = thumbFlexorPositionCommand;
   }

   @Override
   public void setThumbRotatorPositionCommand(float thumbRotatorPositionCommand)
   {
      thumbRotatorPosition = thumbRotatorPositionCommand;
   }
   public void setAllFingers(float fingerPosition)
   {
      indexFingerPosition = fingerPosition;
      middleFingerPosition = fingerPosition;
      ringFingerPosition = fingerPosition;
      pinkyFingerPosition = fingerPosition;
      thumbFlexorPosition = fingerPosition;
      thumbRotatorPosition = -fingerPosition;
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

   public float[] getAllFingerPositions()
   {
      return new float[]{indexFingerPosition, middleFingerPosition, ringFingerPosition, pinkyFingerPosition, thumbFlexorPosition, thumbRotatorPosition};
   }

}
