package us.ihmc.avatar.psyonicAbilityHand;

import us.ihmc.abilityhand.AHWrapper;
import us.ihmc.abilityhand.FloatArray6;
import us.ihmc.abilityhand.global.abilityhand;
import us.ihmc.robotics.robotSide.RobotSide;

public class ROS2AbilityHand implements AbilityHand
{
   private final RobotSide handSide;
   private AHWrapper wrapperHand;

   private final FloatArray6 command = new FloatArray6();

   public ROS2AbilityHand(RobotSide handSide)
   {
      this.handSide = handSide;
   }

   public void connect()
   {
      if (handSide == RobotSide.LEFT)
      {
         wrapperHand = new AHWrapper((byte) 0x50, 460800);
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
      command.put(0, getIndexFingerPositionStatus());
      command.put(1, getMiddleFingerPositionStatus());
      command.put(2, getRingFingerPositionStatus());
      command.put(3, getPinkyFingerPositionStatus());
      command.put(4, getThumbFlexorPositionStatus());
      command.put(5, getThumbRotatorPositionStatus());
      wrapperHand.read_write_once(command, abilityhand.POSITION, (byte) 0);
   }

   @Override
   public void setIndexFingerPositionCommand(float indexFingerPositionCommand)
   {
      command.put(0, indexFingerPositionCommand);
   }

   @Override
   public void setMiddleFingerPositionCommand(float middleFingerPositionCommand)
   {
      command.put(1, middleFingerPositionCommand);
   }

   @Override
   public void setRingFingerPositionCommand(float ringFingerPositionCommand)
   {
      command.put(2, ringFingerPositionCommand);
   }

   @Override
   public void setPinkyFingerPositionCommand(float pinkyFingerPositionCommand)
   {
      command.put(3, pinkyFingerPositionCommand);
   }

   @Override
   public void setThumbFlexorPositionCommand(float thumbFlexorPositionCommand)
   {
      command.put(4, thumbFlexorPositionCommand);
   }

   @Override
   public void setThumbRotatorPositionCommand(float thumbRotatorPositionCommand)
   {
      command.put(5, thumbRotatorPositionCommand);
   }

   @Override
   public float getIndexFingerPositionStatus()
   {
      return command.get(0);
   }

   @Override
   public float getMiddleFingerPositionStatus()
   {
      return command.get(1);
   }

   @Override
   public float getRingFingerPositionStatus()
   {
      return command.get(2);
   }

   @Override
   public float getPinkyFingerPositionStatus()
   {
      return command.get(3);
   }

   @Override
   public float getThumbFlexorPositionStatus()
   {
      return command.get(4);
   }

   @Override
   public float getThumbRotatorPositionStatus()
   {
      return command.get(5);
   }
}
