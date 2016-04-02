package us.ihmc.robotiq.communication.registers;

import org.apache.commons.collections.map.MultiKeyMap;

import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.robotiq.RobotiqGraspMode;
import us.ihmc.robotiq.communication.Finger;
import us.ihmc.robotiq.communication.InvalidFingerException;

public class FingerPositionRequestRegister extends RobotiqOutputRegister
{
   private MultiKeyMap fingerPositionMap = new MultiKeyMap();
   {
      fingerPositionMap.put(RobotiqGraspMode.BASIC_MODE, HandConfiguration.OPEN, Finger.FINGER_A, (byte)0x00);
      fingerPositionMap.put(RobotiqGraspMode.BASIC_MODE, HandConfiguration.OPEN, Finger.FINGER_B, (byte)0x00);
      fingerPositionMap.put(RobotiqGraspMode.BASIC_MODE, HandConfiguration.OPEN, Finger.FINGER_C, (byte)0x00);
      fingerPositionMap.put(RobotiqGraspMode.BASIC_MODE, HandConfiguration.OPEN, Finger.SCISSOR, (byte)0x8C);
      
      fingerPositionMap.put(RobotiqGraspMode.BASIC_MODE, HandConfiguration.CLOSE, Finger.FINGER_A, (byte)0xFF);
      fingerPositionMap.put(RobotiqGraspMode.BASIC_MODE, HandConfiguration.CLOSE, Finger.FINGER_B, (byte)0xFF);
      fingerPositionMap.put(RobotiqGraspMode.BASIC_MODE, HandConfiguration.CLOSE, Finger.FINGER_C, (byte)0xFF);
      fingerPositionMap.put(RobotiqGraspMode.BASIC_MODE, HandConfiguration.CLOSE, Finger.SCISSOR, (byte)0x8C);
      
      fingerPositionMap.put(RobotiqGraspMode.PINCH_MODE, HandConfiguration.OPEN, Finger.FINGER_A, (byte)0x00);
      fingerPositionMap.put(RobotiqGraspMode.PINCH_MODE, HandConfiguration.OPEN, Finger.FINGER_B, (byte)0x00);
      fingerPositionMap.put(RobotiqGraspMode.PINCH_MODE, HandConfiguration.OPEN, Finger.FINGER_C, (byte)0x00);
      fingerPositionMap.put(RobotiqGraspMode.PINCH_MODE, HandConfiguration.OPEN, Finger.SCISSOR, (byte)0xDC);
      
      fingerPositionMap.put(RobotiqGraspMode.PINCH_MODE, HandConfiguration.CLOSE, Finger.FINGER_A, (byte)0x78);
      fingerPositionMap.put(RobotiqGraspMode.PINCH_MODE, HandConfiguration.CLOSE, Finger.FINGER_B, (byte)0x78);
      fingerPositionMap.put(RobotiqGraspMode.PINCH_MODE, HandConfiguration.CLOSE, Finger.FINGER_C, (byte)0x78);
      fingerPositionMap.put(RobotiqGraspMode.PINCH_MODE, HandConfiguration.CLOSE, Finger.SCISSOR, (byte)0xDC);
      
      fingerPositionMap.put(RobotiqGraspMode.WIDE_MODE, HandConfiguration.OPEN, Finger.FINGER_A, (byte)0x00);
      fingerPositionMap.put(RobotiqGraspMode.WIDE_MODE, HandConfiguration.OPEN, Finger.FINGER_B, (byte)0x00);
      fingerPositionMap.put(RobotiqGraspMode.WIDE_MODE, HandConfiguration.OPEN, Finger.FINGER_C, (byte)0x00);
      fingerPositionMap.put(RobotiqGraspMode.WIDE_MODE, HandConfiguration.OPEN, Finger.SCISSOR, (byte)0x19);
      
      fingerPositionMap.put(RobotiqGraspMode.WIDE_MODE, HandConfiguration.CLOSE, Finger.FINGER_A, (byte)0xFF);
      fingerPositionMap.put(RobotiqGraspMode.WIDE_MODE, HandConfiguration.CLOSE, Finger.FINGER_B, (byte)0xFF);
      fingerPositionMap.put(RobotiqGraspMode.WIDE_MODE, HandConfiguration.CLOSE, Finger.FINGER_C, (byte)0xFF);
      fingerPositionMap.put(RobotiqGraspMode.WIDE_MODE, HandConfiguration.CLOSE, Finger.SCISSOR, (byte)0x19);
      
      fingerPositionMap.put(RobotiqGraspMode.SCISSOR_MODE, HandConfiguration.OPEN, Finger.FINGER_A, (byte)0x00);
      fingerPositionMap.put(RobotiqGraspMode.SCISSOR_MODE, HandConfiguration.OPEN, Finger.FINGER_B, (byte)0x00);
      fingerPositionMap.put(RobotiqGraspMode.SCISSOR_MODE, HandConfiguration.OPEN, Finger.FINGER_C, (byte)0x00);
      fingerPositionMap.put(RobotiqGraspMode.SCISSOR_MODE, HandConfiguration.OPEN, Finger.SCISSOR, (byte)0x00);
      
      fingerPositionMap.put(RobotiqGraspMode.SCISSOR_MODE, HandConfiguration.CLOSE, Finger.FINGER_A, (byte)0x00);
      fingerPositionMap.put(RobotiqGraspMode.SCISSOR_MODE, HandConfiguration.CLOSE, Finger.FINGER_B, (byte)0x00);
      fingerPositionMap.put(RobotiqGraspMode.SCISSOR_MODE, HandConfiguration.CLOSE, Finger.FINGER_C, (byte)0x00);
      fingerPositionMap.put(RobotiqGraspMode.SCISSOR_MODE, HandConfiguration.CLOSE, Finger.SCISSOR, (byte)0xFF);
   }
   
   private final Finger finger;
   private final int index;
   private byte position;

   public FingerPositionRequestRegister(Finger finger)
   {
      this.finger = finger;
      switch(finger)
      {
         case FINGER_A:
            index = 3;
            break;
         case FINGER_B:
            index = 6;
            break;
         case FINGER_C:
            index = 9;
            break;
         case SCISSOR:
            index = 12;
            break;
         default:
            throw new InvalidFingerException(finger);
      }
      
      position = (byte)0x00;
   }
   
   public void setFingerPosition(RobotiqGraspMode graspMode, HandConfiguration handConfiguration)
   {
      byte position = (byte)fingerPositionMap.get(graspMode, handConfiguration, finger);
      this.position = position;
   }
   
   @Override
   public byte getRegisterValue()
   {
      return position;
   }

   @Override
   public int getRegisterIndex()
   {
      return index;
   }
   
   @Override
   public void resetRegister()
   {
      position = (byte)0x0;
   }
}
