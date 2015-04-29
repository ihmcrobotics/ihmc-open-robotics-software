package us.ihmc.robotiq.communication.registers;

import org.apache.commons.collections.map.MultiKeyMap;

import us.ihmc.communication.packets.dataobjects.FingerState;
import us.ihmc.robotiq.RobotiqGraspMode;
import us.ihmc.robotiq.communication.Finger;

public class FingerPositionRequestRegister implements RobotiqRegister
{
   private MultiKeyMap fingerPositionMap = new MultiKeyMap();
   {
      fingerPositionMap.put(RobotiqGraspMode.BASIC_MODE, FingerState.OPEN, Finger.FINGER_A, 0x00);
      fingerPositionMap.put(RobotiqGraspMode.BASIC_MODE, FingerState.OPEN, Finger.FINGER_B, 0x00);
      fingerPositionMap.put(RobotiqGraspMode.BASIC_MODE, FingerState.OPEN, Finger.FINGER_C, 0x00);
      fingerPositionMap.put(RobotiqGraspMode.BASIC_MODE, FingerState.OPEN, Finger.SCISSOR, 0x8C);
      
      fingerPositionMap.put(RobotiqGraspMode.BASIC_MODE, FingerState.CLOSE, Finger.FINGER_A, 0xFF);
      fingerPositionMap.put(RobotiqGraspMode.BASIC_MODE, FingerState.CLOSE, Finger.FINGER_B, 0xFF);
      fingerPositionMap.put(RobotiqGraspMode.BASIC_MODE, FingerState.CLOSE, Finger.FINGER_C, 0xFF);
      fingerPositionMap.put(RobotiqGraspMode.BASIC_MODE, FingerState.CLOSE, Finger.SCISSOR, 0x8C);
      
      fingerPositionMap.put(RobotiqGraspMode.PINCH_MODE, FingerState.OPEN, Finger.FINGER_A, 0x00);
      fingerPositionMap.put(RobotiqGraspMode.PINCH_MODE, FingerState.OPEN, Finger.FINGER_B, 0x00);
      fingerPositionMap.put(RobotiqGraspMode.PINCH_MODE, FingerState.OPEN, Finger.FINGER_C, 0x00);
      fingerPositionMap.put(RobotiqGraspMode.PINCH_MODE, FingerState.OPEN, Finger.SCISSOR, 0xDC);
      
      fingerPositionMap.put(RobotiqGraspMode.PINCH_MODE, FingerState.CLOSE, Finger.FINGER_A, 0x78);
      fingerPositionMap.put(RobotiqGraspMode.PINCH_MODE, FingerState.CLOSE, Finger.FINGER_B, 0x78);
      fingerPositionMap.put(RobotiqGraspMode.PINCH_MODE, FingerState.CLOSE, Finger.FINGER_C, 0x78);
      fingerPositionMap.put(RobotiqGraspMode.PINCH_MODE, FingerState.CLOSE, Finger.SCISSOR, 0xDC);
      
      fingerPositionMap.put(RobotiqGraspMode.WIDE_MODE, FingerState.OPEN, Finger.FINGER_A, 0x00);
      fingerPositionMap.put(RobotiqGraspMode.WIDE_MODE, FingerState.OPEN, Finger.FINGER_B, 0x00);
      fingerPositionMap.put(RobotiqGraspMode.WIDE_MODE, FingerState.OPEN, Finger.FINGER_C, 0x00);
      fingerPositionMap.put(RobotiqGraspMode.WIDE_MODE, FingerState.OPEN, Finger.SCISSOR, 0x19);
      
      fingerPositionMap.put(RobotiqGraspMode.WIDE_MODE, FingerState.CLOSE, Finger.FINGER_A, 0xFF);
      fingerPositionMap.put(RobotiqGraspMode.WIDE_MODE, FingerState.CLOSE, Finger.FINGER_B, 0xFF);
      fingerPositionMap.put(RobotiqGraspMode.WIDE_MODE, FingerState.CLOSE, Finger.FINGER_C, 0xFF);
      fingerPositionMap.put(RobotiqGraspMode.WIDE_MODE, FingerState.CLOSE, Finger.SCISSOR, 0x19);
      
      fingerPositionMap.put(RobotiqGraspMode.SCISSOR_MODE, FingerState.OPEN, Finger.FINGER_A, 0x00);
      fingerPositionMap.put(RobotiqGraspMode.SCISSOR_MODE, FingerState.OPEN, Finger.FINGER_B, 0x00);
      fingerPositionMap.put(RobotiqGraspMode.SCISSOR_MODE, FingerState.OPEN, Finger.FINGER_C, 0x00);
      fingerPositionMap.put(RobotiqGraspMode.SCISSOR_MODE, FingerState.OPEN, Finger.SCISSOR, 0x00);
      
      fingerPositionMap.put(RobotiqGraspMode.SCISSOR_MODE, FingerState.CLOSE, Finger.FINGER_A, 0x00);
      fingerPositionMap.put(RobotiqGraspMode.SCISSOR_MODE, FingerState.CLOSE, Finger.FINGER_B, 0x00);
      fingerPositionMap.put(RobotiqGraspMode.SCISSOR_MODE, FingerState.CLOSE, Finger.FINGER_C, 0x00);
      fingerPositionMap.put(RobotiqGraspMode.SCISSOR_MODE, FingerState.CLOSE, Finger.SCISSOR, 0xFF);
   }
   
   private final Finger finger;
   private final int index;

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
            throw new RuntimeException(getClass().getSimpleName() + ": " + finger.name() + " is not recognized as a Robotiq finger");
      }
   }
   
   public byte getFingerPosition(RobotiqGraspMode graspMode, FingerState fingerState)
   {
      return (byte)fingerPositionMap.get(graspMode, fingerState, finger);
   }

   @Override
   public byte getRegisterValue()
   {
      // TODO Auto-generated method stub
      return 0;
   }

   @Override
   public int getRegisterIndex()
   {
      return index;
   }
}
