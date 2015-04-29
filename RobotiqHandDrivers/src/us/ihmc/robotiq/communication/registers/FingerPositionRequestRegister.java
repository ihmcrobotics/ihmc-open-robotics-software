package us.ihmc.robotiq.communication.registers;

import org.apache.commons.collections.map.MultiKeyMap;

import us.ihmc.communication.packets.dataobjects.FingerState;
import us.ihmc.robotiq.RobotiqGraspMode;

public abstract class FingerPositionRequestRegister implements RobotiqRegister
{
   enum Finger
   {
      FINGER_A, FINGER_B, FINGER_C, SCISSOR;
   }
   
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

   public FingerPositionRequestRegister(Finger finger)
   {
      this.finger = finger;
      
   }
   
   public byte getFingerPosition(RobotiqGraspMode graspMode, FingerState fingerState)
   {
      return (byte)fingerPositionMap.get(graspMode, fingerState, finger);
   }
}
