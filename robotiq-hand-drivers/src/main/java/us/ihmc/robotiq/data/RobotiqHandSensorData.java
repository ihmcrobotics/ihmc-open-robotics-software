package us.ihmc.robotiq.data;

import static us.ihmc.robotiq.model.RobotiqHandModel.RobotiqHandJointNameMinimal.*;

import us.ihmc.avatar.handControl.packetsAndConsumers.HandSensorData;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotiq.communication.RobotiqReadResponse;
import us.ihmc.robotiq.communication.registers.FaultStatusRegister.gFLT;
import us.ihmc.robotiq.communication.registers.GripperStatusRegister.gACT;
import us.ihmc.robotiq.model.RobotiqHandModel.RobotiqHandJointNameMinimal;

public class RobotiqHandSensorData implements HandSensorData
{
   private final double[] jointAngles = new double[RobotiqHandJointNameMinimal.values.length];
   private final RobotiqReadResponse response = new RobotiqReadResponse();

   private boolean connected;

   public void update(RobotiqReadResponse response, boolean connected)
   {
      this.response.setAll(response);
      this.connected = connected;
   }
   
   public boolean hasError()
   {
      return !response.getFaultStatus().getGflt().equals(gFLT.NO_FAULT);
   }
   
   public gFLT getFaultStatus()
   {
      return response.getFaultStatus().getGflt();
   }
   
   @Override
   public double[] getFingerJointAngles(RobotSide robotSide)
   {
      jointAngles[FINGER_MIDDLE_JOINT_1.getIndex(robotSide)] = (response.getFingerAPosition().getRegisterValue() & 0xFF) * (25.0/72) * Math.PI / 0xFF; //62.5 degrees
      jointAngles[FINGER_MIDDLE_JOINT_2.getIndex(robotSide)] = (response.getFingerAPosition().getRegisterValue() & 0xFF) * (0.5) * Math.PI / 0xFF; //90 degrees
      jointAngles[FINGER_MIDDLE_JOINT_3.getIndex(robotSide)] = 0.0;
      
      jointAngles[PALM_FINGER_1_JOINT.getIndex(robotSide)] = -((double)(response.getScissorPosition().getRegisterValue() & 0xFF) * (8.0/45) / 0xFF - (4.0/45)) * Math.PI; //32 degrees
      jointAngles[FINGER_1_JOINT_1.getIndex(robotSide)]    =   (double)(response.getFingerBPosition().getRegisterValue() & 0xFF) * (25.0/72) * Math.PI / 0xFF; //62.5 degrees
      jointAngles[FINGER_1_JOINT_2.getIndex(robotSide)]    =   (double)(response.getFingerBPosition().getRegisterValue() & 0xFF) * (0.5) * Math.PI / 0xFF; //90 degrees
      jointAngles[FINGER_1_JOINT_3.getIndex(robotSide)]    = 0.0;
      
      jointAngles[PALM_FINGER_2_JOINT.getIndex(robotSide)] = ((double)(response.getScissorPosition().getRegisterValue() & 0xFF) * (8.0/45) / 0xFF - (4.0/45)) * Math.PI; //32 degrees
      jointAngles[FINGER_2_JOINT_1.getIndex(robotSide)]    =  (double)(response.getFingerCPosition().getRegisterValue() & 0xFF) * (25.0/72) * Math.PI / 0xFF; //62.5 degrees
      jointAngles[FINGER_2_JOINT_2.getIndex(robotSide)]    =  (double)(response.getFingerCPosition().getRegisterValue() & 0xFF) * (0.5) * Math.PI / 0xFF; //90 degrees
      jointAngles[FINGER_2_JOINT_3.getIndex(robotSide)]    = 0.0;
      
      return jointAngles;
   }

   @Override
   public boolean isCalibrated()
   {
      return response.getGripperStatus().getGact().equals(gACT.GRIPPER_ACTIVATION);
   }

   @Override
   public boolean isConnected()
   {
      return connected;
   }

}
