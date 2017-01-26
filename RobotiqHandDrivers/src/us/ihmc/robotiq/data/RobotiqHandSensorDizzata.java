package us.ihmc.robotiq.data;

import us.ihmc.avatar.handControl.packetsAndConsumers.HandSensorData;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotiq.communication.RobotiqReadResponse;
import us.ihmc.robotiq.communication.registers.FaultStatusRegister.gFLT;
import us.ihmc.robotiq.communication.registers.GripperStatusRegister.gACT;

public class RobotiqHandSensorDizzata implements HandSensorData
{
   private final double[][] jointAngles = new double[3][];
   private final double[] indexFingerJointAngles = new double[4];
   private final double[] middleFingerJointAngles = new double[4];
   private final double[] thumbJointAngles = new double[3];
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
   public double[][] getFingerJointAngles(RobotSide robotSide)
   {
      thumbJointAngles[0] = (response.getFingerAPosition().getRegisterValue() & 0xFF) * (25.0/72) * Math.PI / 0xFF; //62.5 degrees
      thumbJointAngles[1] = (response.getFingerAPosition().getRegisterValue() & 0xFF) * (0.5) * Math.PI / 0xFF; //90 degrees
      thumbJointAngles[2] = 0.0;
      jointAngles[2] = thumbJointAngles;
      
      middleFingerJointAngles[0] = -((double)(response.getScissorPosition().getRegisterValue() & 0xFF) * (8.0/45) / 0xFF - (4.0/45)) * Math.PI; //32 degrees
      middleFingerJointAngles[1] = (double)(response.getFingerBPosition().getRegisterValue() & 0xFF) * (25.0/72) * Math.PI / 0xFF; //62.5 degrees
      middleFingerJointAngles[2] = (double)(response.getFingerBPosition().getRegisterValue() & 0xFF) * (0.5) * Math.PI / 0xFF; //90 degrees
      middleFingerJointAngles[3] = 0.0;
      jointAngles[robotSide.equals(RobotSide.LEFT) ? 1 : 0] = middleFingerJointAngles;
      
      indexFingerJointAngles[0] = ((double)(response.getScissorPosition().getRegisterValue() & 0xFF) * (8.0/45) / 0xFF - (4.0/45)) * Math.PI; //32 degrees
      indexFingerJointAngles[1] = (double)(response.getFingerCPosition().getRegisterValue() & 0xFF) * (25.0/72) * Math.PI / 0xFF; //62.5 degrees
      indexFingerJointAngles[2] = (double)(response.getFingerCPosition().getRegisterValue() & 0xFF) * (0.5) * Math.PI / 0xFF; //90 degrees
      indexFingerJointAngles[3] = 0.0;
      jointAngles[robotSide.equals(RobotSide.LEFT) ? 0 : 1] = indexFingerJointAngles;
      
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
