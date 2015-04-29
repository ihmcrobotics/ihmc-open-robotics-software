package us.ihmc.robotiq.communication;

import net.wimpi.modbus.msg.WriteMultipleRegistersRequest;
import us.ihmc.robotiq.communication.registers.ActionRequestRegister;
import us.ihmc.robotiq.communication.registers.FingerForceRegister;
import us.ihmc.robotiq.communication.registers.FingerPositionRequestRegister;
import us.ihmc.robotiq.communication.registers.FingerSpeedRegister;
import us.ihmc.robotiq.communication.registers.GripperOptionRegister;

public class RobotiqWriteRequest
{
   private ActionRequestRegister actionRequest;
   private GripperOptionRegister gripperOption;
   private FingerPositionRequestRegister fingerAPostionRequest;
   private FingerSpeedRegister fingerASpeed;
   private FingerForceRegister fingerAForce;
   private FingerPositionRequestRegister fingerBPostionRequest;
   private FingerSpeedRegister fingerBSpeed;
   private FingerForceRegister fingerBForce;
   private FingerPositionRequestRegister fingerCPostionRequest;
   private FingerSpeedRegister fingerCSpeed;
   private FingerForceRegister fingerCForce;
   
   private WriteMultipleRegistersRequest request;
   
   public RobotiqWriteRequest()
   {
      actionRequest = new ActionRequestRegister();
      gripperOption = new GripperOptionRegister();
   }

   public void setActionRequest(ActionRequestRegister actionRequest)
   {
      this.actionRequest = actionRequest;
   }
   
   public WriteMultipleRegistersRequest convertToJamodRequest()
   {
      return request;
   }
}
