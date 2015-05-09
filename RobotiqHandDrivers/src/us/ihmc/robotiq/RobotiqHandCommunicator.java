package us.ihmc.robotiq;

import java.net.SocketTimeoutException;

import net.wimpi.modbus.ModbusException;
import net.wimpi.modbus.procimg.InputRegister;
import net.wimpi.modbus.procimg.Register;
import us.ihmc.communication.configuration.NetworkParameterKeys;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.communication.packets.dataobjects.FingerState;
import us.ihmc.robotiq.communication.JamodTCPMaster;
import us.ihmc.robotiq.communication.RobotiqReadResponseFactory;
import us.ihmc.robotiq.communication.RobotiqWriteRequestFactory;
import us.ihmc.robotiq.communication.registers.GripperStatusRegister.gACT;
import us.ihmc.robotiq.data.RobotiqHandSensorDizzata;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.robotSide.RobotSide;

public class RobotiqHandCommunicator
{
   private final int PORT = 502;
   
   private boolean connected = false;

   private JamodTCPMaster communicator;
   
   private RobotiqWriteRequestFactory writeRequestFactory = new RobotiqWriteRequestFactory();
   private RobotiqReadResponseFactory readResponseFactory = new RobotiqReadResponseFactory();
   
   private final RobotiqHandSensorDizzata handSensorData = new RobotiqHandSensorDizzata();
   
   private RobotiqGraspMode graspMode = RobotiqGraspMode.BASIC_MODE;
   private FingerState fingerState = FingerState.OPEN;
   
   public RobotiqHandCommunicator(RobotSide robotSide)
   {
      NetworkParameterKeys key = robotSide.equals(RobotSide.LEFT) ? NetworkParameterKeys.leftHand : NetworkParameterKeys.rightHand;
      communicator = new JamodTCPMaster(NetworkParameters.getHost(key), PORT);
      communicator.setAutoReconnect(true);
      System.out.println(getClass().getSimpleName() + ": Successfully connected to Robotiq hand at " + NetworkParameters.getHost(key));
   }
   
   public void read()
   {
      try
      {
         InputRegister[] response = communicator.readInputRegisters(0, 8);
         readResponseFactory.updateRobotiqResponse(response);
         connected = true;
      }
      catch (Exception e)
      {
         connected = false;
      }
      
      handSensorData.update(readResponseFactory.getResponse(), connected);
   }
   
   public void reconnect()
   {
      communicator.reconnect();
   }
   
   public boolean isConnected()
   {
      return connected;
   }
   
   public void initialize()
   {
      Register[] request = writeRequestFactory.createActivationRequest();
      try
      {
         communicator.writeMultipleRegisters(0, request);
      }
      catch (ModbusException | SocketTimeoutException e)
      {
         System.err.println(getClass().getSimpleName() + ": Unable to initialize. Consider resetting hand.");
      }
   }
   
   public void reset()
   {
      Register[] request = writeRequestFactory.createDeactivationRequest();
      do
      {
         ThreadTools.sleep(100);
         try
         {
            read();
            communicator.writeMultipleRegisters(0, request);
         }
         catch (Exception e)
         {
            System.err.println(getClass().getSimpleName() + ": Unable to reset. Consider reconnecting to hand.");
         }
      }
      while(!readResponseFactory.getResponse().getGripperStatus().getGact().equals(gACT.GRIPPER_RESET));
      initialize();
   }
   
   public void sendFingerState(FingerState fingerState)
   {
      handleGraspModes(fingerState);
      sendCommand();
   }
   
   private void sendCommand()
   {
      Register[] request = writeRequestFactory.createFingerPositionRequest(graspMode, fingerState);
      try
      {
         communicator.writeMultipleRegisters(0, request);
      }
      catch (ModbusException | SocketTimeoutException e)
      {
         System.err.println(getClass().getSimpleName() + ": Failed to write " + graspMode.name() + " " + fingerState.name() + " command. Consider resetting hand.");
      }
   }
   
   private void handleGraspModes(FingerState fingerState)
   {
      switch(fingerState)
      {
         case BASIC_GRIP:
            graspMode = RobotiqGraspMode.BASIC_MODE;
            break;
         case PINCH_GRIP:
            graspMode = RobotiqGraspMode.PINCH_MODE;
            break;
         case WIDE_GRIP:
            graspMode = RobotiqGraspMode.WIDE_MODE;
            break;
         case SCISSOR_GRIP:
            graspMode = RobotiqGraspMode.SCISSOR_MODE;
            break;
         case CRUSH:
            fingerState = FingerState.CLOSE;
         default:
            this.fingerState = fingerState;
      }
   }
   
   public RobotiqHandSensorDizzata getHandSensorData()
   {
      return handSensorData;
   }
}
