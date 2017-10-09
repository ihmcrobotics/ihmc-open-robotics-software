package us.ihmc.robotiq;

import java.net.SocketTimeoutException;

import net.wimpi.modbus.ModbusException;
import net.wimpi.modbus.procimg.InputRegister;
import net.wimpi.modbus.procimg.Register;
import us.ihmc.communication.configuration.NetworkParameterKeys;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotiq.communication.JamodTCPMaster;
import us.ihmc.robotiq.communication.RobotiqReadResponseFactory;
import us.ihmc.robotiq.communication.RobotiqWriteRequestFactory;
import us.ihmc.robotiq.communication.registers.GripperStatusRegister.gACT;
import us.ihmc.robotiq.data.RobotiqHandSensorData;
import us.ihmc.tools.thread.ThreadTools;

public class RobotiqHandCommunicator
{
   private final boolean DEBUG = false;
   
   private final int PORT = 502;
   
   private JamodTCPMaster communicator;
   
   private RobotiqWriteRequestFactory writeRequestFactory = new RobotiqWriteRequestFactory();
   private RobotiqReadResponseFactory readResponseFactory = new RobotiqReadResponseFactory();
   
   private final RobotiqHandSensorData handSensorData = new RobotiqHandSensorData();
   
   private RobotiqGraspMode graspMode = RobotiqGraspMode.BASIC_MODE;
   private HandConfiguration handConfiguration = HandConfiguration.OPEN;
   
   private boolean connected = false;
   
   public RobotiqHandCommunicator(RobotSide robotSide)
   {
      NetworkParameterKeys key = robotSide.equals(RobotSide.LEFT) ? NetworkParameterKeys.leftHand : NetworkParameterKeys.rightHand;
      communicator = new JamodTCPMaster(NetworkParameters.getHost(key), PORT);
      communicator.setAutoReconnect(true);
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
      
      handSensorData.update(readResponseFactory.getResponse(), isConnected());
   }
   
   public void reconnect()
   {
      communicator.reconnect();
   }
   
   public boolean isConnected()
   {
      return communicator.isConnected() && connected;
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
      try
      {
         do
         {
            ThreadTools.sleep(100);
            read();
            communicator.writeMultipleRegisters(0, request);
         }
         while(!readResponseFactory.getResponse().getGripperStatus().getGact().equals(gACT.GRIPPER_RESET));
         initialize();
      }
      catch (Exception e)
      {
         System.err.println(getClass().getSimpleName() + ": Unable to reset. Consider reconnecting to hand.");
      }
   }
   
   public void sendHandCommand(HandConfiguration handConfiguration)
   {
      handleGraspModes(handConfiguration);
      sendCommand(writeRequestFactory.createWholeHandPositionRequest(graspMode, this.handConfiguration));
   }
   
   public void sendFingersCommand(HandConfiguration handConfiguration)
   {
      handleGraspModes(handConfiguration);
      sendCommand(writeRequestFactory.createFingersPositionRequest(graspMode, this.handConfiguration));
   }
   
   public void sendThumbCommand(HandConfiguration handConfiguration)
   {
      handleGraspModes(handConfiguration);
      sendCommand(writeRequestFactory.createThumbPositionRequest(graspMode, this.handConfiguration));
   }
   
   private void sendCommand(Register[] request)
   {
      if(DEBUG)
      {
         System.out.println("Write:");
         printRegisters(request);
      }
      
      try
      {
         communicator.writeMultipleRegisters(0, request);
      }
      catch (ModbusException | SocketTimeoutException e)
      {
         System.err.println(getClass().getSimpleName() + ": Failed to write " + graspMode.name() + " " + handConfiguration.name() + " command. Consider resetting hand.");
      }
   }
   
   private void handleGraspModes(HandConfiguration handConfiguration)
   {
      switch(handConfiguration)
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
         case CLOSE_FINGERS:
         case CLOSE_THUMB:
         case CRUSH:
            this.handConfiguration = HandConfiguration.CLOSE;
            break;
         case OPEN_FINGERS:
         case OPEN_THUMB:
            this.handConfiguration = HandConfiguration.OPEN;
            break;
         default:
            this.handConfiguration = handConfiguration;
      }
   }
   
   public RobotiqHandSensorData getHandSensorData()
   {
      return handSensorData;
   }
   
   private void printRegisters(InputRegister[] registers)
   {
      for(InputRegister reg : registers)
      {
         for(byte b : reg.toBytes())
         {
            System.out.print(b + " ");
         }
      }
      System.out.println();
   }
}
