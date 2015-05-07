package us.ihmc.robotiq;

import net.wimpi.modbus.ModbusException;
import net.wimpi.modbus.facade.ModbusTCPMaster;
import net.wimpi.modbus.procimg.InputRegister;
import net.wimpi.modbus.procimg.Register;
import us.ihmc.communication.configuration.NetworkParameterKeys;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.communication.packets.dataobjects.FingerState;
import us.ihmc.robotiq.communication.RobotiqReadResponseFactory;
import us.ihmc.robotiq.communication.RobotiqWriteRequestFactory;
import us.ihmc.robotiq.communication.registers.GripperStatusRegister.gACT;
import us.ihmc.robotiq.data.RobotiqHandSensorData;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.robotSide.RobotSide;

public class RobotiqHandCommunicator
{
   private final int PORT = 502;
   private final int CONNECT_ATTEMPTS = 10;
   private final boolean AUTORECONNECT = true;

   private ModbusTCPMaster communicator;
   
   private RobotiqWriteRequestFactory writeRequestFactory = new RobotiqWriteRequestFactory();
   private RobotiqReadResponseFactory readResponseFactory = new RobotiqReadResponseFactory();
   
   private final RobotiqHandSensorData handSensorData = new RobotiqHandSensorData();
   
   private RobotiqGraspMode graspMode = RobotiqGraspMode.BASIC_MODE;
   private FingerState fingerState = FingerState.OPEN;
   
   public RobotiqHandCommunicator(RobotSide robotSide)
   {
      NetworkParameterKeys key = robotSide.equals(RobotSide.LEFT) ? NetworkParameterKeys.leftHand : NetworkParameterKeys.rightHand;
      communicator = new ModbusTCPMaster(NetworkParameters.getHost(key), PORT);
      
      int count = 0;
      try
      {
         communicator.connect();
      }
      catch (Exception e)
      {
         if(count < CONNECT_ATTEMPTS)
         {
            count++;
            System.out.println(getClass().getSimpleName() + ": Unable to connect to hand. Retrying...");
         }
         else
         {
            System.out.println(getClass().getSimpleName() + ": Initial connect unsuccessful.");
         }
      }
      
      if(AUTORECONNECT && isConnected())
      {
         new Thread(new ConnectionListener(robotSide)).start();
      }
   }
   
   public void read() throws Exception
   {
      InputRegister[] response = communicator.readInputRegisters(0, 8);
      readResponseFactory.updateRobotiqResponse(response);
      handSensorData.update(readResponseFactory.getResponse(), isConnected());
   }
   
   public void reconnect()
   {
      communicator.disconnect();
      ThreadTools.sleep(50);
      try
      {
         communicator.connect();
      }
      catch (Exception e)
      {
         System.out.println(getClass().getSimpleName() + ": Unable to reconnect. Check power and network connections.");
      }
   }
   
   public boolean isConnected()
   {
      try
      {
         read();
      }
      catch (Exception e)
      {
         return false;
      }
      
      return true;
   }
   
   public void initialize()
   {
      Register[] request = writeRequestFactory.createActivationRequest();
      try
      {
         communicator.writeMultipleRegisters(0, request);
      }
      catch (ModbusException e)
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
   
   public void open()
   {
      fingerState = FingerState.OPEN;
      sendCommand();
   }
   
   public void close()
   {
      fingerState = FingerState.CLOSE;
      sendCommand();
   }
   
   private void sendCommand()
   {
      Register[] request = writeRequestFactory.createFingerPositionRequest(graspMode, fingerState);
      try
      {
         communicator.writeMultipleRegisters(0, request);
      }
      catch (ModbusException e)
      {
         System.err.println(getClass().getSimpleName() + ": Failed to write " + graspMode.name() + " " + fingerState.name() + " command. Consider resetting hand.");
      }
   }
   
   public void crush()
   {
      close();
   }
   
   public void basicGrip()
   {
      graspMode = RobotiqGraspMode.BASIC_MODE;
      sendCommand();
   }
   
   public void pinchGrip()
   {
      graspMode = RobotiqGraspMode.PINCH_MODE;
      sendCommand();
   }
   
   public void wideGrip()
   {
      graspMode = RobotiqGraspMode.WIDE_MODE;
      sendCommand();
   }
   
   public RobotiqHandSensorData getHandSensorData()
   {
      return handSensorData;
   }
   
   class ConnectionListener implements Runnable
   {
      private final RobotSide robotSide;
      
      public ConnectionListener(RobotSide robotSide)
      {
         this.robotSide = robotSide;
      }
      
      public void run()
      {
         if(!isConnected())
         {
            System.out.println("RobotiqHandCommunicator: " + robotSide.getCamelCaseNameForStartOfExpression() + " hand disconnected. Attempting to reconnect...");
            reconnect();
            ThreadTools.sleep(200);
         }
      }
   }
}
