package us.ihmc.robotiq;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import net.wimpi.modbus.ModbusException;
import net.wimpi.modbus.facade.ModbusTCPMaster;
import net.wimpi.modbus.procimg.InputRegister;
import net.wimpi.modbus.procimg.Register;
import net.wimpi.modbus.procimg.SimpleRegister;
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

   private ModbusTCPMaster communicator;
   
   private RobotiqWriteRequestFactory writeRequestFactory = new RobotiqWriteRequestFactory();
   private RobotiqReadResponseFactory readResponseFactory = new RobotiqReadResponseFactory();
   
   private RobotiqGraspMode graspMode = RobotiqGraspMode.BASIC_MODE;
   private FingerState fingerState = FingerState.OPEN;
   
   public RobotiqHandCommunicator(RobotSide robotSide)
   {
      NetworkParameterKeys key = robotSide.equals(RobotSide.LEFT) ? NetworkParameterKeys.leftHand : NetworkParameterKeys.rightHand;
      try
      {
         communicator = new ModbusTCPMaster(NetworkParameters.getHost(key), PORT);
         communicator.connect();
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }
   }
   
   public void read() throws ModbusException
   {
      InputRegister[] response = communicator.readInputRegisters(0, 8);
      for(InputRegister register : response)
      {
         for(byte b : register.toBytes())
            System.out.print(b + " ");
      }
      System.out.println();
      readResponseFactory.updateRobotiqResponse(response);
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
         e.printStackTrace();
      }
   }
   
   public boolean isConnected()
   {
      try
      {
         read();
      }
      catch (ModbusException e)
      {
         return false;
      }
      
      return true;
   }
   
   public void initialize() throws ModbusException
   {
      Register[] request = writeRequestFactory.createActivationRequest();
      communicator.writeMultipleRegisters(0, request);
   }
   
   public void reset() throws ModbusException
   {
      Register[] request = writeRequestFactory.createDeactivationRequest();
//      Register[] request = new SimpleRegister[]{new SimpleRegister((byte)0, (byte)0),
//                                                new SimpleRegister((byte)0, (byte)0),
//                                                new SimpleRegister((byte)0, (byte)0),
//                                                new SimpleRegister((byte)0, (byte)0),
//                                                new SimpleRegister((byte)0, (byte)0),
//                                                new SimpleRegister((byte)0, (byte)0),
//                                                new SimpleRegister((byte)0, (byte)0),
//                                                new SimpleRegister((byte)0, (byte)0)};
      System.out.println("Before reset:");
      read();
      do
      {
         System.out.println("Writing:");
         for(Register register : request)
         {
            for(byte b : register.toBytes())
               System.out.print(b + " ");
         }
         System.out.println();
         communicator.writeMultipleRegisters(0, request);
         read();
      }
      while(!readResponseFactory.getResponse().getGripperStatus().getGact().equals(gACT.GRIPPER_RESET));
      initialize();
   }
   
   public void open() throws ModbusException
   {
      fingerState = FingerState.OPEN;
      sendCommand();
   }
   
   public void close() throws ModbusException
   {
      fingerState = FingerState.CLOSE;
      sendCommand();
   }
   
   private void sendCommand() throws ModbusException
   {
      Register[] request = writeRequestFactory.createFingerPositionRequest(graspMode, fingerState);
      communicator.writeMultipleRegisters(0, request);
   }
   
   public void crush() throws ModbusException
   {
      close();
   }
   
   public void basicGrip() throws ModbusException
   {
      graspMode = RobotiqGraspMode.BASIC_MODE;
      sendCommand();
   }
   
   public void pinchGrip() throws ModbusException
   {
      graspMode = RobotiqGraspMode.PINCH_MODE;
      sendCommand();
   }
   
   public void wideGrip() throws ModbusException
   {
      graspMode = RobotiqGraspMode.WIDE_MODE;
      sendCommand();
   }
   
   public RobotiqHandSensorData updateHandStatus()
   {
      // TODO
      // take stuff from ModbusResponse and put into something nice
      // so other stuff doesn't need to see Modbus stuff
      
      return null;
   }
   
   public static void main(String[] args)
   {
      RobotSide robotSide = RobotSide.LEFT;
      
      final RobotiqHandCommunicator hand = new RobotiqHandCommunicator(robotSide);
      
      try
      {
         hand.reset();
//         hand.initialize();
         
      }
      catch (ModbusException e)
      {
         e.printStackTrace();
      }

      ScheduledExecutorService executor = Executors.newSingleThreadScheduledExecutor();
      executor.scheduleAtFixedRate(new Runnable()
      {
         @Override
         public void run()
         {
            try
            {
               hand.read();
            }
            catch (Exception e)
            {
               e.printStackTrace();
            }
         }
      }, 0, 10, TimeUnit.MILLISECONDS);
      
      
//      executor.schedule(new Runnable()
//      {
//         @Override
//         public void run()
//         {
//            System.out.println("Closing...");
//            try
//            {
//               hand.close();
//            }
//            catch (ModbusException e)
//            {
//               e.printStackTrace();
//            }
//         }
//      }, 20, TimeUnit.SECONDS);
//      
//      executor.schedule(new Runnable()
//      {
//         @Override
//         public void run()
//         {
//            System.out.println("Opening...");
//            try
//            {
//               hand.open();
//            }
//            catch (ModbusException e)
//            {
//               e.printStackTrace();
//            }
//         }
//      }, 25, TimeUnit.SECONDS);
   }
}
