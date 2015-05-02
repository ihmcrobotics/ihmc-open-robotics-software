package us.ihmc.robotiq;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import net.wimpi.modbus.ModbusException;
import net.wimpi.modbus.facade.ModbusTCPMaster;
import net.wimpi.modbus.procimg.InputRegister;
import net.wimpi.modbus.procimg.Register;
import us.ihmc.communication.configuration.NetworkParameterKeys;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.communication.packets.dataobjects.FingerState;
import us.ihmc.robotiq.communication.RobotiqWriteRequestFactory;
import us.ihmc.robotiq.data.RobotiqHandSensorData;
import us.ihmc.utilities.robotSide.RobotSide;

public class RobotiqHandCommunicator
{
   private final int PORT = 502;

   private ModbusTCPMaster communicator;
   
   private RobotiqWriteRequestFactory writeRequestFactory = new RobotiqWriteRequestFactory();
   private InputRegister[] response = new InputRegister[8];
   
   private RobotiqGraspMode graspMode = RobotiqGraspMode.BASIC_MODE;
   
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
      response = communicator.readInputRegisters(0, 8);
      
   }
   
   public void initialize() throws ModbusException
   {
      Register[] request = writeRequestFactory.createActivationRequest();
      communicator.writeMultipleRegisters(0, request);
   }
   
   public void reset() throws ModbusException
   {
      Register[] request = writeRequestFactory.createDeactivationRequest();
      communicator.writeMultipleRegisters(0, request);
   }
   
   public void open() throws ModbusException
   {
      Register[] request = writeRequestFactory.createFingerPositionRequest(graspMode, FingerState.OPEN);
      communicator.writeMultipleRegisters(0, request);
   }
   
   public void close() throws ModbusException
   {
      Register[] request = writeRequestFactory.createFingerPositionRequest(graspMode, FingerState.CLOSE);
      communicator.writeMultipleRegisters(0, request);
   }
   
   public void crush()
   {
      
   }
   
   public void basicGrip()
   {
      
   }
   
   public void pinchGrip()
   {
      
   }
   
   public void wideGrip()
   {
      
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
//         hand.reset();
         hand.initialize();
         
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
            catch (ModbusException e)
            {
               e.printStackTrace();
            }
         }
      }, 0, 10, TimeUnit.MILLISECONDS);
      
      
      executor.schedule(new Runnable()
      {
         @Override
         public void run()
         {
            System.out.println("Closing...");
            try
            {
               hand.close();
            }
            catch (ModbusException e)
            {
               e.printStackTrace();
            }
         }
      }, 20, TimeUnit.SECONDS);
      
      executor.schedule(new Runnable()
      {
         @Override
         public void run()
         {
            System.out.println("Opening...");
            try
            {
               hand.open();
            }
            catch (ModbusException e)
            {
               e.printStackTrace();
            }
         }
      }, 25, TimeUnit.SECONDS);
   }
}
