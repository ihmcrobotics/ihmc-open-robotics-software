package us.ihmc.robotiq;

import java.net.InetAddress;
import java.net.UnknownHostException;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import net.wimpi.modbus.msg.ModbusResponse;
import us.ihmc.communication.configuration.NetworkParameterKeys;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.robotiq.communication.JamodTCPConnection;
import us.ihmc.robotiq.communication.RobotiqReadRequestFactory;
import us.ihmc.robotiq.communication.RobotiqWriteRequestFactory;
import us.ihmc.robotiq.data.RobotiqHandSensorData;
import us.ihmc.utilities.robotSide.RobotSide;

public class RobotiqHandCommunicator
{
   private final int PORT = 502;

   private JamodTCPConnection communicator;
   
   private RobotiqWriteRequestFactory writeRequestFactory = new RobotiqWriteRequestFactory();
   private RobotiqReadRequestFactory readRequestFactory = new RobotiqReadRequestFactory();
   private ModbusResponse response;
   
   public RobotiqHandCommunicator(RobotSide robotSide)
   {
      NetworkParameterKeys key = robotSide.equals(RobotSide.LEFT) ? NetworkParameterKeys.leftHand : NetworkParameterKeys.rightHand;
      try
      {
         communicator = new JamodTCPConnection(InetAddress.getByName(NetworkParameters.getHost(key)), PORT);
         communicator.connect();
      }
      catch (UnknownHostException e)
      {
         e.printStackTrace();
      }
   }
   
   public boolean isConnected()
   {
      return communicator.isConnected();
   }
   
   public void read()
   {
      response = communicator.sendRequest(readRequestFactory.getReadRequest());
   }
   
   public void initialize()
   {
      response = communicator.sendRequest(writeRequestFactory.createActivationRequest());
   }
   
   public void open()
   {
      
   }
   
   public void close()
   {
      
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
      //TODO
      return null;
   }
   
   public static void main(String[] args)
   {
      RobotSide robotSide = RobotSide.RIGHT;
      
      final RobotiqHandCommunicator hand = new RobotiqHandCommunicator(robotSide);
      
      hand.initialize();
      
      ScheduledExecutorService executor = Executors.newSingleThreadScheduledExecutor();
      executor.scheduleAtFixedRate(new Runnable()
      {
         @Override
         public void run()
         {
            hand.read();
         }
      }, 0, 10, TimeUnit.MILLISECONDS);
   }
}
