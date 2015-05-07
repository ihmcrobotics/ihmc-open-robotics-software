package us.ihmc.robotiq.control;

import java.io.IOException;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import net.wimpi.modbus.ModbusException;
import us.ihmc.commonWalkingControlModules.packetConsumers.FingerStateProvider;
import us.ihmc.communication.packets.dataobjects.FingerState;
import us.ihmc.communication.packets.manipulation.FingerStatePacket;
import us.ihmc.communication.packets.manipulation.ManualHandControlPacket;
import us.ihmc.darpaRoboticsChallenge.handControl.HandControlThread;
import us.ihmc.darpaRoboticsChallenge.handControl.packetsAndConsumers.HandJointAngleCommunicator;
import us.ihmc.darpaRoboticsChallenge.handControl.packetsAndConsumers.ManualHandControlProvider;
import us.ihmc.robotiq.RobotiqHandCommunicator;
import us.ihmc.utilities.robotSide.RobotSide;

import com.martiansoftware.jsap.FlaggedOption;
import com.martiansoftware.jsap.JSAP;
import com.martiansoftware.jsap.JSAPException;
import com.martiansoftware.jsap.JSAPResult;

public class RobotiqControlThrizzead extends HandControlThread
{
   private final boolean CALIBRATE_ON_CONNECT = false;
   
   private final RobotSide robotSide;
   private final RobotiqHandCommunicator handCommunicator;
   
   private final FingerStateProvider fingerStateProvider;
   private final ManualHandControlProvider manualHandControlProvider;
   private final HandJointAngleCommunicator jointAngleCommunicator;
   
   public RobotiqControlThrizzead(RobotSide robotSide)
   {
      super(robotSide);
      
      this.robotSide = robotSide;
      handCommunicator = new RobotiqHandCommunicator(robotSide);
      fingerStateProvider = new FingerStateProvider(robotSide);
      manualHandControlProvider = new ManualHandControlProvider(robotSide);
      jointAngleCommunicator = new HandJointAngleCommunicator(robotSide, packetCommunicator);
      
      packetCommunicator.attachListener(FingerStatePacket.class, fingerStateProvider);
      packetCommunicator.attachListener(ManualHandControlPacket.class, manualHandControlProvider);
   }
   
   @Override
   public void connect()
   {
      try
      {
         packetCommunicator.connect();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }

   @Override
   public void run()
   {
      System.out.println("DEBUG0");
      if(handCommunicator.isConnected())
      {
         System.out.println("Hand is connected");
         if(fingerStateProvider.isNewFingerStateAvailable())
         {
            System.out.println("New FingerState available");
            FingerState fingerState = fingerStateProvider.pullPacket().getFingerState();
            
            try
            {
               switch(fingerState)
               {
               case BASIC_GRIP:
                  handCommunicator.basicGrip();
                  break;
               case CALIBRATE:
                  handCommunicator.initialize();
                  break;
               case CLOSE:
                  handCommunicator.close();
                  break;
               case CLOSE_FINGERS:
                  break;
               case CLOSE_THUMB:
                  break;
               case CONNECT:
                  break;
               case CRUSH:
                  handCommunicator.crush();
                  break;
               case HALF_CLOSE:
                  break;
               case HOOK:
                  break;
               case OPEN:
                  handCommunicator.open();
                  break;
               case OPEN_FINGERS:
                  break;
               case OPEN_THUMB:
                  break;
               case PINCH_GRIP:
                  handCommunicator.pinchGrip();
                  break;
               case RESET:
                  handCommunicator.reset();
                  break;
               case SCISSOR_GRIP:
                  break;
               case STOP:
                  break;
               case WIDE_GRIP:
                  handCommunicator.wideGrip();
                  break;
               }
            }
            catch(ModbusException e)
            {
               e.printStackTrace();
            }
         }
         
         if(manualHandControlProvider.isNewPacketAvailable())
         {
            //TODO
         }
      }
      else
      {
//         handCommunicator.reconnect();
      }
   }

   public static void main(String[] args)
   {
      JSAP jsap = new JSAP();
      
      FlaggedOption robotSide = new FlaggedOption("robotSide").setRequired(true).setLongFlag("robotSide").setShortFlag('r').setStringParser(JSAP.STRING_PARSER);
      
      try
      {
         jsap.registerParameter(robotSide);
         JSAPResult config = jsap.parse(args);
         
         if(config.success())
         {
            RobotiqControlThrizzead controlThread = new RobotiqControlThrizzead(RobotSide.valueOf(config.getString("robotSide").toUpperCase()));
            controlThread.connect();
            ScheduledExecutorService executor = Executors.newSingleThreadScheduledExecutor();
            executor.scheduleWithFixedDelay(controlThread, 0, 50, TimeUnit.MILLISECONDS);
         }
      }
      catch (JSAPException e)
      {
         e.printStackTrace();
      }
   }
}
