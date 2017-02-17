package us.ihmc.acsell.hardware;

import java.io.IOException;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.net.NetworkInterface;
import java.net.StandardProtocolFamily;
import java.net.StandardSocketOptions;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.channels.DatagramChannel;

import us.ihmc.acsell.hardware.configuration.AcsellNetworkParameters;
import us.ihmc.communication.configuration.NetworkParameterKeys;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.multicastLogDataProtocol.LogUtils;
import us.ihmc.realtime.MonotonicTime;
import us.ihmc.realtime.PeriodicParameters;
import us.ihmc.realtime.PriorityParameters;
import us.ihmc.realtime.RealtimeThread;
import us.ihmc.robotModels.visualizer.RobotVisualizer;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.tools.thread.ThreadTools;

public class AcsellSetup extends RealtimeThread
{
   private static final long POWER_BUS_ON_DELAY = 1000000000;
   private static final long POWER_BUS_OFF_DELAY = 100000000;


   private enum PowerState
   {
      IDLE, DELAYED_RIGHT_LEG_OFF, DELAYED_BACK_OFF, DELAYED_RIGHT_LEG_ON, DELAYED_BACK_ON
   }

   private static final PriorityParameters priority = new PriorityParameters(50);
   private static final MonotonicTime interval = new MonotonicTime(0, 100000000);

   private final YoVariableRegistry registry = new YoVariableRegistry("StepprSetup");

   private final IntegerYoVariable logicPowerStateRequest = new IntegerYoVariable("logicPowerStateRequest", registry);
   private final IntegerYoVariable motorPowerStateRequest = new IntegerYoVariable("motorPowerStateRequest", registry);

   private long logicSwitchTime = -1;
   private long motorSwitchTime = -1;

   private PowerState logicPowerState = PowerState.IDLE;
   private PowerState motorPowerState = PowerState.IDLE;

   private DatagramChannel channel;
   private ByteBuffer sendBuffer;
   private InetSocketAddress powerCommandAddress;

   private final RobotVisualizer visualizer;

   public AcsellSetup(RobotVisualizer vizualizer)
   {
      super(priority, new PeriodicParameters(interval));
      this.visualizer = vizualizer;

      if (vizualizer != null)
      {
         vizualizer.addRegistry(registry, null);
      }
   }

   @Override
   public void run()
   {
      try
      {
         NetworkInterface iface = LogUtils.getMyInterface(NetworkParameters.getHost(NetworkParameterKeys.robotController));
         System.out.println("Binding setup to interface: " + iface);

         InetAddress group = InetAddress.getByName(AcsellNetworkParameters.ACSELL_MULTICAST_GROUP);
         powerCommandAddress = new InetSocketAddress(group, AcsellNetworkParameters.UDP_MULTICAST_POWER_BUS_PORT);

         InetSocketAddress receiveAddress = new InetSocketAddress(AcsellNetworkParameters.UDP_MULTICAST_PARAMETER_REPLY_PORT);
         channel = DatagramChannel.open(StandardProtocolFamily.INET).setOption(StandardSocketOptions.SO_REUSEADDR, true)
               .setOption(StandardSocketOptions.IP_MULTICAST_IF, iface).bind(receiveAddress);
         channel.configureBlocking(false);
         channel.socket().setReceiveBufferSize(65535);
         channel.join(group, iface);

         ByteBuffer receiveBuffer = ByteBuffer.allocate(65535);
         receiveBuffer.order(ByteOrder.LITTLE_ENDIAN);

         sendBuffer = ByteBuffer.allocate(65535);
         sendBuffer.order(ByteOrder.LITTLE_ENDIAN);

         while (true)
         {
            super.waitForNextPeriod();
            handlePower();
            visualizer.update(getCurrentMonotonicClockTime(), registry);
         }
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }



   private void sendPowerState(int channel, int state) throws IOException
   {
      sendBuffer.clear();
      sendBuffer.put((byte) 0x01);
      sendBuffer.put((byte) 0x01);
      sendBuffer.put((byte) 0x08);
      sendBuffer.put((byte) 0x07);
      sendBuffer.put((byte) 0x42);
      sendBuffer.put((byte) 0x03);

      byte id = (byte) 0x01;
      byte channelB = (byte) channel;
      byte stateB = (byte) state;

      short checksum = (short) (id & 0xFFFF);
      checksum = (short) ((checksum + (short) (channelB & 0xFFFF)) & 0xFFFF);
      checksum = (short) ((checksum + (short) (stateB & 0xFFFF)) & 0xFFFF);

      sendBuffer.put(id);
      sendBuffer.put(channelB);
      sendBuffer.put(stateB);

      // Screw you Morgan, curveball big-endian
      sendBuffer.put((byte) (checksum >>> 8));
      sendBuffer.put((byte) (checksum & 0xFF));
      sendBuffer.flip();
      this.channel.send(sendBuffer, powerCommandAddress);
   }

   private boolean motorSwitchOnTime()
   {
      return getCurrentMonotonicClockTime() - motorSwitchTime > POWER_BUS_ON_DELAY;
   }

   private boolean motorSwitchOffTime()
   {
      return getCurrentMonotonicClockTime() - motorSwitchTime > POWER_BUS_OFF_DELAY;
   }

   private boolean logicSwitchOnTime()
   {
      return getCurrentMonotonicClockTime() - logicSwitchTime > POWER_BUS_ON_DELAY;
   }

   private boolean logicSwitchOffTime()
   {
      return getCurrentMonotonicClockTime() - logicSwitchTime > POWER_BUS_OFF_DELAY;
   }

   private void handlePower() throws IOException
   {
      switch (motorPowerState)
      {
      case IDLE:
         if (motorPowerStateRequest.getIntegerValue() == 1)
         {
            motorPowerState = PowerState.DELAYED_RIGHT_LEG_ON;
            sendPowerState(0, 2);
            motorSwitchTime = getCurrentMonotonicClockTime();
         }
         else if (motorPowerStateRequest.getIntegerValue() == -1)
         {
            sendPowerState(0, 0);
            motorSwitchTime = getCurrentMonotonicClockTime();
            motorPowerState = PowerState.DELAYED_RIGHT_LEG_OFF;
         }

         break;
      case DELAYED_RIGHT_LEG_ON:
         if (motorSwitchOnTime())
         {
            sendPowerState(1, 2);
            motorSwitchTime = getCurrentMonotonicClockTime();
            motorPowerState = PowerState.DELAYED_BACK_ON;
         }
         break;
      case DELAYED_BACK_ON:
         if (motorSwitchOnTime())
         {
            sendPowerState(2, 2);
            motorSwitchTime = getCurrentMonotonicClockTime();
            motorPowerState = PowerState.IDLE;
         }
         break;
      case DELAYED_RIGHT_LEG_OFF:
         if (motorSwitchOffTime())
         {
            sendPowerState(1, 0);
            motorSwitchTime = getCurrentMonotonicClockTime();
            motorPowerState = PowerState.DELAYED_BACK_OFF;
         }
         break;
      case DELAYED_BACK_OFF:
         if (motorSwitchOffTime())
         {
            sendPowerState(2, 0);
            motorSwitchTime = getCurrentMonotonicClockTime();
            motorPowerState = PowerState.IDLE;
         }
         break;
      }

      switch (logicPowerState)
      {
      case IDLE:
         if (logicPowerStateRequest.getIntegerValue() == 1)
         {
            logicPowerState = PowerState.DELAYED_RIGHT_LEG_ON;
            sendPowerState(6, 2);
            logicSwitchTime = getCurrentMonotonicClockTime();
         }
         else if (logicPowerStateRequest.getIntegerValue() == -1)
         {
            sendPowerState(6, 0);
            logicSwitchTime = getCurrentMonotonicClockTime();
            logicPowerState = PowerState.DELAYED_RIGHT_LEG_OFF;
         }

         break;
      case DELAYED_RIGHT_LEG_ON:
         if (logicSwitchOnTime())
         {
            sendPowerState(7, 2);
            logicSwitchTime = getCurrentMonotonicClockTime();
            logicPowerState = PowerState.DELAYED_BACK_ON;
         }
         break;
      case DELAYED_BACK_ON:
         if (logicSwitchOnTime())
         {
            sendPowerState(8, 2);
            logicSwitchTime = getCurrentMonotonicClockTime();
            logicPowerState = PowerState.IDLE;
         }
         break;
      case DELAYED_RIGHT_LEG_OFF:
         if (logicSwitchOffTime())
         {
            sendPowerState(7, 0);
            logicSwitchTime = getCurrentMonotonicClockTime();
            logicPowerState = PowerState.DELAYED_BACK_OFF;
         }
         break;
      case DELAYED_BACK_OFF:
         if (logicSwitchOffTime())
         {
            sendPowerState(8, 0);
            logicSwitchTime = getCurrentMonotonicClockTime();
            logicPowerState = PowerState.IDLE;
         }
         break;
      default:
         throw new RuntimeException("Should not get here");
      }

      motorPowerStateRequest.set(0);
      logicPowerStateRequest.set(0);
   }

   public static void startStreamingData()
   {
      try
      {
         NetworkInterface iface = LogUtils.getMyInterface(NetworkParameters.getHost(NetworkParameterKeys.robotController));

         InetAddress group = InetAddress.getByName(AcsellNetworkParameters.ACSELL_MULTICAST_GROUP);
         InetSocketAddress streamAddress = new InetSocketAddress(group, AcsellNetworkParameters.UDP_MULTICAST_STREAM_COMMAND_PORT);

         DatagramChannel channel = DatagramChannel.open(StandardProtocolFamily.INET).setOption(StandardSocketOptions.SO_REUSEADDR, true)
               .setOption(StandardSocketOptions.IP_MULTICAST_IF, iface);
         channel.join(group, iface);

         ThreadTools.sleep(100);

         ByteBuffer command = ByteBuffer.allocate(11);

         // Canonical command, does not work for now
         //         command.put((byte) 1);
         //         command.put((byte) 0);
         //         command.put((byte) 1);
         //         command.put((byte) 1);
         //         command.put((byte) 0);
         // This should work
         command.put((byte) 0x04);
         command.put((byte) 0x00);
         command.put((byte) 0x01);
         command.put((byte) 0x01);
         command.put((byte) 0x06);
         command.put((byte) 0x00);
         command.put((byte) 0xff);
         command.put((byte) 0x03);
         command.put((byte) 0xe8);
         command.put((byte) 0x03);
         command.put((byte) 0x00);

         command.flip();
         System.out.println("Send command of " + channel.send(command, streamAddress) + " bytes");
         channel.close();

      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }

   }

}
