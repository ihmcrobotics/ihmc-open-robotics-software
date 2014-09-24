package us.ihmc.steppr.hardware;

import java.io.IOException;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.net.NetworkInterface;
import java.net.StandardProtocolFamily;
import java.net.StandardSocketOptions;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.channels.DatagramChannel;
import java.nio.channels.MembershipKey;
import java.util.EnumMap;

import us.ihmc.realtime.MonotonicTime;
import us.ihmc.realtime.PeriodicParameters;
import us.ihmc.realtime.PriorityParameters;
import us.ihmc.realtime.RealtimeThread;
import us.ihmc.steppr.hardware.configuration.StepprNetworkParameters;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.IntegerYoVariable;

public class StepprSetup extends RealtimeThread
{
   private static final int PARAMETER_INDEX_ENCODER_WRAP = 22;

   private enum NudgeState
   {
      IDLE, INCREASE, DECREASE
   }

   private static final PriorityParameters priority = new PriorityParameters(50);
   private static final MonotonicTime interval = new MonotonicTime(0, 100000000);

   private final YoVariableRegistry registry = new YoVariableRegistry("StepprSetup");

   private final EnumMap<StepprActuator, IntegerYoVariable> nudges = new EnumMap<>(StepprActuator.class);
   private final EnumMap<StepprActuator, NudgeState> nudgeStates = new EnumMap<>(StepprActuator.class);
   
   private DatagramChannel channel;
   private ByteBuffer sendBuffer;
   private InetSocketAddress sendAddress;

   
   
   public StepprSetup(YoVariableRegistry parentRegistry)
   {
      super(priority, new PeriodicParameters(interval));
      for (StepprActuator actuator : StepprActuator.values)
      {
         nudges.put(actuator, new IntegerYoVariable(actuator.getName() + "Nudge", registry));
         nudgeStates.put(actuator, NudgeState.IDLE);
      }

      
      parentRegistry.addChild(registry);
   }

   @Override
   public void run()
   {
      try
      {
         NetworkInterface iface = NetworkInterface.getByInetAddress(InetAddress.getByName(StepprNetworkParameters.CONTROL_COMPUTER_HOST));
         System.out.println("Binding setup to interface: " + iface);

         InetAddress group = InetAddress.getByName(StepprNetworkParameters.STEPPR_MULTICAST_GROUP);
         sendAddress = new InetSocketAddress(group, StepprNetworkParameters.UDP_MULTICAST_PARAMETER_REQUEST_PORT);
         InetSocketAddress receiveAddress = new InetSocketAddress(StepprNetworkParameters.UDP_MULTICAST_PARAMETER_REPLY_PORT);
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

            for (StepprActuator actuator : StepprActuator.values)
            {
               if (nudgeStates.get(actuator) == NudgeState.IDLE)
               {
                  IntegerYoVariable nudge = nudges.get(actuator);
                  if (nudge.getIntegerValue() > 0)
                  {
                     nudgeStates.put(actuator, NudgeState.INCREASE);
                  }
                  else if (nudge.getIntegerValue() < 0)
                  {
                     nudgeStates.put(actuator, NudgeState.DECREASE);
                  }
                  nudge.set(0);
               }
            }

            while (channel.receive(receiveBuffer) != null)
            {
               receiveBuffer.flip();
               handleResponse(receiveBuffer);
               receiveBuffer.clear();
            }

            for (StepprActuator actuator : StepprActuator.values)
            {
               if (nudgeStates.get(actuator) == NudgeState.INCREASE || nudgeStates.get(actuator) == NudgeState.DECREASE)
               {
                  requestParameterValue(actuator, PARAMETER_INDEX_ENCODER_WRAP);
               }
            }

         }
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }

   private void handleResponse(ByteBuffer receiveBuffer) throws IOException
   {
      int bus = (receiveBuffer.get() & 0xFF) - 1;
      int length = receiveBuffer.get() & 0xFF;
      if(length < 15)
      {
         return;
      }
      int index = receiveBuffer.get() & 0xFF;
      @SuppressWarnings("unused")
      int format = receiveBuffer.get() & 0xFF;
      @SuppressWarnings("unused")
      int subMessageLength = receiveBuffer.get() & 0xFF;
      int messageType = receiveBuffer.get() & 0xFF;

      switch(messageType)
      {
      case 3:
         handleParameterMessage(receiveBuffer, bus, index);
         default:
            // Nothing to do here
      }
   }

   private void requestParameterValue(StepprActuator actuator, int parameterIndex) throws IOException
   {
      sendBuffer.clear();
      sendBuffer.put((byte) 1);
      sendBuffer.put((byte) 1);
      sendBuffer.put((byte) actuator.getBus());
      sendBuffer.put((byte) 6);
      sendBuffer.put((byte) actuator.getIndex());
      sendBuffer.put((byte) 0);
      sendBuffer.put((byte) 3);
      sendBuffer.put((byte) 2);
      sendBuffer.putShort((short) parameterIndex);
      sendBuffer.flip();
      channel.send(sendBuffer, sendAddress);
   }

   private void setParameterValue(StepprActuator actuator, int parameterIndex, double value) throws IOException
   {
      sendBuffer.clear();
      sendBuffer.put((byte) 1);
      sendBuffer.put((byte) 1);
      sendBuffer.put((byte) actuator.getBus());
      sendBuffer.put((byte) 14);
      sendBuffer.put((byte) actuator.getIndex());
      sendBuffer.put((byte) 0);
      sendBuffer.put((byte) 3);
      sendBuffer.put((byte) 3);
      sendBuffer.putShort((short) parameterIndex);
      sendBuffer.putDouble(value);
      sendBuffer.flip();
      channel.send(sendBuffer, sendAddress);
   }
   
   private void handleParameterMessage(ByteBuffer receiveBuffer, int bus, int index) throws IOException
   {
      
      int parameterType = receiveBuffer.get() & 0xFF;
      switch(parameterType)
      {
      case 2:
         handleParameterValueResponse(receiveBuffer, bus, index);
         break;
      default:
         // Nothing to do here
      }
      
   }

   private void handleParameterValueResponse(ByteBuffer receiveBuffer, int bus, int index) throws IOException
   {
      for (StepprActuator actuator : StepprActuator.values)
      {
         if(actuator.getBus() == bus && actuator.getIndex() == index)
         {
            int parameterIndex = receiveBuffer.getShort() & 0xFF;
            switch(parameterIndex)
            {
            case PARAMETER_INDEX_ENCODER_WRAP:
               double value = receiveBuffer.getDouble();
               if(nudgeStates.get(actuator) == NudgeState.INCREASE)
               {
                  value += 1;
               }
               else if (nudgeStates.get(actuator) == NudgeState.DECREASE)
               {
                  value -= 1;
               }
               
               setParameterValue(actuator, PARAMETER_INDEX_ENCODER_WRAP, value);
               nudgeStates.put(actuator, NudgeState.IDLE);
               
               break;
            }
         }
      }
   }

}
