package us.ihmc.quadrupedRobotics.input;

import java.io.IOException;
import java.util.Collections;
import java.util.EnumMap;
import java.util.Map;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceControllerRequestedEvent;
import us.ihmc.quadrupedRobotics.input.value.InputValueIntegrator;
import us.ihmc.quadrupedRobotics.packets.BodyOrientationPacket;
import us.ihmc.quadrupedRobotics.packets.ComPositionPacket;
import us.ihmc.quadrupedRobotics.packets.PlanarVelocityPacket;
import us.ihmc.quadrupedRobotics.packets.QuadrupedForceControllerEventPacket;
import us.ihmc.quadrupedRobotics.params.DoubleParameter;
import us.ihmc.quadrupedRobotics.params.ParameterFactory;
import us.ihmc.communication.net.NetClassList;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.util.NetworkPorts;

public class QuadrupedBodyTeleopNode implements InputEventCallback
{
   /**
    * Period at which to send control packets.
    */
   private static final double DT = 0.01;

   private enum QuadrupedTeleopMode
   {
      POSITION, VELOCITY
   }

   private final ParameterFactory parameterFactory = new ParameterFactory(getClass());
   private final DoubleParameter rollScaleParameter = parameterFactory.createDouble("paramRollScale", 0.15);
   private final DoubleParameter pitchScaleParameter = parameterFactory.createDouble("paramPitchScale", 0.15);
   private final DoubleParameter yawScaleParameter = parameterFactory.createDouble("paramYawScale", 0.15);
   private final DoubleParameter xScaleParameter = parameterFactory.createDouble("paramXScale", 0.20);
   private final DoubleParameter yScaleParameter = parameterFactory.createDouble("paramYScale", 0.10);
   private final DoubleParameter vxScaleParameter = parameterFactory.createDouble("paramVxScale", 1.0);
   private final DoubleParameter vyScaleParameter = parameterFactory.createDouble("paramVyScale", 0.5);
   private final DoubleParameter vzScaleParameter = parameterFactory.createDouble("paramVzScale", 0.25);
   private final DoubleParameter wzScaleParameter = parameterFactory.createDouble("paramWzScale", 1.0);
   private final DoubleParameter defaultComHeightParameter = parameterFactory.createDouble("paramDefaultComHeight", 0.55);

   private final ScheduledExecutorService executorService = Executors.newSingleThreadScheduledExecutor();
   private final PacketCommunicator packetCommunicator;
   private final PollingInputDevice device;
   private final Map<InputChannel, Double> channels = Collections.synchronizedMap(new EnumMap<InputChannel, Double>(InputChannel.class));

   private InputValueIntegrator comZ;
   private QuadrupedTeleopMode mode;

   public QuadrupedBodyTeleopNode(String host, NetClassList netClassList, PollingInputDevice device) throws IOException
   {
      // TODO: Don't hardcode localhost
      this.packetCommunicator = PacketCommunicator.createTCPPacketCommunicatorClient(host, NetworkPorts.CONTROLLER_PORT, netClassList);
      this.device = device;
      this.comZ = new InputValueIntegrator(DT, defaultComHeightParameter.get());

      // Initialize all channels to zero.
      for (InputChannel channel : InputChannel.values())
      {
         channels.put(channel, 0.0);
      }

      // Initialize teleop mode
      mode = QuadrupedTeleopMode.POSITION;
   }

   public void start() throws IOException
   {
      packetCommunicator.connect();

      // Send packets and integrate at fixed interval.
      executorService.scheduleAtFixedRate(new Runnable()
      {
         @Override
         public void run()
         {
            update();
         }
      }, 0, (long) (DT * 1000), TimeUnit.MILLISECONDS);

      // Poll indefinitely.
      device.registerCallback(this);
      device.poll();
   }

   public void update()
   {
      double bodyYaw = 0.0;
      double bodyRoll = 0.0;
      double bodyPitch = get(InputChannel.RIGHT_STICK_Y) * pitchScaleParameter.get();
      if (mode == QuadrupedTeleopMode.POSITION)
      {
         bodyYaw = get(InputChannel.RIGHT_STICK_X) * yawScaleParameter.get();
         bodyRoll = -get(InputChannel.LEFT_STICK_X) * rollScaleParameter.get();
      }
      BodyOrientationPacket orientationPacket = new BodyOrientationPacket(bodyYaw, bodyPitch, bodyRoll);
      packetCommunicator.send(orientationPacket);

      double xVelocity = 0.0;
      double yVelocity = 0.0;
      double yawRate = 0.0;
      if (mode == QuadrupedTeleopMode.VELOCITY)
      {
         xVelocity = get(InputChannel.LEFT_STICK_Y) * vxScaleParameter.get();
         yVelocity = get(InputChannel.LEFT_STICK_X) * vyScaleParameter.get();
         yawRate = get(InputChannel.RIGHT_STICK_X) * wzScaleParameter.get();
      }
      PlanarVelocityPacket velocityPacket = new PlanarVelocityPacket(xVelocity, yVelocity, yawRate);
      packetCommunicator.send(velocityPacket);

      double comX = 0.0;
      double comY = 0.0;
      double comZdot = (get(InputChannel.RIGHT_BUTTON) - get(InputChannel.LEFT_BUTTON)) * vzScaleParameter.get();
      if (mode == QuadrupedTeleopMode.POSITION)
      {
         comX = get(InputChannel.LEFT_STICK_Y) * xScaleParameter.get();
      }
      ComPositionPacket comPositionPacket = new ComPositionPacket(comX, comY, comZ.update(comZdot));
      packetCommunicator.send(comPositionPacket);
   }

   @Override
   public void onInputEvent(InputEvent e)
   {
      // Store updated value in a cache so historical values for all channels can be used.
      channels.put(e.getChannel(), e.getValue());

      // Handle events that should trigger once immediately after the event is triggered.
      switch (e.getChannel())
      {
      case BUTTON_A:
         if (get(InputChannel.BUTTON_A) > 0.5)
         {
            QuadrupedForceControllerEventPacket eventPacket = new QuadrupedForceControllerEventPacket(QuadrupedForceControllerRequestedEvent.REQUEST_STAND);
            packetCommunicator.send(eventPacket);
            mode = QuadrupedTeleopMode.POSITION;
         }
         break;
      case BUTTON_X:
         if (get(InputChannel.BUTTON_X) > 0.5)
         {
            QuadrupedForceControllerEventPacket eventPacket = new QuadrupedForceControllerEventPacket(QuadrupedForceControllerRequestedEvent.REQUEST_TROT);
            packetCommunicator.send(eventPacket);
            mode = QuadrupedTeleopMode.VELOCITY;
         }
         break;
      case BUTTON_Y:
         if (get(InputChannel.BUTTON_Y) > 0.5)
         {
            QuadrupedForceControllerEventPacket eventPacket = new QuadrupedForceControllerEventPacket(QuadrupedForceControllerRequestedEvent.REQUEST_AMBLE);
            packetCommunicator.send(eventPacket);
            mode = QuadrupedTeleopMode.VELOCITY;
         }
         break;
      case BUTTON_B:
         if (get(InputChannel.BUTTON_B) > 0.5)
         {
            QuadrupedForceControllerEventPacket eventPacket = new QuadrupedForceControllerEventPacket(QuadrupedForceControllerRequestedEvent.REQUEST_PACE);
            packetCommunicator.send(eventPacket);
            mode = QuadrupedTeleopMode.VELOCITY;
         }
         break;
      }
   }

   private double get(InputChannel channel)
   {
      return channels.get(channel);
   }
}
