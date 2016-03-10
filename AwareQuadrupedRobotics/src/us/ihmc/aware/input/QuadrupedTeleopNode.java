package us.ihmc.aware.input;

import java.io.IOException;
import java.util.EnumMap;
import java.util.Map;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import us.ihmc.aware.controller.force.QuadrupedForceControllerEvent;
import us.ihmc.aware.input.devices.XboxControllerInputDevice;
import us.ihmc.aware.packets.BodyOrientationPacket;
import us.ihmc.aware.packets.ComPositionPacket;
import us.ihmc.aware.packets.PlanarVelocityPacket;
import us.ihmc.aware.packets.QuadrupedForceControllerEventPacket;
import us.ihmc.aware.params.ParameterMap;
import us.ihmc.aware.params.ParameterMapRepository;
import us.ihmc.communication.net.NetClassList;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;

public class QuadrupedTeleopNode implements InputEventCallback
{
   private static final String PARAM_ROLL_SCALE = "rollScale";
   private static final String PARAM_PITCH_SCALE = "pitchScale";
   private static final String PARAM_YAW_SCALE = "yawScale";

   private static final String PARAM_XDOT_SCALE = "xDotScale";
   private static final String PARAM_YDOT_SCALE = "yDotScale";
   private static final String PARAM_ZDOT_SCALE = "zDotScale";
   private static final String PARAM_ADOT_SCALE = "yawRateScale";

   private static final String PARAM_EXP_ORDER = "expOrder";

   private static final String PARAM_DEFAULT_COM_HEIGHT = "defaultComHeight";

   private static final double DT = 0.01;

   private final ScheduledExecutorService executorService = Executors.newSingleThreadScheduledExecutor();

   private final YoVariableRegistry registry = new YoVariableRegistry(QuadrupedTeleopNode.class.getSimpleName());
   private final ParameterMapRepository repository = new ParameterMapRepository(registry);
   private final ParameterMap params = repository.get(QuadrupedTeleopNode.class);

   private final PacketCommunicator packetCommunicator;
   private final InputDevice input;
   private final Map<InputChannel, Double> channels = new EnumMap<>(InputChannel.class);

   private double comHeight;

   public QuadrupedTeleopNode(NetClassList netClassList) throws IOException
   {
      // TODO: Don't use TOUCH_MODULE_PORT
      // TODO: Don't hardcode localhost
      this.packetCommunicator = PacketCommunicator
            .createTCPPacketCommunicatorClient("localhost", NetworkPorts.TOUCH_MODULE_PORT, netClassList);
      this.input = new XboxControllerInputDevice();
      this.input.registerCallback(this);

      for (InputChannel axis : InputChannel.values())
      {
         channels.put(axis, 0.0);
      }

      params.setDefault(PARAM_ROLL_SCALE, 0.2);
      params.setDefault(PARAM_PITCH_SCALE, 0.3);
      params.setDefault(PARAM_YAW_SCALE, 0.6);

      params.setDefault(PARAM_XDOT_SCALE, 1.0);
      params.setDefault(PARAM_YDOT_SCALE, 0.5);
      params.setDefault(PARAM_ZDOT_SCALE, 0.5);
      params.setDefault(PARAM_ADOT_SCALE, 1.0);

      params.setDefault(PARAM_EXP_ORDER, 3.0);

      params.setDefault(PARAM_DEFAULT_COM_HEIGHT, 0.55);

      this.comHeight = params.get(PARAM_DEFAULT_COM_HEIGHT);

      executorService.scheduleAtFixedRate(new Runnable()
      {
         @Override
         public void run()
         {
            update();
         }
      }, 0, (long) (DT * 1000), TimeUnit.MILLISECONDS);
   }

   public void start() throws IOException
   {
      packetCommunicator.connect();

      // Poll indefinitely.
      input.poll();
   }

   public void update()
   {
      double roll = (channels.get(InputChannel.RIGHT_TRIGGER) - channels.get(InputChannel.LEFT_TRIGGER)) * params.get(PARAM_ROLL_SCALE);
      double pitch = channels.get(InputChannel.RIGHT_STICK_Y) * params.get(PARAM_PITCH_SCALE);
      double yaw = channels.get(InputChannel.RIGHT_STICK_X) * params.get(PARAM_YAW_SCALE);
      BodyOrientationPacket orientationPacket = new BodyOrientationPacket(yaw, pitch, roll);
      packetCommunicator.send(orientationPacket);

      double xdot = exp(channels.get(InputChannel.LEFT_STICK_Y)) * params.get(PARAM_XDOT_SCALE);
      double ydot = exp(channels.get(InputChannel.LEFT_STICK_X)) * params.get(PARAM_YDOT_SCALE);
      double adot = exp(channels.get(InputChannel.RIGHT_STICK_X)) * params.get(PARAM_ADOT_SCALE);
      PlanarVelocityPacket velocityPacket = new PlanarVelocityPacket(xdot, ydot, adot);
      packetCommunicator.send(velocityPacket);

      double zdot = (channels.get(InputChannel.RIGHT_BUTTON) - channels.get(InputChannel.LEFT_BUTTON)) * params.get(PARAM_ZDOT_SCALE);
      comHeight = comHeight + zdot * DT;
      ComPositionPacket comPositionPacket = new ComPositionPacket(0.0, 0.0, comHeight);
      packetCommunicator.send(comPositionPacket);
   }

   @Override
   public void onInputEvent(InputEvent e)
   {
      // Store updated value in a cache so historical values for all channels can be used.
      channels.put(e.getChannel(), e.getValue());

      switch (e.getChannel())
      {
      case HOME_BUTTON:
         if (channels.get(InputChannel.HOME_BUTTON) > 0.5)
         {
            QuadrupedForceControllerEventPacket eventPacket = new QuadrupedForceControllerEventPacket(
                  QuadrupedForceControllerEvent.REQUEST_TROT);
            packetCommunicator.send(eventPacket);
         }
         break;
      }
   }

   private double exp(double in)
   {
      return Math.pow(in, params.get(PARAM_EXP_ORDER));
   }
}
