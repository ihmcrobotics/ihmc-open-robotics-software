package us.ihmc.aware.input;

import java.io.IOException;
import java.util.Collections;
import java.util.EnumMap;
import java.util.Map;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import us.ihmc.aware.controller.force.QuadrupedForceControllerEvent;
import us.ihmc.aware.input.value.InputValueIntegrator;
import us.ihmc.aware.packets.*;
import us.ihmc.aware.params.ParameterMap;
import us.ihmc.aware.params.ParameterMapRepository;
import us.ihmc.communication.net.NetClassList;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;

public class QuadrupedTeleopNode implements InputEventCallback
{
   /**
    * Period at which to send control packets.
    */
   private static final double DT = 0.01;

   private static final String PARAM_ROLL_SCALE = "rollScale";
   private static final String PARAM_PITCH_SCALE = "pitchScale";
   private static final String PARAM_YAW_SCALE = "yawScale";
   private static final String PARAM_X_SCALE = "xScale";
   private static final String PARAM_Y_SCALE = "yScale";
   private static final String PARAM_VX_SCALE = "vxScale";
   private static final String PARAM_VY_SCALE = "vyScale";
   private static final String PARAM_VZ_SCALE = "vzScale";
   private static final String PARAM_WZ_SCALE = "wzScale";
   private static final String PARAM_DEFAULT_COM_HEIGHT = "defaultComHeight";

   private final ScheduledExecutorService executorService = Executors.newSingleThreadScheduledExecutor();

   private final YoVariableRegistry registry = new YoVariableRegistry(QuadrupedTeleopNode.class.getSimpleName());
   private final ParameterMapRepository repository = new ParameterMapRepository(registry);
   private final ParameterMap params = repository.get(QuadrupedTeleopNode.class);

   private final PacketCommunicator packetCommunicator;

   private final PollingInputDevice device;
   private final Map<InputChannel, Double> channels = Collections.synchronizedMap(new EnumMap<InputChannel, Double>(InputChannel.class));

   private InputValueIntegrator comHeight;

   public QuadrupedTeleopNode(NetClassList netClassList, PollingInputDevice device) throws IOException
   {
      params.setDefault(PARAM_ROLL_SCALE, 0.2);
      params.setDefault(PARAM_PITCH_SCALE, 0.2);
      params.setDefault(PARAM_YAW_SCALE, 0.4);
      params.setDefault(PARAM_X_SCALE, 0.20);
      params.setDefault(PARAM_Y_SCALE, 0.10);
      params.setDefault(PARAM_VX_SCALE, 1.0);
      params.setDefault(PARAM_VY_SCALE, 0.5);
      params.setDefault(PARAM_VZ_SCALE, 0.5);
      params.setDefault(PARAM_WZ_SCALE, 1.0);
      params.setDefault(PARAM_DEFAULT_COM_HEIGHT, 0.55);

      // TODO: Don't hardcode localhost
      this.packetCommunicator = PacketCommunicator
            .createTCPPacketCommunicatorClient("localhost", NetworkPorts.XBOX_CONTROLLER_TELEOP_PORT, netClassList);
      this.device = device;
      this.comHeight = new InputValueIntegrator(DT, params.get(PARAM_DEFAULT_COM_HEIGHT));

      // Initialize all channels to zero.
      for (InputChannel channel : InputChannel.values())
      {
         channels.put(channel, 0.0);
      }
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
      double roll = (get(InputChannel.RIGHT_TRIGGER) - get(InputChannel.LEFT_TRIGGER)) * params.get(PARAM_ROLL_SCALE);
      double pitch = get(InputChannel.RIGHT_STICK_Y) * params.get(PARAM_PITCH_SCALE);
      double yaw = get(InputChannel.RIGHT_STICK_X) * params.get(PARAM_YAW_SCALE);
      BodyOrientationPacket orientationPacket = new BodyOrientationPacket(yaw, pitch, roll);
      packetCommunicator.send(orientationPacket);

      double vx = get(InputChannel.LEFT_STICK_Y) * params.get(PARAM_VX_SCALE);
      double vy = get(InputChannel.LEFT_STICK_X) * params.get(PARAM_VY_SCALE);
      double wz = get(InputChannel.RIGHT_STICK_X) * params.get(PARAM_WZ_SCALE);
      PlanarVelocityPacket velocityPacket = new PlanarVelocityPacket(vx, vy, wz);
      packetCommunicator.send(velocityPacket);

      double x = get(InputChannel.LEFT_STICK_Y) * params.get(PARAM_X_SCALE);
      double y = get(InputChannel.LEFT_STICK_X) * params.get(PARAM_Y_SCALE);
      double vz = (get(InputChannel.RIGHT_BUTTON) - get(InputChannel.LEFT_BUTTON)) * params.get(PARAM_VZ_SCALE);
      ComPositionPacket comPositionPacket = new ComPositionPacket(x, y, comHeight.update(vz));
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
      case HOME_BUTTON:
         if (get(InputChannel.HOME_BUTTON) > 0.5)
         {
            QuadrupedForceControllerEventPacket eventPacket = new QuadrupedForceControllerEventPacket(
                  QuadrupedForceControllerEvent.REQUEST_TROT);
            packetCommunicator.send(eventPacket);
         }
         break;
      }
   }

   private double get(InputChannel channel)
   {
      return channels.get(channel);
   }
}
