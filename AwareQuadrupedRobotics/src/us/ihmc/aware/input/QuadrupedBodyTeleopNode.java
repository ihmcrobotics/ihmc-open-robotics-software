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

public class QuadrupedBodyTeleopNode implements InputEventCallback
{
   /**
    * Period at which to send control packets.
    */
   private static final double DT = 0.01;

   private enum QuadrupedTeleopMode {POSITION, VELOCITY}

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

   private final YoVariableRegistry registry = new YoVariableRegistry(QuadrupedBodyTeleopNode.class.getSimpleName());
   private final ParameterMapRepository repository = new ParameterMapRepository(registry);
   private final ParameterMap params = repository.get(QuadrupedBodyTeleopNode.class);

   private final ScheduledExecutorService executorService = Executors.newSingleThreadScheduledExecutor();
   private final PacketCommunicator packetCommunicator;
   private final PollingInputDevice device;
   private final Map<InputChannel, Double> channels = Collections.synchronizedMap(new EnumMap<InputChannel, Double>(InputChannel.class));

   private InputValueIntegrator comZ;
   private QuadrupedTeleopMode mode;

   public QuadrupedBodyTeleopNode(String host, NetClassList netClassList, PollingInputDevice device) throws IOException
   {
      params.setDefault(PARAM_ROLL_SCALE, 0.15);
      params.setDefault(PARAM_PITCH_SCALE, 0.15);
      params.setDefault(PARAM_YAW_SCALE, 0.15);
      params.setDefault(PARAM_X_SCALE, 0.20);
      params.setDefault(PARAM_Y_SCALE, 0.10);
      params.setDefault(PARAM_VX_SCALE, 0.1);
      params.setDefault(PARAM_VY_SCALE, 0.5);
      params.setDefault(PARAM_VZ_SCALE, 0.25);
      params.setDefault(PARAM_WZ_SCALE, 0.25);
      params.setDefault(PARAM_DEFAULT_COM_HEIGHT, 0.55);

      // TODO: Don't hardcode localhost
      this.packetCommunicator = PacketCommunicator.createTCPPacketCommunicatorClient(host, NetworkPorts.CONTROLLER_PORT, netClassList);
      this.device = device;
      this.comZ = new InputValueIntegrator(DT, params.get(PARAM_DEFAULT_COM_HEIGHT));

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
      double bodyPitch = get(InputChannel.RIGHT_STICK_Y) * params.get(PARAM_PITCH_SCALE);
      if (mode == QuadrupedTeleopMode.POSITION)
      {
         bodyYaw = get(InputChannel.RIGHT_STICK_X) * params.get(PARAM_YAW_SCALE);
         bodyRoll = -get(InputChannel.LEFT_STICK_X) * params.get(PARAM_ROLL_SCALE);
      }
      BodyOrientationPacket orientationPacket = new BodyOrientationPacket(bodyYaw, bodyPitch, bodyRoll);
      packetCommunicator.send(orientationPacket);

      double xVelocity = 0.0;
      double yVelocity = 0.0;
      double yawRate = 0.0;
      if (mode == QuadrupedTeleopMode.VELOCITY)
      {
         xVelocity = get(InputChannel.LEFT_STICK_Y) * params.get(PARAM_VX_SCALE);
         yVelocity = get(InputChannel.LEFT_STICK_X) * params.get(PARAM_VY_SCALE);
         yawRate = get(InputChannel.RIGHT_STICK_X) * params.get(PARAM_WZ_SCALE);
      }
      PlanarVelocityPacket velocityPacket = new PlanarVelocityPacket(xVelocity, yVelocity, yawRate);
      packetCommunicator.send(velocityPacket);

      double comX = 0.0;
      double comY = 0.0;
      double comZdot = (get(InputChannel.RIGHT_BUTTON) - get(InputChannel.LEFT_BUTTON)) * params.get(PARAM_VZ_SCALE);
      if (mode == QuadrupedTeleopMode.POSITION)
      {
         comX = get(InputChannel.LEFT_STICK_Y) * params.get(PARAM_X_SCALE);
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
            QuadrupedForceControllerEventPacket eventPacket = new QuadrupedForceControllerEventPacket(QuadrupedForceControllerEvent.REQUEST_STAND);
            packetCommunicator.send(eventPacket);
            mode = QuadrupedTeleopMode.POSITION;
         }
         break;
      case BUTTON_X:
         if (get(InputChannel.BUTTON_X) > 0.5)
         {
            QuadrupedForceControllerEventPacket eventPacket = new QuadrupedForceControllerEventPacket(QuadrupedForceControllerEvent.REQUEST_PACE);
            packetCommunicator.send(eventPacket);
            mode = QuadrupedTeleopMode.VELOCITY;
         }
         break;
      case BUTTON_Y:
         if (get(InputChannel.BUTTON_Y) > 0.5)
         {
            QuadrupedForceControllerEventPacket eventPacket = new QuadrupedForceControllerEventPacket(QuadrupedForceControllerEvent.REQUEST_AMBLE);
            packetCommunicator.send(eventPacket);
            mode = QuadrupedTeleopMode.VELOCITY;
         }
         break;
      case BUTTON_B:
         if (get(InputChannel.BUTTON_B) > 0.5)
         {
            QuadrupedForceControllerEventPacket eventPacket = new QuadrupedForceControllerEventPacket(QuadrupedForceControllerEvent.REQUEST_TROT);
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
