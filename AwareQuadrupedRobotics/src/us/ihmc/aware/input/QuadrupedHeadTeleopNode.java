package us.ihmc.aware.input;

import us.ihmc.aware.input.value.InputValueIntegrator;
import us.ihmc.aware.packets.QuadrupedNeckJointPositionPacket;
import us.ihmc.aware.params.ParameterMap;
import us.ihmc.aware.params.ParameterMapRepository;
import us.ihmc.communication.net.NetClassList;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedJointName;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;

import java.io.IOException;
import java.util.Collections;
import java.util.EnumMap;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

public class QuadrupedHeadTeleopNode implements InputEventCallback
{
   /**
    * Period at which to send control packets.
    */
   private static final double DT = 0.01;

   private static final String PARAM_PROXIMAL_NECK_YAW_SCALE = "proximalNeckYawRateScale";
   private static final String PARAM_PROXIMAL_NECK_PITCH_SCALE = "proximalNeckPitchRateScale";
   private static final String PARAM_PROXIMAL_NECK_ROLL_SCALE = "proximalNeckRollRateScale";
   private static final String PARAM_DISTAL_NECK_YAW_SCALE = "distalNeckYawScale";
   private static final String PARAM_DISTAL_NECK_PITCH_SCALE = "distalNeckPitchScale";
   private static final String PARAM_DISTAL_NECK_ROLL_SCALE = "distalNeckRollScale";

   private final YoVariableRegistry registry = new YoVariableRegistry(QuadrupedHeadTeleopNode.class.getSimpleName());
   private final ParameterMapRepository repository = new ParameterMapRepository(registry);
   private final ParameterMap params = repository.get(QuadrupedHeadTeleopNode.class);

   private final ScheduledExecutorService executorService = Executors.newSingleThreadScheduledExecutor();
   private final PacketCommunicator packetCommunicator;
   private final PollingInputDevice device;
   private final Map<InputChannel, Double> channels = Collections.synchronizedMap(new EnumMap<InputChannel, Double>(InputChannel.class));

   private HashMap<QuadrupedJointName, Double> neckJointPositionSetpoints = new HashMap<>();

   public QuadrupedHeadTeleopNode(String host, NetClassList netClassList, PollingInputDevice device) throws IOException
   {
      params.setDefault(PARAM_PROXIMAL_NECK_YAW_SCALE, 0.9);
      params.setDefault(PARAM_PROXIMAL_NECK_PITCH_SCALE, 0.9);
      params.setDefault(PARAM_PROXIMAL_NECK_ROLL_SCALE, 0.5);
      params.setDefault(PARAM_DISTAL_NECK_YAW_SCALE, 0.9);
      params.setDefault(PARAM_DISTAL_NECK_PITCH_SCALE, 0.9);
      params.setDefault(PARAM_DISTAL_NECK_ROLL_SCALE, 0.4);

      // TODO: Don't hardcode localhost
      this.packetCommunicator = PacketCommunicator.createTCPPacketCommunicatorClient(host, NetworkPorts.CONTROLLER_PORT, netClassList);
      this.device = device;

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
      try
      {
         double distalNeckYaw = get(InputChannel.RIGHT_STICK_X) * params.get(PARAM_DISTAL_NECK_YAW_SCALE);
         double distalNeckPitch = get(InputChannel.RIGHT_STICK_Y) * params.get(PARAM_DISTAL_NECK_PITCH_SCALE);
         double distalNeckRoll = (get(InputChannel.RIGHT_TRIGGER) - get(InputChannel.LEFT_TRIGGER)) * params.get(PARAM_DISTAL_NECK_ROLL_SCALE);
         double proximalNeckYaw = get(InputChannel.LEFT_STICK_X) * params.get(PARAM_PROXIMAL_NECK_YAW_SCALE);
         double proximalNeckPitch = get(InputChannel.LEFT_STICK_Y) * params.get(PARAM_PROXIMAL_NECK_PITCH_SCALE);
         double proximalNeckRoll = 0.0;

         neckJointPositionSetpoints.put(QuadrupedJointName.DISTAL_NECK_YAW, distalNeckYaw);
         neckJointPositionSetpoints.put(QuadrupedJointName.DISTAL_NECK_PITCH, distalNeckPitch);
         neckJointPositionSetpoints.put(QuadrupedJointName.DISTAL_NECK_ROLL, distalNeckRoll);
         neckJointPositionSetpoints.put(QuadrupedJointName.PROXIMAL_NECK_YAW, proximalNeckYaw);
         neckJointPositionSetpoints.put(QuadrupedJointName.PROXIMAL_NECK_PITCH, proximalNeckPitch);
         neckJointPositionSetpoints.put(QuadrupedJointName.PROXIMAL_NECK_ROLL, proximalNeckRoll);

         QuadrupedNeckJointPositionPacket neckJointPositionPacket = new QuadrupedNeckJointPositionPacket(neckJointPositionSetpoints);
         packetCommunicator.send(neckJointPositionPacket);

      } catch (Exception e)
      {
         e.printStackTrace();
      }
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
         }
         break;
      case BUTTON_X:
         if (get(InputChannel.BUTTON_X) > 0.5)
         {
         }
         break;
      case BUTTON_Y:
         if (get(InputChannel.BUTTON_Y) > 0.5)
         {
         }
         break;
      case BUTTON_B:
         if (get(InputChannel.BUTTON_B) > 0.5)
         {
         }
         break;
      }
   }

   private double get(InputChannel channel)
   {
      return channels.get(channel);
   }
}
