package us.ihmc.quadrupedRobotics.input;

import java.io.IOException;
import java.util.Collections;
import java.util.EnumMap;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import us.ihmc.quadrupedRobotics.packets.QuadrupedNeckJointPositionPacket;
import us.ihmc.quadrupedRobotics.params.DoubleParameter;
import us.ihmc.quadrupedRobotics.params.ParameterFactory;
import us.ihmc.communication.net.NetClassList;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.SdfLoader.partNames.QuadrupedJointName;

public class QuadrupedHeadTeleopNode implements InputEventCallback
{
   /**
    * Period at which to send control packets.
    */
   private static final double DT = 0.01;

   private final ParameterFactory parameterFactory = new ParameterFactory(getClass());
   private final DoubleParameter proximalNeckYawScaleParameter = parameterFactory.createDouble("proximalNeckYawScale", 0.9);
   private final DoubleParameter proximalNeckPitchScaleParameter = parameterFactory.createDouble("proximalNeckPitchScale", 0.9);
   private final DoubleParameter proximalNeckRollScaleParameter = parameterFactory.createDouble("proximalNeckRollScale", 0.5);
   private final DoubleParameter distalNeckYawScaleParameter = parameterFactory.createDouble("distalNeckYawScale", 0.8);
   private final DoubleParameter distalNeckPitchScaleParameter = parameterFactory.createDouble("distalNeckPitchScale", 0.8);
   private final DoubleParameter distalNeckRollScaleParameter = parameterFactory.createDouble("distalNeckRollScale", 0.5);

   private final PollingInputDevice device;
   private final ScheduledExecutorService executorService = Executors.newSingleThreadScheduledExecutor();
   private final PacketCommunicator packetCommunicator;
   private final Map<InputChannel, Double> channels = Collections.synchronizedMap(new EnumMap<InputChannel, Double>(InputChannel.class));

   private boolean teleopEnabled = true;
   private HashMap<QuadrupedJointName, Double> neckJointPositionSetpoints = new HashMap<>();

   public QuadrupedHeadTeleopNode(String host, NetClassList netClassList, PollingInputDevice device) throws IOException
   {

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
         double distalNeckYaw = get(InputChannel.RIGHT_STICK_X) * distalNeckYawScaleParameter.get();
         double distalNeckPitch = get(InputChannel.RIGHT_STICK_Y) * distalNeckPitchScaleParameter.get();
         double distalNeckRoll = (get(InputChannel.RIGHT_TRIGGER) - get(InputChannel.LEFT_TRIGGER)) * distalNeckRollScaleParameter.get();
         double proximalNeckYaw = get(InputChannel.LEFT_STICK_X) * proximalNeckYawScaleParameter.get();
         double proximalNeckPitch = get(InputChannel.LEFT_STICK_Y) * proximalNeckPitchScaleParameter.get();
         double proximalNeckRoll = 0.0 * proximalNeckRollScaleParameter.get();

         neckJointPositionSetpoints.put(QuadrupedJointName.DISTAL_NECK_YAW, distalNeckYaw);
         neckJointPositionSetpoints.put(QuadrupedJointName.DISTAL_NECK_PITCH, distalNeckPitch);
         neckJointPositionSetpoints.put(QuadrupedJointName.DISTAL_NECK_ROLL, distalNeckRoll);
         neckJointPositionSetpoints.put(QuadrupedJointName.PROXIMAL_NECK_YAW, proximalNeckYaw);
         neckJointPositionSetpoints.put(QuadrupedJointName.PROXIMAL_NECK_PITCH, proximalNeckPitch);
         neckJointPositionSetpoints.put(QuadrupedJointName.PROXIMAL_NECK_ROLL, proximalNeckRoll);

         if (teleopEnabled)
         {
            QuadrupedNeckJointPositionPacket neckJointPositionPacket = new QuadrupedNeckJointPositionPacket(neckJointPositionSetpoints);
            packetCommunicator.send(neckJointPositionPacket);
         }

      }
      catch (Exception e)
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
            teleopEnabled = true;
         }
         break;
      case BUTTON_X:
         if (get(InputChannel.BUTTON_X) > 0.5)
         {
            teleopEnabled = false;
         }
         break;
      case BUTTON_Y:
         if (get(InputChannel.BUTTON_Y) > 0.5)
         {
            teleopEnabled = false;
         }
         break;
      case BUTTON_B:
         if (get(InputChannel.BUTTON_B) > 0.5)
         {
            teleopEnabled = false;
         }
         break;
      }
   }

   private double get(InputChannel channel)
   {
      return channels.get(channel);
   }
}
