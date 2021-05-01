package us.ihmc.quadrupedPlanning.stepStream.input;

import net.java.games.input.Event;
import us.ihmc.communication.net.NetClassList;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.robotics.partNames.QuadrupedJointName;
import us.ihmc.tools.inputDevices.joystick.Joystick;
import us.ihmc.tools.inputDevices.joystick.JoystickCustomizationFilter;
import us.ihmc.tools.inputDevices.joystick.JoystickEventListener;
import us.ihmc.tools.inputDevices.joystick.mapping.XBoxOneMapping;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.io.IOException;
import java.util.Collections;
import java.util.EnumMap;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

public class QuadrupedHeadTeleopNode implements JoystickEventListener
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   /**
    * Period at which to send control packets.
    */
   private static final double DT = 0.01;

   private final DoubleParameter proximalNeckYawScaleParameter = new DoubleParameter("proximalNeckYawScale", registry, 0.9);
   private final DoubleParameter proximalNeckPitchScaleParameter = new DoubleParameter("proximalNeckPitchScale", registry, 0.9);
   private final DoubleParameter proximalNeckRollScaleParameter = new DoubleParameter("proximalNeckRollScale", registry, 0.5);
   private final DoubleParameter distalNeckYawScaleParameter = new DoubleParameter("distalNeckYawScale", registry, 0.8);
   private final DoubleParameter distalNeckPitchScaleParameter = new DoubleParameter("distalNeckPitchScale", registry, 0.8);
   private final DoubleParameter distalNeckRollScaleParameter = new DoubleParameter("distalNeckRollScale", registry, 0.5);

   private final Joystick device;
   private final ScheduledExecutorService executorService = Executors.newSingleThreadScheduledExecutor();
   private final PacketCommunicator packetCommunicator;
   private final Map<XBoxOneMapping, Double> channels = Collections.synchronizedMap(new EnumMap<XBoxOneMapping, Double>(XBoxOneMapping.class));

   private boolean teleopEnabled = true;
   private HashMap<QuadrupedJointName, Double> neckJointPositionSetpoints = new HashMap<>();

   public QuadrupedHeadTeleopNode(String host, NetworkPorts port, NetClassList netClassList, Joystick device, YoRegistry parentRegistry) throws IOException
   {

      // TODO: Don't hardcode localhost
      this.packetCommunicator = PacketCommunicator.createTCPPacketCommunicatorClient(host, port, netClassList);
      this.device = device;

      // Initialize all channels to zero.
      for (XBoxOneMapping channel : XBoxOneMapping.values)
      {
         channels.put(channel, 0.0);
      }

      parentRegistry.addChild(registry);
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

      configureJoystickFilters(device);
      device.addJoystickEventListener(this);
      device.setPollInterval(10);
   }

   private void configureJoystickFilters(Joystick device)
   {
      device.setCustomizationFilter(new JoystickCustomizationFilter(XBoxOneMapping.LEFT_TRIGGER, false, 0.05, 1, 1.0));
      device.setCustomizationFilter(new JoystickCustomizationFilter(XBoxOneMapping.RIGHT_TRIGGER, false, 0.05, 1, 1.0));
      device.setCustomizationFilter(new JoystickCustomizationFilter(XBoxOneMapping.LEFT_STICK_X, true, 0.1, 1));
      device.setCustomizationFilter(new JoystickCustomizationFilter(XBoxOneMapping.LEFT_STICK_Y, true, 0.1, 1));
      device.setCustomizationFilter(new JoystickCustomizationFilter(XBoxOneMapping.RIGHT_STICK_X, true, 0.1, 1));
      device.setCustomizationFilter(new JoystickCustomizationFilter(XBoxOneMapping.RIGHT_STICK_Y, true, 0.1, 1));
   }


   public void update()
   {
      try
      {
         double distalNeckYaw = get(XBoxOneMapping.RIGHT_STICK_X) * distalNeckYawScaleParameter.getValue();
         double distalNeckPitch = get(XBoxOneMapping.RIGHT_STICK_Y) * distalNeckPitchScaleParameter.getValue();
         double distalNeckRoll = (get(XBoxOneMapping.RIGHT_TRIGGER) - get(XBoxOneMapping.LEFT_TRIGGER)) * distalNeckRollScaleParameter.getValue();
         double proximalNeckYaw = get(XBoxOneMapping.LEFT_STICK_X) * proximalNeckYawScaleParameter.getValue();
         double proximalNeckPitch = get(XBoxOneMapping.LEFT_STICK_Y) * proximalNeckPitchScaleParameter.getValue();
         double proximalNeckRoll = 0.0 * proximalNeckRollScaleParameter.getValue();

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
   public void processEvent(Event event)
   {
      // Store updated value in a cache so historical values for all channels can be used.
      channels.put(XBoxOneMapping.getMapping(event), (double) event.getValue());

      // Handle events that should trigger once immediately after the event is triggered.
      switch (XBoxOneMapping.getMapping(event))
      {
      case A:
         if (get(XBoxOneMapping.A) > 0.5)
         {
            teleopEnabled = true;
         }
         break;
      case X:
         if (get(XBoxOneMapping.X) > 0.5)
         {
            teleopEnabled = false;
         }
         break;
      case Y:
         if (get(XBoxOneMapping.Y) > 0.5)
         {
            teleopEnabled = false;
         }
         break;
      case B:
         if (get(XBoxOneMapping.B) > 0.5)
         {
            teleopEnabled = false;
         }
         break;
      default:
         break;
      }
   }

   private double get(XBoxOneMapping channel)
   {
      return channels.get(channel);
   }
}
