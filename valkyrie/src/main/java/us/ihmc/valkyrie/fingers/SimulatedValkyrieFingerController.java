package us.ihmc.valkyrie.fingers;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;

import controller_msgs.msg.dds.HandDesiredConfigurationMessage;
import controller_msgs.msg.dds.HandJointAnglePacket;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ROS2Tools.MessageTopicNameGenerator;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandJointName;
import us.ihmc.humanoidRobotics.communication.subscribers.HandDesiredConfigurationMessageSubscriber;
import us.ihmc.robotics.partNames.FingerName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.RealtimeRos2Node;
import us.ihmc.simulationConstructionSetTools.robotController.MultiThreadedRobotControlElement;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.tools.thread.CloseableAndDisposableRegistry;
import us.ihmc.wholeBodyController.concurrent.ThreadDataSynchronizerInterface;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoLong;

public class SimulatedValkyrieFingerController implements MultiThreadedRobotControlElement, RobotController
{
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);

   private final YoLong lastEstimatorStartTime = new YoLong("nextExecutionTime", registry);

   private final long controlDTInNS;
   private final long estimatorDTInNS;

   private final ThreadDataSynchronizerInterface threadDataSynchronizer;
   private final YoDouble handControllerTime = new YoDouble("handControllerTime", registry);
   private final SimulatedValkyrieFingerJointAngleProducer jointAngleProducer;

   // For ValkyrieFingerController, replace ValkyrieHandJointName to ValkyrieFingerMotorName.
   private final SideDependentList<EnumMap<ValkyrieHandJointName, DoubleProvider>> sideDependentControlSpaceHandlers = SideDependentList.createListOfEnumMaps(ValkyrieHandJointName.class);
   private final SideDependentList<EnumMap<ValkyrieHandJointName, YoDouble>> closedAngles = SideDependentList.createListOfEnumMaps(ValkyrieHandJointName.class);
   private final SideDependentList<EnumMap<ValkyrieHandJointName, YoDouble>> openedAngles = SideDependentList.createListOfEnumMaps(ValkyrieHandJointName.class);
   private final double delayTime = 2.0;
   private final double trajectoryTime = 3.0;

   private final SideDependentList<HandDesiredConfigurationMessageSubscriber> handDesiredConfigurationMessageSubscribers = new SideDependentList<>();

   private final SideDependentList<ProposedValkyrieFingerSetController> fingerSetControllers = new SideDependentList<>();

   private final SideDependentList<List<OneDegreeOfFreedomJoint>> allFingerJoints = new SideDependentList<>();

   public SimulatedValkyrieFingerController(FloatingRootJointRobot simulatedRobot, ThreadDataSynchronizerInterface threadDataSynchronizer,
                                            RealtimeRos2Node realtimeRos2Node, CloseableAndDisposableRegistry closeableAndDisposableRegistry,
                                            DRCRobotModel robotModel, MessageTopicNameGenerator pubTopicNameGenerator,
                                            MessageTopicNameGenerator subTopicNameGenerator)
   {
      this.threadDataSynchronizer = threadDataSynchronizer;
      this.controlDTInNS = Conversions.secondsToNanoseconds(robotModel.getControllerDT());
      this.estimatorDTInNS = Conversions.secondsToNanoseconds(robotModel.getEstimatorDT());

      if (realtimeRos2Node != null)
      {
         IHMCRealtimeROS2Publisher<HandJointAnglePacket> jointAnglePublisher = ROS2Tools.createPublisher(realtimeRos2Node, HandJointAnglePacket.class,
                                                                                                         pubTopicNameGenerator);
         jointAngleProducer = new SimulatedValkyrieFingerJointAngleProducer(jointAnglePublisher, simulatedRobot, closeableAndDisposableRegistry);
      }
      else
      {
         jointAngleProducer = null;
      }

      // ValkyrieFingerSetController
      for (RobotSide robotSide : RobotSide.values)
      {
         for (ValkyrieHandJointName valkyrieHandJointName : ValkyrieHandJointName.values())
         {
            sideDependentControlSpaceHandlers.get(robotSide).put(valkyrieHandJointName, new DoubleProvider()
            {
               @Override
               public double getValue()
               {
                  return simulatedRobot.getOneDegreeOfFreedomJoint(valkyrieHandJointName.getJointName(robotSide)).getQ();
               }
            });
         }
         ProposedValkyrieFingerSetController<ValkyrieHandJointName> fingerSetController = new ProposedValkyrieFingerSetController<ValkyrieHandJointName>(robotSide,
                                                                                                                                                         handControllerTime,
                                                                                                                                                         sideDependentControlSpaceHandlers.get(robotSide),
                                                                                                                                                         registry);
         fingerSetControllers.put(robotSide, fingerSetController);
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         allFingerJoints.put(robotSide, new ArrayList<OneDegreeOfFreedomJoint>());
         for (HandJointName jointEnum : robotModel.getHandModel().getHandJointNames())
         {
            OneDegreeOfFreedomJoint fingerJoint = simulatedRobot.getOneDegreeOfFreedomJoint(jointEnum.getJointName(robotSide));
            allFingerJoints.get(robotSide).add(fingerJoint);
         }
      }

      // Subscribers
      for (RobotSide robotSide : RobotSide.values)
      {
         HandDesiredConfigurationMessageSubscriber handDesiredConfigurationSubscriber = new HandDesiredConfigurationMessageSubscriber(robotSide);
         handDesiredConfigurationMessageSubscribers.put(robotSide, handDesiredConfigurationSubscriber);
         if (realtimeRos2Node != null)
         {
            ROS2Tools.createCallbackSubscription(realtimeRos2Node, HandDesiredConfigurationMessage.class, subTopicNameGenerator,
                                                 handDesiredConfigurationSubscriber);
         }
      }
      constructDesiredHandFingerJointMap();
   }

   private void constructDesiredHandFingerJointMap()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         openedAngles.put(robotSide, new EnumMap<>(ValkyrieHandJointName.class));
         closedAngles.put(robotSide, new EnumMap<>(ValkyrieHandJointName.class));
         for (ValkyrieHandJointName jointEnum : ValkyrieHandJointName.values)
         {
            String jointName = jointEnum.getJointName(robotSide);
            YoDouble openedAngle = new YoDouble(jointName + "OpenedAngle", registry);
            openedAngle.set(ValkyrieFingerControlParameters.getOpenedDesiredHandJointDefinition(robotSide).get(jointEnum));
            openedAngles.get(robotSide).put(jointEnum, openedAngle);
            YoDouble closedAngle = new YoDouble(jointName + "ClosedAngle", registry);
            closedAngle.set(ValkyrieFingerControlParameters.getClosedDesiredHandJointDefinition(robotSide).get(jointEnum));
            closedAngles.get(robotSide).put(jointEnum, closedAngle);
         }
      }
   }

   @Override
   public void initialize()
   {

   }

   @Override
   public void read(long currentClockTime)
   {
      long timestamp;
      if (threadDataSynchronizer != null)
      {
         timestamp = threadDataSynchronizer.getTimestamp();
         handControllerTime.set(Conversions.nanosecondsToSeconds(timestamp));
      }
      else
      {
         handControllerTime.add(Conversions.nanosecondsToSeconds(controlDTInNS));
      }

      if (jointAngleProducer != null)
      {
         jointAngleProducer.sendHandJointAnglesPacket();
      }
   }

   @Override
   public void run()
   {
      checkForNewHandDesiredConfigurationRequested();

      for (RobotSide robotSide : RobotSide.values)
      {
         fingerSetControllers.get(robotSide).doControl();
      }
   }

   @Override
   public void write(long timestamp)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         List<OneDegreeOfFreedomJoint> oneSideFingerJoints = allFingerJoints.get(robotSide);
         for (int i = 0; i < oneSideFingerJoints.size(); i++)
         {
            OneDegreeOfFreedomJoint joint = oneSideFingerJoints.get(i);
            double desiredQ = fingerSetControllers.get(robotSide).getDesired(joint.getName());
            joint.getQYoVariable().set(desiredQ);
         }
      }
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public String getName()
   {
      return name;
   }

   @Override
   public YoGraphicsListRegistry getYoGraphicsListRegistry()
   {
      return null;
   }

   @Override
   public long nextWakeupTime()
   {
      if (lastEstimatorStartTime.getLongValue() == Long.MIN_VALUE)
      {
         return Long.MIN_VALUE;
      }
      else
      {
         return lastEstimatorStartTime.getLongValue() + controlDTInNS + estimatorDTInNS;
      }
   }

   @Override
   public String getDescription()
   {
      return registry.getName();
   }

   @Override
   public void doControl()
   {
      read(0L);
      run();
      write(0L);
   }

   private void checkForNewHandDesiredConfigurationRequested()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         if (handDesiredConfigurationMessageSubscribers.get(robotSide).isNewDesiredConfigurationAvailable())
         {
            HandConfiguration handDesiredConfiguration = HandConfiguration.fromByte(handDesiredConfigurationMessageSubscribers.get(robotSide).pollMessage()
                                                                                                                              .getDesiredHandConfiguration());
            System.out.println("handDesiredConfiguration " + robotSide + " " + handDesiredConfiguration);
            switch (handDesiredConfiguration)
            {
            case CLOSE:
               EnumMap<ValkyrieHandJointName, YoDouble> closedAnglesMap = closedAngles.get(robotSide);

               for (ValkyrieHandJointName handJointName : ValkyrieHandJointName.values)
               {
                  double delayTime;
                  if (handJointName.getFingerName() == FingerName.THUMB)
                     delayTime = this.delayTime;
                  else
                     delayTime = 0.0;
                  fingerSetControllers.get(robotSide).setDesired(handJointName.toString(), trajectoryTime, delayTime,
                                                                 closedAnglesMap.get(handJointName).getDoubleValue());
               }

               break;

            case OPEN:
               EnumMap<ValkyrieHandJointName, YoDouble> opendAnglesMap = openedAngles.get(robotSide);

               for (ValkyrieHandJointName handJointName : ValkyrieHandJointName.values)
               {
                  double delayTime;
                  if (handJointName.getFingerName() == FingerName.THUMB)
                     delayTime = 0.0;
                  else
                     delayTime = this.delayTime;
                  fingerSetControllers.get(robotSide).setDesired(handJointName.toString(), trajectoryTime, delayTime,
                                                                 opendAnglesMap.get(handJointName).getDoubleValue());
               }
               break;
            default:

               break;
            }
         }
      }
   }
}