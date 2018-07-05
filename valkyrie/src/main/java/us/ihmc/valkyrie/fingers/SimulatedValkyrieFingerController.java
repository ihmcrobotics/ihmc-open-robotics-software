package us.ihmc.valkyrie.fingers;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;

import controller_msgs.msg.dds.HandDesiredConfigurationMessage;
import controller_msgs.msg.dds.HandFingerTrajectoryMessage;
import controller_msgs.msg.dds.HandJointAnglePacket;
import controller_msgs.msg.dds.OneDoFJointTrajectoryMessage;
import controller_msgs.msg.dds.TrajectoryPoint1DMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ROS2Tools.MessageTopicNameGenerator;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandJointName;
import us.ihmc.humanoidRobotics.communication.subscribers.HandDesiredConfigurationMessageSubscriber;
import us.ihmc.humanoidRobotics.communication.subscribers.HandFingerTrajectoryMessageSubscriber;
import us.ihmc.idl.IDLSequence.Object;
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

   private final SideDependentList<EnumMap<ValkyrieHandJointName, YoDouble>> sideDependentHandJointHandlers = SideDependentList.createListOfEnumMaps(ValkyrieHandJointName.class);
   private final SideDependentList<EnumMap<ValkyrieHandJointName, YoDouble>> closedHandJointAngles = SideDependentList.createListOfEnumMaps(ValkyrieHandJointName.class);
   private final SideDependentList<EnumMap<ValkyrieHandJointName, YoDouble>> openedHandJointAngles = SideDependentList.createListOfEnumMaps(ValkyrieHandJointName.class);
   private final SideDependentList<EnumMap<ValkyrieFingerMotorName, YoDouble>> sideDependentFingerMotorHandlers = SideDependentList.createListOfEnumMaps(ValkyrieFingerMotorName.class);
   private final SideDependentList<EnumMap<ValkyrieFingerMotorName, YoDouble>> closedFingerMotorAngles = SideDependentList.createListOfEnumMaps(ValkyrieFingerMotorName.class);
   private final SideDependentList<EnumMap<ValkyrieFingerMotorName, YoDouble>> openedFingerMotorAngles = SideDependentList.createListOfEnumMaps(ValkyrieFingerMotorName.class);

   private final double delayTime = 1.0;
   private final double trajectoryTime = 1.0;

   private final SideDependentList<HandDesiredConfigurationMessageSubscriber> handDesiredConfigurationMessageSubscribers = new SideDependentList<>();
   private final SideDependentList<HandFingerTrajectoryMessageSubscriber> handFingerTrajectoryMessageSubscribers = new SideDependentList<>();

   private final SideDependentList<ProposedValkyrieFingerSetController<ValkyrieHandJointName>> handJointFingerSetControllers = new SideDependentList<>();
   private final SideDependentList<ProposedValkyrieFingerSetController<ValkyrieFingerMotorName>> fingerSetControllers = new SideDependentList<>();

   // to put desired value.
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
         jointAngleProducer = null;

      /*
       * FingerSetController 1: ValkyrieHandJointName
       */
      for (RobotSide robotSide : RobotSide.values)
      {
         for (ValkyrieHandJointName valkyrieHandJointName : ValkyrieHandJointName.values())
         {
            YoDouble handJointHandler = new YoDouble(valkyrieHandJointName.getJointName(robotSide) + "_handJointHandler", registry);
            double currentHandJoint = simulatedRobot.getOneDegreeOfFreedomJoint(valkyrieHandJointName.getJointName(robotSide)).getQ();
            handJointHandler.set(currentHandJoint);
            sideDependentHandJointHandlers.get(robotSide).put(valkyrieHandJointName, handJointHandler);
         }
         ProposedValkyrieFingerSetController<ValkyrieHandJointName> handJointFingerSetController = new ProposedValkyrieFingerSetController<ValkyrieHandJointName>(ValkyrieHandJointName.class.getSimpleName(),
                                                                                                                                                                  robotSide,
                                                                                                                                                                  handControllerTime,
                                                                                                                                                                  sideDependentHandJointHandlers.get(robotSide),
                                                                                                                                                                  registry);
         handJointFingerSetControllers.put(robotSide, handJointFingerSetController);
      }

      /*
       * FingerSetController 2: ValkyrieFingerMotorName
       */
      for (RobotSide robotSide : RobotSide.values)
      {
         for (ValkyrieFingerMotorName valkyrieFingerMotorName : ValkyrieFingerMotorName.values())
         {
            YoDouble fingerMotorHandler = new YoDouble(valkyrieFingerMotorName.getJointName(robotSide) + "_fingerMotorHandler", registry);
            double currentFingerMotor = 0.0; // TODO : use ValkyrieRosControlFingerStateEstimator 
            fingerMotorHandler.set(currentFingerMotor);
            sideDependentFingerMotorHandlers.get(robotSide).put(valkyrieFingerMotorName, fingerMotorHandler);
         }
         ProposedValkyrieFingerSetController<ValkyrieFingerMotorName> fingerSetController = new ProposedValkyrieFingerSetController<ValkyrieFingerMotorName>(ValkyrieFingerMotorName.class.getSimpleName(),
                                                                                                                                                             robotSide,
                                                                                                                                                             handControllerTime,
                                                                                                                                                             sideDependentFingerMotorHandlers.get(robotSide),
                                                                                                                                                             registry);
         fingerSetControllers.put(robotSide, fingerSetController);
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         allFingerJoints.put(robotSide, new ArrayList<OneDegreeOfFreedomJoint>());
         for (HandJointName jointEnum : robotModel.getHandModel().getHandJointNames())
            allFingerJoints.get(robotSide).add(simulatedRobot.getOneDegreeOfFreedomJoint(jointEnum.getJointName(robotSide)));
      }

      // Subscribers
      for (RobotSide robotSide : RobotSide.values)
      {
         HandDesiredConfigurationMessageSubscriber handDesiredConfigurationSubscriber = new HandDesiredConfigurationMessageSubscriber(robotSide);
         handDesiredConfigurationMessageSubscribers.put(robotSide, handDesiredConfigurationSubscriber);

         HandFingerTrajectoryMessageSubscriber handFingerTrajectoryMessageSubscriber = new HandFingerTrajectoryMessageSubscriber(robotSide);
         handFingerTrajectoryMessageSubscribers.put(robotSide, handFingerTrajectoryMessageSubscriber);
         if (realtimeRos2Node != null)
         {
            ROS2Tools.createCallbackSubscription(realtimeRos2Node, HandDesiredConfigurationMessage.class, subTopicNameGenerator,
                                                 handDesiredConfigurationSubscriber);
            ROS2Tools.createCallbackSubscription(realtimeRos2Node, HandFingerTrajectoryMessage.class, subTopicNameGenerator,
                                                 handFingerTrajectoryMessageSubscriber);
         }
      }
      constructDesiredHandFingerJointMap();
   }

   private void constructDesiredHandFingerJointMap()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         openedHandJointAngles.put(robotSide, new EnumMap<>(ValkyrieHandJointName.class));
         closedHandJointAngles.put(robotSide, new EnumMap<>(ValkyrieHandJointName.class));
         for (ValkyrieHandJointName jointEnum : ValkyrieHandJointName.values)
         {
            String jointName = jointEnum.getJointName(robotSide);
            YoDouble openedAngle = new YoDouble(jointName + "OpenedAngle", registry);
            openedAngle.set(ValkyrieFingerControlParameters.getOpenedDesiredHandJointDefinition(robotSide).get(jointEnum));
            openedHandJointAngles.get(robotSide).put(jointEnum, openedAngle);
            YoDouble closedAngle = new YoDouble(jointName + "ClosedAngle", registry);
            closedAngle.set(ValkyrieFingerControlParameters.getClosedDesiredHandJointDefinition(robotSide).get(jointEnum));
            closedHandJointAngles.get(robotSide).put(jointEnum, closedAngle);
         }

         openedFingerMotorAngles.put(robotSide, new EnumMap<>(ValkyrieFingerMotorName.class));
         closedFingerMotorAngles.put(robotSide, new EnumMap<>(ValkyrieFingerMotorName.class));
         for (ValkyrieFingerMotorName jointEnum : ValkyrieFingerMotorName.values)
         {
            String jointName = jointEnum.getJointName(robotSide);
            YoDouble openedAngle = new YoDouble(jointName + "OpenedAngle", registry);
            openedAngle.set(ValkyrieFingerControlParameters.getOpenDesiredDefinition(robotSide).get(jointEnum));
            openedFingerMotorAngles.get(robotSide).put(jointEnum, openedAngle);
            YoDouble closedAngle = new YoDouble(jointName + "ClosedAngle", registry);
            closedAngle.set(ValkyrieFingerControlParameters.getClosedDesiredDefinition(robotSide).get(jointEnum));
            closedFingerMotorAngles.get(robotSide).put(jointEnum, closedAngle);
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
      checkForNewHandFingerTrajectoryRequested();

      for (RobotSide robotSide : RobotSide.values)
      {
         handJointFingerSetControllers.get(robotSide).doControl();
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
            double desiredQ = handJointFingerSetControllers.get(robotSide).getDesired(joint.getName());
            joint.getQYoVariable().set(desiredQ);
         }
         EnumMap<ValkyrieFingerMotorName, YoDouble> enumMap = sideDependentFingerMotorHandlers.get(robotSide);
         for (ValkyrieFingerMotorName valkyrieFingerMotorName : ValkyrieFingerMotorName.values)
         {
            double desiredQ = fingerSetControllers.get(robotSide).getDesired(valkyrieFingerMotorName.getJointName(robotSide));
            enumMap.get(valkyrieFingerMotorName).set(desiredQ);
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
               EnumMap<ValkyrieHandJointName, YoDouble> closedHandJointAnglesMap = closedHandJointAngles.get(robotSide);

               for (ValkyrieHandJointName handJointName : ValkyrieHandJointName.values)
               {
                  double delayTime;
                  if (handJointName.getFingerName() == FingerName.THUMB)
                     delayTime = this.delayTime;
                  else
                     delayTime = 0.0;
                  handJointFingerSetControllers.get(robotSide).setDesired(handJointName.toString(), trajectoryTime, delayTime,
                                                                          closedHandJointAnglesMap.get(handJointName).getDoubleValue());
               }

               EnumMap<ValkyrieFingerMotorName, YoDouble> closedAnglesMap = closedFingerMotorAngles.get(robotSide);
               for (ValkyrieFingerMotorName fingerMotorName : ValkyrieFingerMotorName.values)
               {
                  double delayTime;
                  if (fingerMotorName.getFingerName() == FingerName.THUMB)
                     delayTime = this.delayTime;
                  else
                     delayTime = 0.0;
                  fingerSetControllers.get(robotSide).setDesired(fingerMotorName.toString(), trajectoryTime, delayTime,
                                                                 closedAnglesMap.get(fingerMotorName).getDoubleValue());
               }

               break;

            case OPEN:
               EnumMap<ValkyrieHandJointName, YoDouble> opendHandJointAnglesMap = openedHandJointAngles.get(robotSide);

               for (ValkyrieHandJointName handJointName : ValkyrieHandJointName.values)
               {
                  double delayTime;
                  if (handJointName.getFingerName() == FingerName.THUMB)
                     delayTime = 0.0;
                  else
                     delayTime = this.delayTime;
                  handJointFingerSetControllers.get(robotSide).setDesired(handJointName.toString(), trajectoryTime, delayTime,
                                                                          opendHandJointAnglesMap.get(handJointName).getDoubleValue());
               }

               EnumMap<ValkyrieFingerMotorName, YoDouble> openedAnglesMap = openedFingerMotorAngles.get(robotSide);
               for (ValkyrieFingerMotorName fingerMotorName : ValkyrieFingerMotorName.values)
               {
                  double delayTime;
                  if (fingerMotorName.getFingerName() == FingerName.THUMB)
                     delayTime = 0.0;
                  else
                     delayTime = this.delayTime;
                  fingerSetControllers.get(robotSide).setDesired(fingerMotorName.toString(), trajectoryTime, delayTime,
                                                                 openedAnglesMap.get(fingerMotorName).getDoubleValue());
               }
               break;
            default:

               break;
            }
         }
      }
   }

   private void checkForNewHandFingerTrajectoryRequested()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         if (handFingerTrajectoryMessageSubscribers.get(robotSide).isNewDesiredConfigurationAvailable())
         {
            EnumMap<ValkyrieFingerMotorName, Double> desiredPositionMap = new EnumMap<>(ValkyrieFingerMotorName.class);
            EnumMap<ValkyrieFingerMotorName, Double> trajectoryTimeMap = new EnumMap<>(ValkyrieFingerMotorName.class);
            EnumMap<ValkyrieFingerMotorName, Double> delayTimeMap = new EnumMap<>(ValkyrieFingerMotorName.class);

            HandFingerTrajectoryMessage handFingerTrajectoryMessage = handFingerTrajectoryMessageSubscribers.get(robotSide).pollMessage();
            ValkyrieFingerMotorName[] valkyrieFingerMotorNames = ValkyrieFingerMotorName.values;
            Object<OneDoFJointTrajectoryMessage> jointTrajectoryMessages = handFingerTrajectoryMessage.getJointTrajectoryMessages();
            for (int i = 0; i < jointTrajectoryMessages.size(); i++)
            {
               Object<TrajectoryPoint1DMessage> trajectoryPoints = jointTrajectoryMessages.get(i).getTrajectoryPoints();

               desiredPositionMap.put(valkyrieFingerMotorNames[i], trajectoryPoints.get(0).getPosition());
               trajectoryTimeMap.put(valkyrieFingerMotorNames[i], trajectoryPoints.get(0).getTime());
               delayTimeMap.put(valkyrieFingerMotorNames[i], handFingerTrajectoryMessage.getListQueueingProperties().get(i).getExecutionDelayTime());
            }

            for (ValkyrieFingerMotorName fingerMotorName : ValkyrieFingerMotorName.values)
            {
               fingerSetControllers.get(robotSide).setDesired(fingerMotorName.getJointName(robotSide), trajectoryTimeMap.get(fingerMotorName),
                                                              delayTimeMap.get(fingerMotorName), desiredPositionMap.get(fingerMotorName));
            }
         }
      }
   }
}