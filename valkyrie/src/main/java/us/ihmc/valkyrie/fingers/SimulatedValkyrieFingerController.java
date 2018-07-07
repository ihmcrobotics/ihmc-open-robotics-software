package us.ihmc.valkyrie.fingers;

import java.util.EnumMap;

import controller_msgs.msg.dds.HandDesiredConfigurationMessage;
import controller_msgs.msg.dds.HandJointAnglePacket;
import controller_msgs.msg.dds.OneDoFJointTrajectoryMessage;
import controller_msgs.msg.dds.TrajectoryPoint1DMessage;
import controller_msgs.msg.dds.ValkyrieHandFingerTrajectoryMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.PrintTools;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ROS2Tools.MessageTopicNameGenerator;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.humanoidRobotics.communication.subscribers.HandDesiredConfigurationMessageSubscriber;
import us.ihmc.humanoidRobotics.communication.subscribers.ValkyrieHandFingerTrajectoryMessageSubscriber;
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

   private final double delayTime = 0.3;
   private final double trajectoryTime = 0.7;

   /*
    * The handlers corresponds to state estimators in Valkyrie ros controller.
    */
   private final SideDependentList<EnumMap<ValkyrieHandJointName, YoDouble>> sideDependentHandJointHandlers = SideDependentList.createListOfEnumMaps(ValkyrieHandJointName.class);
   private final SideDependentList<EnumMap<ValkyrieFingerMotorName, YoDouble>> sideDependentFingerMotorHandlers = SideDependentList.createListOfEnumMaps(ValkyrieFingerMotorName.class);

   private final SideDependentList<HandDesiredConfigurationMessageSubscriber> handDesiredConfigurationMessageSubscribers = new SideDependentList<>();
   private final SideDependentList<ValkyrieHandFingerTrajectoryMessageSubscriber> valkyrieHandFingerTrajectoryMessageSubscribers = new SideDependentList<>();

   private final SideDependentList<ProposedValkyrieFingerSetController<ValkyrieHandJointName>> handJointFingerSetControllers = new SideDependentList<>();
   private final SideDependentList<ProposedValkyrieFingerSetController<ValkyrieFingerMotorName>> fingerSetControllers = new SideDependentList<>();

   private final SideDependentList<EnumMap<ValkyrieHandJointName, OneDegreeOfFreedomJoint>> allHandJointsNameToJointsMap = SideDependentList.createListOfEnumMaps(ValkyrieHandJointName.class);
   private final SideDependentList<EnumMap<ValkyrieHandJointName, YoDouble>> currentHandJointAngles = SideDependentList.createListOfEnumMaps(ValkyrieHandJointName.class);

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
         ProposedValkyrieFingerSetController<ValkyrieHandJointName> handJointFingerSetController = new ProposedValkyrieFingerSetController<ValkyrieHandJointName>(ValkyrieHandJointName.class,
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
         ProposedValkyrieFingerSetController<ValkyrieFingerMotorName> fingerSetController = new ProposedValkyrieFingerSetController<ValkyrieFingerMotorName>(ValkyrieFingerMotorName.class,
                                                                                                                                                             robotSide,
                                                                                                                                                             handControllerTime,
                                                                                                                                                             sideDependentFingerMotorHandlers.get(robotSide),
                                                                                                                                                             registry);
         fingerSetControllers.put(robotSide, fingerSetController);
      }

      // to simulate finger joint
      for (RobotSide robotSide : RobotSide.values)
      {
         for (ValkyrieHandJointName valkyrieHandJointName : ValkyrieHandJointName.values())
         {
            String jointName = valkyrieHandJointName.getJointName(robotSide);
            OneDegreeOfFreedomJoint oneDegreeOfFreedomJoint = simulatedRobot.getOneDegreeOfFreedomJoint(jointName);
            allHandJointsNameToJointsMap.get(robotSide).put(valkyrieHandJointName, oneDegreeOfFreedomJoint);
            YoDouble currentHandJointAngle = new YoDouble("q_current_" + jointName, registry);
            currentHandJointAngles.get(robotSide).put(valkyrieHandJointName, currentHandJointAngle);
         }
      }

      // Subscribers
      for (RobotSide robotSide : RobotSide.values)
      {
         HandDesiredConfigurationMessageSubscriber handDesiredConfigurationSubscriber = new HandDesiredConfigurationMessageSubscriber(robotSide);
         handDesiredConfigurationMessageSubscribers.put(robotSide, handDesiredConfigurationSubscriber);

         ValkyrieHandFingerTrajectoryMessageSubscriber valkyrieHandFingerTrajectoryMessageSubscriber = new ValkyrieHandFingerTrajectoryMessageSubscriber(robotSide);
         valkyrieHandFingerTrajectoryMessageSubscribers.put(robotSide, valkyrieHandFingerTrajectoryMessageSubscriber);
         if (realtimeRos2Node != null)
         {
            ROS2Tools.createCallbackSubscription(realtimeRos2Node, HandDesiredConfigurationMessage.class, subTopicNameGenerator,
                                                 handDesiredConfigurationSubscriber);
            ROS2Tools.createCallbackSubscription(realtimeRos2Node, ValkyrieHandFingerTrajectoryMessage.class, subTopicNameGenerator,
                                                 valkyrieHandFingerTrajectoryMessageSubscriber);
         }
      }
   }

   @Override
   public void initialize()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         for (ValkyrieHandJointName valkyrieHandJointName : ValkyrieHandJointName.values())
         {
            OneDegreeOfFreedomJoint oneDegreeOfFreedomJoint = allHandJointsNameToJointsMap.get(robotSide).get(valkyrieHandJointName);
            currentHandJointAngles.get(robotSide).get(valkyrieHandJointName).set(oneDegreeOfFreedomJoint.getQ());
         }
      }
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
      checkForNewValkyrieHandFingerTrajectoryRequested();

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
         for (ValkyrieHandJointName valkyrieHandJointName : ValkyrieHandJointName.values())
         {
            double desiredQ = handJointFingerSetControllers.get(robotSide).getDesired(valkyrieHandJointName);

            OneDegreeOfFreedomJoint oneDegreeOfFreedomJoint = allHandJointsNameToJointsMap.get(robotSide).get(valkyrieHandJointName);
            oneDegreeOfFreedomJoint.getQYoVariable().set(desiredQ);
            YoDouble currentHandJointAngle = currentHandJointAngles.get(robotSide).get(valkyrieHandJointName);
            currentHandJointAngle.set(oneDegreeOfFreedomJoint.getQ());
         }

         EnumMap<ValkyrieHandJointName, YoDouble> handJointEnumMap = sideDependentHandJointHandlers.get(robotSide);
         for (ValkyrieHandJointName valkyrieHandJointName : ValkyrieHandJointName.values)
         {
            double desiredQ = handJointFingerSetControllers.get(robotSide).getDesired(valkyrieHandJointName);
            handJointEnumMap.get(valkyrieHandJointName).set(desiredQ);
         }
         EnumMap<ValkyrieFingerMotorName, YoDouble> fingerMotorEnumMap = sideDependentFingerMotorHandlers.get(robotSide);
         for (ValkyrieFingerMotorName valkyrieFingerMotorName : ValkyrieFingerMotorName.values)
         {
            double desiredQ = fingerSetControllers.get(robotSide).getDesired(valkyrieFingerMotorName);
            fingerMotorEnumMap.get(valkyrieFingerMotorName).set(desiredQ);
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
            switch (handDesiredConfiguration)
            {
            case CLOSE:
               for (ValkyrieHandJointName handJointName : ValkyrieHandJointName.values)
               {
                  double delayTime;
                  if (handJointName.getFingerName() == FingerName.THUMB)
                     delayTime = this.delayTime;
                  else
                     delayTime = 0.0;
                  handJointFingerSetControllers.get(robotSide).setDesired(handJointName, trajectoryTime, delayTime,
                                                                          ValkyrieFingerControlParameters.getDesiredHandJoint(robotSide, handJointName, 1.0));
               }

               for (ValkyrieFingerMotorName fingerMotorName : ValkyrieFingerMotorName.values)
               {
                  double delayTime;
                  if (fingerMotorName.getFingerName() == FingerName.THUMB)
                     delayTime = this.delayTime;
                  else
                     delayTime = 0.0;
                  fingerSetControllers.get(robotSide).setDesired(fingerMotorName, trajectoryTime, delayTime,
                                                                 ValkyrieFingerControlParameters.getDesiredFingerMotor(robotSide, fingerMotorName, 1.0));
               }

               break;

            case OPEN:
               for (ValkyrieHandJointName handJointName : ValkyrieHandJointName.values)
               {
                  double delayTime;
                  if (handJointName.getFingerName() == FingerName.THUMB)
                     delayTime = 0.0;
                  else
                     delayTime = this.delayTime;
                  handJointFingerSetControllers.get(robotSide).setDesired(handJointName, trajectoryTime, delayTime,
                                                                          ValkyrieFingerControlParameters.getDesiredHandJoint(robotSide, handJointName, 0.0));
               }
               for (ValkyrieFingerMotorName fingerMotorName : ValkyrieFingerMotorName.values)
               {
                  double delayTime;
                  if (fingerMotorName.getFingerName() == FingerName.THUMB)
                     delayTime = 0.0;
                  else
                     delayTime = this.delayTime;
                  fingerSetControllers.get(robotSide).setDesired(fingerMotorName, trajectoryTime, delayTime,
                                                                 ValkyrieFingerControlParameters.getDesiredFingerMotor(robotSide, fingerMotorName, 0.0));
               }
               break;

            case STOP:
               for (ValkyrieHandJointName handJointName : ValkyrieHandJointName.values)
                  handJointFingerSetControllers.get(robotSide).setStop(handJointName,
                                                                       sideDependentHandJointHandlers.get(robotSide).get(handJointName).getDoubleValue());
               for (ValkyrieFingerMotorName fingerMotorName : ValkyrieFingerMotorName.values)
                  fingerSetControllers.get(robotSide).setStop(fingerMotorName,
                                                              sideDependentFingerMotorHandlers.get(robotSide).get(fingerMotorName).getDoubleValue());
               break;

            default:

               break;
            }
            handJointFingerSetControllers.get(robotSide).executeTrajectories();
            fingerSetControllers.get(robotSide).executeTrajectories();
         }
      }
   }

   private void checkForNewValkyrieHandFingerTrajectoryRequested()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         if (valkyrieHandFingerTrajectoryMessageSubscribers.get(robotSide).isNewDesiredConfigurationAvailable())
         {
            ValkyrieHandFingerTrajectoryMessage handFingerTrajectoryMessage = valkyrieHandFingerTrajectoryMessageSubscribers.get(robotSide).pollMessage();
            PrintTools.info("ValkyrieHandFingerTrajectoryMessage " + robotSide);

            for (ValkyrieFingerMotorName fingerMotorName : ValkyrieFingerMotorName.values)
            {
               int indexOfTrajectory = hasTrajectory(handFingerTrajectoryMessage.getFingerMotorNames(), fingerMotorName);

               if (indexOfTrajectory == -1)
               {
                  fingerSetControllers.get(robotSide).setStop(fingerMotorName,
                                                              sideDependentFingerMotorHandlers.get(robotSide).get(fingerMotorName).getDoubleValue());
                  setEstimatedStopedValueOnHandJointFingerSetController(robotSide, fingerMotorName);
               }
               else
               {
                  Object<OneDoFJointTrajectoryMessage> jointTrajectoryMessages = handFingerTrajectoryMessage.getJointspaceTrajectory()
                                                                                                            .getJointTrajectoryMessages();
                  Object<TrajectoryPoint1DMessage> trajectoryPoints = jointTrajectoryMessages.get(indexOfTrajectory).getTrajectoryPoints();
                  fingerSetControllers.get(robotSide).setDesired(fingerMotorName, trajectoryPoints.get(0).getTime(),
                                                                 handFingerTrajectoryMessage.getDelayTimes().get(indexOfTrajectory),
                                                                 ValkyrieFingerControlParameters.getDesiredFingerMotor(robotSide, fingerMotorName,
                                                                                                                       trajectoryPoints.get(0).getPosition()));
                  setEstimatedDesiredValueOnHandJointFingerSetController(robotSide, fingerMotorName, handFingerTrajectoryMessage);
               }
            }
            handJointFingerSetControllers.get(robotSide).executeTrajectories();
            fingerSetControllers.get(robotSide).executeTrajectories();
         }
      }
   }

   private int hasTrajectory(us.ihmc.idl.IDLSequence.Byte namesInMessage, ValkyrieFingerMotorName fingerMotorName)
   {
      for (int i = 0; i < namesInMessage.size(); i++)
         if (fingerMotorName == ValkyrieFingerMotorName.fromByte(namesInMessage.get(i)))
            return i;

      return -1;
   }

   private void setEstimatedDesiredValueOnHandJointFingerSetController(RobotSide robotSide, ValkyrieFingerMotorName fingerMotorName,
                                                                       ValkyrieHandFingerTrajectoryMessage handFingerTrajectoryMessage)
   {
      int numberOfHandJoints = 3;
      if (fingerMotorName == ValkyrieFingerMotorName.ThumbMotorRoll)
         numberOfHandJoints = 1;
      for (int i = 1; i <= numberOfHandJoints; i++)
      {
         ValkyrieHandJointName handJointName = fingerMotorName.getCorrespondingJointName(i);

         int indexOfTrajectory = hasTrajectory(handFingerTrajectoryMessage.getFingerMotorNames(), fingerMotorName);

         Object<OneDoFJointTrajectoryMessage> jointTrajectoryMessages = handFingerTrajectoryMessage.getJointspaceTrajectory().getJointTrajectoryMessages();
         Object<TrajectoryPoint1DMessage> trajectoryPoints = jointTrajectoryMessages.get(indexOfTrajectory).getTrajectoryPoints();

         handJointFingerSetControllers.get(robotSide).setDesired(handJointName, trajectoryPoints.get(0).getTime(),
                                                                 handFingerTrajectoryMessage.getDelayTimes().get(indexOfTrajectory),
                                                                 ValkyrieFingerControlParameters.getDesiredHandJoint(robotSide, handJointName,
                                                                                                                     trajectoryPoints.get(0).getPosition()));
      }
   }

   private void setEstimatedStopedValueOnHandJointFingerSetController(RobotSide robotSide, ValkyrieFingerMotorName fingerMotorName)
   {
      int numberOfHandJoints = 3;
      if (fingerMotorName == ValkyrieFingerMotorName.ThumbMotorRoll)
         numberOfHandJoints = 1;
      for (int i = 1; i <= numberOfHandJoints; i++)
      {
         ValkyrieHandJointName handJointName = fingerMotorName.getCorrespondingJointName(i);

         handJointFingerSetControllers.get(robotSide).setStop(handJointName,
                                                              sideDependentFingerMotorHandlers.get(robotSide).get(fingerMotorName).getDoubleValue());
      }
   }
}