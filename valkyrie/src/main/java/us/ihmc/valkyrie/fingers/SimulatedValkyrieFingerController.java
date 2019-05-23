package us.ihmc.valkyrie.fingers;

import java.util.EnumMap;

import controller_msgs.msg.dds.HandDesiredConfigurationMessage;
import controller_msgs.msg.dds.HandJointAnglePacket;
import controller_msgs.msg.dds.OneDoFJointTrajectoryMessage;
import controller_msgs.msg.dds.TrajectoryPoint1DMessage;
import controller_msgs.msg.dds.ValkyrieHandFingerTrajectoryMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.factory.SimulatedHandControlTask;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextData;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ROS2Tools.MessageTopicNameGenerator;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.humanoidRobotics.communication.subscribers.HandDesiredConfigurationMessageSubscriber;
import us.ihmc.humanoidRobotics.communication.subscribers.ValkyrieHandFingerTrajectoryMessageSubscriber;
import us.ihmc.idl.IDLSequence.Object;
import us.ihmc.robotics.partNames.FingerName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.RealtimeRos2Node;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.dataBuffer.MirroredYoVariableRegistry;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class SimulatedValkyrieFingerController extends SimulatedHandControlTask
{
   private final YoDouble handControllerTime;
   private final SimulatedValkyrieFingerJointAngleProducer jointAngleProducer;

   private final double trajectoryTime = ValkyrieHandFingerTrajectoryMessageConversion.trajectoryTimeForSim;
   private final double extendedTrajectoryTime = trajectoryTime * ValkyrieHandFingerTrajectoryMessageConversion.extendedTimeRatioForThumb;

   /*
    * The handlers corresponds to state estimators in Valkyrie ros controller.
    */
   private final SideDependentList<EnumMap<ValkyrieHandJointName, YoDouble>> sideDependentHandJointHandlers = SideDependentList.createListOfEnumMaps(ValkyrieHandJointName.class);
   private final SideDependentList<EnumMap<ValkyrieFingerMotorName, YoDouble>> sideDependentFingerMotorHandlers = SideDependentList.createListOfEnumMaps(ValkyrieFingerMotorName.class);

   private final SideDependentList<HandDesiredConfigurationMessageSubscriber> handDesiredConfigurationMessageSubscribers = new SideDependentList<>();
   private final SideDependentList<ValkyrieHandFingerTrajectoryMessageSubscriber> valkyrieHandFingerTrajectoryMessageSubscribers = new SideDependentList<>();

   private final SideDependentList<ValkyrieFingerSetTrajectoryGenerator<ValkyrieHandJointName>> handJointFingerSetControllers = new SideDependentList<>();
   private final SideDependentList<ValkyrieFingerSetTrajectoryGenerator<ValkyrieFingerMotorName>> fingerSetControllers = new SideDependentList<>();

   private final SideDependentList<EnumMap<ValkyrieHandJointName, OneDegreeOfFreedomJoint>> allHandJointsNameToJointsMap = SideDependentList.createListOfEnumMaps(ValkyrieHandJointName.class);
   private final SideDependentList<EnumMap<ValkyrieHandJointName, YoDouble>> currentHandJointAngles = SideDependentList.createListOfEnumMaps(ValkyrieHandJointName.class);

   private long timestamp;

   private final MirroredYoVariableRegistry registry;

   public SimulatedValkyrieFingerController(FloatingRootJointRobot simulatedRobot, RealtimeRos2Node realtimeRos2Node, DRCRobotModel robotModel,
                                            MessageTopicNameGenerator pubTopicNameGenerator, MessageTopicNameGenerator subTopicNameGenerator)
   {
      super((int) Math.round(robotModel.getControllerDT() / robotModel.getSimulateDT()));

      YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
      handControllerTime = new YoDouble("handControllerTime", registry);

      if (realtimeRos2Node != null)
      {
         IHMCRealtimeROS2Publisher<HandJointAnglePacket> jointAnglePublisher = ROS2Tools.createPublisher(realtimeRos2Node, HandJointAnglePacket.class,
                                                                                                         pubTopicNameGenerator);
         jointAngleProducer = new SimulatedValkyrieFingerJointAngleProducer(jointAnglePublisher, simulatedRobot);
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
         ValkyrieFingerSetTrajectoryGenerator<ValkyrieHandJointName> handJointFingerSetController = new ValkyrieFingerSetTrajectoryGenerator<ValkyrieHandJointName>(ValkyrieHandJointName.class,
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
         ValkyrieFingerSetTrajectoryGenerator<ValkyrieFingerMotorName> fingerSetController = new ValkyrieFingerSetTrajectoryGenerator<ValkyrieFingerMotorName>(ValkyrieFingerMotorName.class,
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

      this.registry = new MirroredYoVariableRegistry(registry);
   }

   @Override
   public boolean initialize()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         for (ValkyrieHandJointName valkyrieHandJointName : ValkyrieHandJointName.values())
         {
            OneDegreeOfFreedomJoint oneDegreeOfFreedomJoint = allHandJointsNameToJointsMap.get(robotSide).get(valkyrieHandJointName);
            currentHandJointAngles.get(robotSide).get(valkyrieHandJointName).set(oneDegreeOfFreedomJoint.getQ());
         }
      }
      return true;
   }

   public void read()
   {
      handControllerTime.set(Conversions.nanosecondsToSeconds(timestamp));

      if (jointAngleProducer != null)
      {
         jointAngleProducer.sendHandJointAnglesPacket();
      }
   }

   @Override
   public void execute()
   {
      checkForNewHandDesiredConfigurationRequested();
      checkForNewValkyrieHandFingerTrajectoryRequested();

      for (RobotSide robotSide : RobotSide.values)
      {
         handJointFingerSetControllers.get(robotSide).doControl();
         fingerSetControllers.get(robotSide).doControl();
      }
   }

   public void write()
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
   public YoVariableRegistry getRegistry()
   {
      return registry;
   }

   private void checkForNewHandDesiredConfigurationRequested()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         if (handDesiredConfigurationMessageSubscribers.get(robotSide).isNewDesiredConfigurationAvailable())
         {
            HandConfiguration desiredHandConfiguration = HandConfiguration.fromByte(handDesiredConfigurationMessageSubscribers.get(robotSide).pollMessage()
                                                                                                                              .getDesiredHandConfiguration());
            handJointFingerSetControllers.get(robotSide).clearTrajectories();
            fingerSetControllers.get(robotSide).clearTrajectories();
            switch (desiredHandConfiguration)
            {
            case CLOSE:
               for (ValkyrieHandJointName handJointName : ValkyrieHandJointName.values)
               {
                  double desiredHandJoint = ValkyrieFingerControlParameters.getDesiredHandJoint(robotSide, handJointName, 1.0);
                  if (handJointName.getFingerName() == FingerName.THUMB)
                     handJointFingerSetControllers.get(robotSide).appendWayPoint(handJointName, extendedTrajectoryTime, desiredHandJoint);
                  else
                     handJointFingerSetControllers.get(robotSide).appendWayPoint(handJointName, trajectoryTime, desiredHandJoint);
               }

               for (ValkyrieFingerMotorName fingerMotorName : ValkyrieFingerMotorName.values)
               {
                  double desiredFingerMotor = ValkyrieFingerControlParameters.getDesiredFingerMotorPosition(robotSide, fingerMotorName, 1.0);
                  if (fingerMotorName.getFingerName() == FingerName.THUMB)
                     fingerSetControllers.get(robotSide).appendWayPoint(fingerMotorName, extendedTrajectoryTime, desiredFingerMotor);
                  else
                     fingerSetControllers.get(robotSide).appendWayPoint(fingerMotorName, trajectoryTime, desiredFingerMotor);
               }

               break;

            case OPEN:
               for (ValkyrieHandJointName handJointName : ValkyrieHandJointName.values)
               {
                  double desiredHandJoint = ValkyrieFingerControlParameters.getDesiredHandJoint(robotSide, handJointName, 0.0);
                  if (handJointName.getFingerName() == FingerName.THUMB)
                     handJointFingerSetControllers.get(robotSide).appendWayPoint(handJointName, extendedTrajectoryTime, desiredHandJoint);
                  else
                     handJointFingerSetControllers.get(robotSide).appendWayPoint(handJointName, trajectoryTime, desiredHandJoint);
               }
               for (ValkyrieFingerMotorName fingerMotorName : ValkyrieFingerMotorName.values)
               {
                  double desiredFingerMotor = ValkyrieFingerControlParameters.getDesiredFingerMotorPosition(robotSide, fingerMotorName, 0.0);
                  if (fingerMotorName.getFingerName() == FingerName.THUMB)
                     fingerSetControllers.get(robotSide).appendWayPoint(fingerMotorName, extendedTrajectoryTime, desiredFingerMotor);
                  else
                     fingerSetControllers.get(robotSide).appendWayPoint(fingerMotorName, trajectoryTime, desiredFingerMotor);
               }
               break;

            case STOP:
               for (ValkyrieHandJointName handJointName : ValkyrieHandJointName.values)
                  handJointFingerSetControllers.get(robotSide)
                                               .appendStopPoint(handJointName,
                                                                sideDependentHandJointHandlers.get(robotSide).get(handJointName).getDoubleValue());
               for (ValkyrieFingerMotorName fingerMotorName : ValkyrieFingerMotorName.values)
                  fingerSetControllers.get(robotSide).appendStopPoint(fingerMotorName,
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

            handJointFingerSetControllers.get(robotSide).clearTrajectories();
            fingerSetControllers.get(robotSide).clearTrajectories();

            for (ValkyrieFingerMotorName fingerMotorName : ValkyrieFingerMotorName.values)
            {
               int indexOfTrajectory = hasTrajectory(handFingerTrajectoryMessage.getValkyrieFingerMotorNames(), fingerMotorName);

               if (indexOfTrajectory == -1)
               {
                  fingerSetControllers.get(robotSide).appendStopPoint(fingerMotorName,
                                                                      sideDependentFingerMotorHandlers.get(robotSide).get(fingerMotorName).getDoubleValue());
                  setEstimatedStopedValueOnHandJointFingerSetController(robotSide, fingerMotorName);
               }
               else if (indexOfTrajectory == -2)
               {
                  ;
               }
               else
               {
                  Object<OneDoFJointTrajectoryMessage> jointTrajectoryMessages = handFingerTrajectoryMessage.getJointspaceTrajectory()
                                                                                                            .getJointTrajectoryMessages();
                  Object<TrajectoryPoint1DMessage> trajectoryPoints = jointTrajectoryMessages.get(indexOfTrajectory).getTrajectoryPoints();

                  for (int i = 0; i < trajectoryPoints.size(); i++)
                  {
                     TrajectoryPoint1DMessage trajectoryPoint1DMessage = trajectoryPoints.get(i);
                     double wayPointTime = trajectoryPoint1DMessage.getTime();
                     double wayPointPosition = ValkyrieFingerControlParameters.getDesiredFingerMotorPosition(robotSide, fingerMotorName,
                                                                                                     trajectoryPoint1DMessage.getPosition());
                     fingerSetControllers.get(robotSide).appendWayPoint(fingerMotorName, wayPointTime, wayPointPosition);
                  }
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
      if (fingerMotorName == ValkyrieFingerMotorName.ThumbMotorPitch2)
         return -2;

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

         int indexOfTrajectory = hasTrajectory(handFingerTrajectoryMessage.getValkyrieFingerMotorNames(), fingerMotorName);

         Object<OneDoFJointTrajectoryMessage> jointTrajectoryMessages = handFingerTrajectoryMessage.getJointspaceTrajectory().getJointTrajectoryMessages();
         Object<TrajectoryPoint1DMessage> trajectoryPoints = jointTrajectoryMessages.get(indexOfTrajectory).getTrajectoryPoints();

         for (int j = 0; j < trajectoryPoints.size(); j++)
         {
            TrajectoryPoint1DMessage trajectoryPoint1DMessage = trajectoryPoints.get(j);
            double wayPointTime = trajectoryPoint1DMessage.getTime();
            double wayPointPosition = ValkyrieFingerControlParameters.getDesiredHandJoint(robotSide, handJointName, trajectoryPoint1DMessage.getPosition());
            handJointFingerSetControllers.get(robotSide).appendWayPoint(handJointName, wayPointTime, wayPointPosition);
         }
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
         handJointFingerSetControllers.get(robotSide).appendStopPoint(handJointName,
                                                                      sideDependentHandJointHandlers.get(robotSide).get(handJointName).getDoubleValue());
      }
   }

   @Override
   protected void cleanup()
   {
      if (jointAngleProducer != null)
      {
         jointAngleProducer.cleanup();
      }
   }

   @Override
   protected void updateMasterContext(HumanoidRobotContextData context)
   {
      write();
      registry.updateMirror();
   }

   @Override
   protected void updateLocalContext(HumanoidRobotContextData context)
   {
      timestamp = context.getTimestamp();
      read();
   }
}