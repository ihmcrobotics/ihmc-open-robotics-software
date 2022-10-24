package us.ihmc.valkyrie.fingers;

import java.util.EnumMap;

import controller_msgs.msg.dds.HandDesiredConfigurationMessage;
import controller_msgs.msg.dds.HandJointAnglePacket;
import controller_msgs.msg.dds.OneDoFJointTrajectoryMessage;
import ihmc_common_msgs.msg.dds.TrajectoryPoint1DMessage;
import controller_msgs.msg.dds.ValkyrieHandFingerTrajectoryMessage;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.humanoidRobotics.communication.subscribers.HandDesiredConfigurationMessageSubscriber;
import us.ihmc.humanoidRobotics.communication.subscribers.ValkyrieHandFingerTrajectoryMessageSubscriber;
import us.ihmc.idl.IDLSequence.Object;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.partNames.FingerName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.sensorProcessing.outputData.JointDesiredControlMode;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputBasics;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListBasics;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class SimulatedValkyrieFingerController implements RobotController
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final SimulatedValkyrieFingerJointAngleProducer jointAngleProducer;
   private final JointDesiredOutputListBasics jointDesiredOutputList;

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

   private final SideDependentList<EnumMap<ValkyrieHandJointName, OneDoFJointBasics>> allHandJointsNameToJointsMap = SideDependentList.createListOfEnumMaps(ValkyrieHandJointName.class);

   public SimulatedValkyrieFingerController(FullRobotModel fullRobotModel,
                                            JointDesiredOutputListBasics jointDesiredOutputList,
                                            DoubleProvider yoTime,
                                            RealtimeROS2Node realtimeROS2Node,
                                            ROS2Topic<?> outputTopic,
                                            ROS2Topic<?> inputTopic)
   {
      this.jointDesiredOutputList = jointDesiredOutputList;

      if (realtimeROS2Node != null)
      {
         IHMCRealtimeROS2Publisher<HandJointAnglePacket> jointAnglePublisher = ROS2Tools.createPublisherTypeNamed(realtimeROS2Node,
                                                                                                                  HandJointAnglePacket.class,
                                                                                                                  outputTopic);
         jointAngleProducer = new SimulatedValkyrieFingerJointAngleProducer(jointAnglePublisher, fullRobotModel);
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
            double currentHandJoint = fullRobotModel.getOneDoFJointByName(valkyrieHandJointName.getJointName(robotSide)).getQ(); // FIXME That's not going to get initialized properly
            handJointHandler.set(currentHandJoint);
            sideDependentHandJointHandlers.get(robotSide).put(valkyrieHandJointName, handJointHandler);
         }
         ValkyrieFingerSetTrajectoryGenerator<ValkyrieHandJointName> handJointFingerSetController = new ValkyrieFingerSetTrajectoryGenerator<>(ValkyrieHandJointName.class,
                                                                                                                                               robotSide,
                                                                                                                                               yoTime,
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
         ValkyrieFingerSetTrajectoryGenerator<ValkyrieFingerMotorName> fingerSetController = new ValkyrieFingerSetTrajectoryGenerator<>(ValkyrieFingerMotorName.class,
                                                                                                                                                               robotSide,
                                                                                                                                                               yoTime,
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
            OneDoFJointBasics oneDoFJoint = fullRobotModel.getOneDoFJointByName(jointName);
            allHandJointsNameToJointsMap.get(robotSide).put(valkyrieHandJointName, oneDoFJoint);
         }
      }

      // Subscribers
      for (RobotSide robotSide : RobotSide.values)
      {
         HandDesiredConfigurationMessageSubscriber handDesiredConfigurationSubscriber = new HandDesiredConfigurationMessageSubscriber(robotSide);
         handDesiredConfigurationMessageSubscribers.put(robotSide, handDesiredConfigurationSubscriber);

         ValkyrieHandFingerTrajectoryMessageSubscriber valkyrieHandFingerTrajectoryMessageSubscriber = new ValkyrieHandFingerTrajectoryMessageSubscriber(robotSide);
         valkyrieHandFingerTrajectoryMessageSubscribers.put(robotSide, valkyrieHandFingerTrajectoryMessageSubscriber);
         if (realtimeROS2Node != null)
         {
            ROS2Tools.createCallbackSubscriptionTypeNamed(realtimeROS2Node,
                                                          HandDesiredConfigurationMessage.class,
                                                          inputTopic,
                                                          handDesiredConfigurationSubscriber);
            ROS2Tools.createCallbackSubscriptionTypeNamed(realtimeROS2Node,
                                                          ValkyrieHandFingerTrajectoryMessage.class,
                                                          inputTopic,
                                                          valkyrieHandFingerTrajectoryMessageSubscriber);
         }
      }
   }

   @Override
   public void initialize()
   {
   }

   @Override
   public void doControl()
   {
      read();
      execute();
      write();
   }

   public void read()
   {
      if (jointAngleProducer != null)
      {
         jointAngleProducer.sendHandJointAnglesPacket();
      }
   }

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
            OneDoFJointBasics joint = allHandJointsNameToJointsMap.get(robotSide).get(valkyrieHandJointName);
            JointDesiredOutputBasics jointDesiredOutput = jointDesiredOutputList.getJointDesiredOutput(joint);
            jointDesiredOutput.setControlMode(JointDesiredControlMode.POSITION);
            jointDesiredOutput.setDesiredPosition(desiredQ);
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
   public YoRegistry getYoRegistry()
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

               }
               else
               {
                  Object<OneDoFJointTrajectoryMessage> jointTrajectoryMessages = handFingerTrajectoryMessage.getJointspaceTrajectory()
                                                                                                            .getJointTrajectoryMessages();
                  Object<TrajectoryPoint1DMessage> trajectoryPoints = jointTrajectoryMessages.get(indexOfTrajectory).getTrajectoryPoints();

                  for (TrajectoryPoint1DMessage trajectoryPoint1DMessage : trajectoryPoints)
                  {
                     double wayPointTime = trajectoryPoint1DMessage.getTime();
                     double wayPointPosition = ValkyrieFingerControlParameters.getDesiredFingerMotorPosition(robotSide,
                                                                                                             fingerMotorName,
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

   private void setEstimatedDesiredValueOnHandJointFingerSetController(RobotSide robotSide,
                                                                       ValkyrieFingerMotorName fingerMotorName,
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

         for (TrajectoryPoint1DMessage trajectoryPoint1DMessage : trajectoryPoints)
         {
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

   protected void cleanup()
   {
      if (jointAngleProducer != null)
      {
         jointAngleProducer.cleanup();
      }
   }
}