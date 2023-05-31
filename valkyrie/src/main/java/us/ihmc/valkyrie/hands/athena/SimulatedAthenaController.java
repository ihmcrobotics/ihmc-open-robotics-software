package us.ihmc.valkyrie.hands.athena;

import java.util.EnumMap;

import controller_msgs.msg.dds.HandDesiredConfigurationMessage;
import controller_msgs.msg.dds.HandJointAnglePacket;
import controller_msgs.msg.dds.OneDoFJointTrajectoryMessage;
import controller_msgs.msg.dds.ValkyrieHandFingerTrajectoryMessage;
import ihmc_common_msgs.msg.dds.TrajectoryPoint1DMessage;
import us.ihmc.avatar.handControl.packetsAndConsumers.HandJointAngleCommunicator;
import us.ihmc.avatar.handControl.packetsAndConsumers.HandSensorData;
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
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.sensorProcessing.outputData.JointDesiredControlMode;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputBasics;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListBasics;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.valkyrie.hands.athena.AthenaHandModel.AthenaFingerMotorName;
import us.ihmc.valkyrie.hands.athena.AthenaHandModel.AthenaJointName;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class SimulatedAthenaController implements RobotController
{
   private final YoRegistry registry;

   private final RobotSide robotSide;
   private final JointDesiredOutputListBasics jointDesiredOutputList;

   private final HandJointAngleCommunicator jointAngleProducer;
   private double[] jointAngles;
   private final HandSensorData handSensorData = new HandSensorData()
   {
      @Override
      public boolean isConnected()
      {
         return true;
      }

      @Override
      public boolean isCalibrated()
      {
         return true;
      }

      @Override
      public double[] getFingerJointAngles(RobotSide robotSide)
      {
         return jointAngles;
      }
   };

   private final double trajectoryTime = AthenaTrajectoryMessageConversion.trajectoryTimeForSim;
   private final double extendedTrajectoryTime = trajectoryTime * AthenaTrajectoryMessageConversion.extendedTimeRatioForThumb;

   private final AthenaTrajectoryGenerator<AthenaJointName> handJointFingerSetController;
   private final AthenaTrajectoryGenerator<AthenaFingerMotorName> fingerSetController;

   private final HandDesiredConfigurationMessageSubscriber handDesiredConfigurationSubscriber;
   private final ValkyrieHandFingerTrajectoryMessageSubscriber handFingerTrajectorySubscriber;

   private final EnumMap<AthenaJointName, YoDouble> handJointHandlers = new EnumMap<>(AthenaJointName.class);
   private final EnumMap<AthenaFingerMotorName, YoDouble> fingerMotorHandlers = new EnumMap<>(AthenaFingerMotorName.class);

   private final EnumMap<AthenaJointName, OneDoFJointBasics> jointsNameToJointsMap = new EnumMap<>(AthenaJointName.class);

   public SimulatedAthenaController(RobotSide robotSide,
                                    FullRobotModel fullRobotModel,
                                    JointDesiredOutputListBasics jointDesiredOutputList,
                                    DoubleProvider yoTime,
                                    RealtimeROS2Node realtimeROS2Node,
                                    ROS2Topic<?> outputTopic,
                                    ROS2Topic<?> inputTopic)
   {
      this.robotSide = robotSide;
      this.jointDesiredOutputList = jointDesiredOutputList;

      registry = new YoRegistry(robotSide.getCamelCaseName() + getClass().getSimpleName());
      // TODO Consider moving up
      if (realtimeROS2Node != null)
      {
         IHMCRealtimeROS2Publisher<HandJointAnglePacket> publisher = ROS2Tools.createPublisherTypeNamed(realtimeROS2Node,
                                                                                                        HandJointAnglePacket.class,
                                                                                                        outputTopic);
         jointAngleProducer = new HandJointAngleCommunicator(robotSide, publisher);
      }
      else
      {
         jointAngleProducer = null;
      }

      for (AthenaJointName valkyrieHandJointName : AthenaJointName.values())
      {
         YoDouble handJointHandler = new YoDouble(valkyrieHandJointName.getJointName(robotSide) + "_handJointHandler", registry);
         double currentHandJoint = fullRobotModel.getOneDoFJointByName(valkyrieHandJointName.getJointName(robotSide)).getQ(); // FIXME That's not going to get initialized properly
         handJointHandler.set(currentHandJoint);
         handJointHandlers.put(valkyrieHandJointName, handJointHandler);
      }
      handJointFingerSetController = new AthenaTrajectoryGenerator<>(AthenaJointName.class, robotSide, yoTime, handJointHandlers, registry);

      for (AthenaFingerMotorName valkyrieFingerMotorName : AthenaFingerMotorName.values())
      {
         YoDouble fingerMotorHandler = new YoDouble(valkyrieFingerMotorName.getJointName(robotSide) + "_fingerMotorHandler", registry);
         double currentFingerMotor = 0.0; // TODO : use ValkyrieRosControlFingerStateEstimator
         fingerMotorHandler.set(currentFingerMotor);
         fingerMotorHandlers.put(valkyrieFingerMotorName, fingerMotorHandler);
      }
      fingerSetController = new AthenaTrajectoryGenerator<>(AthenaFingerMotorName.class, robotSide, yoTime, fingerMotorHandlers, registry);

      for (AthenaJointName valkyrieHandJointName : AthenaJointName.values())
      {
         String jointName = valkyrieHandJointName.getJointName(robotSide);
         OneDoFJointBasics oneDoFJoint = fullRobotModel.getOneDoFJointByName(jointName);
         jointsNameToJointsMap.put(valkyrieHandJointName, oneDoFJoint);
      }

      if (realtimeROS2Node != null)
      {
         handDesiredConfigurationSubscriber = new HandDesiredConfigurationMessageSubscriber(robotSide);
         handFingerTrajectorySubscriber = new ValkyrieHandFingerTrajectoryMessageSubscriber(robotSide);
         ROS2Tools.createCallbackSubscriptionTypeNamed(realtimeROS2Node, HandDesiredConfigurationMessage.class, inputTopic, handDesiredConfigurationSubscriber);
         ROS2Tools.createCallbackSubscriptionTypeNamed(realtimeROS2Node, ValkyrieHandFingerTrajectoryMessage.class, inputTopic, handFingerTrajectorySubscriber);
      }
      else
      {
         handDesiredConfigurationSubscriber = null;
         handFingerTrajectorySubscriber = null;
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
         jointAngles = jointsNameToJointsMap.values().stream().mapToDouble(j -> j.getQ()).toArray();
         jointAngleProducer.updateHandAngles(handSensorData);
         jointAngleProducer.write();
      }
   }

   public void execute()
   {
      checkForNewHandDesiredConfigurationRequested();
      checkForNewValkyrieHandFingerTrajectoryRequested();

      handJointFingerSetController.doControl();
      fingerSetController.doControl();
   }

   public void write()
   {
      for (AthenaJointName valkyrieHandJointName : AthenaJointName.values())
      {
         double desiredQ = handJointFingerSetController.getDesired(valkyrieHandJointName);
         OneDoFJointBasics joint = jointsNameToJointsMap.get(valkyrieHandJointName);
         JointDesiredOutputBasics jointDesiredOutput = jointDesiredOutputList.getJointDesiredOutput(joint);
         jointDesiredOutput.setControlMode(JointDesiredControlMode.POSITION);
         jointDesiredOutput.setDesiredPosition(desiredQ);
      }

      for (AthenaJointName valkyrieHandJointName : AthenaJointName.values)
      {
         double desiredQ = handJointFingerSetController.getDesired(valkyrieHandJointName);
         handJointHandlers.get(valkyrieHandJointName).set(desiredQ);
      }
      for (AthenaFingerMotorName valkyrieFingerMotorName : AthenaFingerMotorName.values)
      {
         double desiredQ = fingerSetController.getDesired(valkyrieFingerMotorName);
         fingerMotorHandlers.get(valkyrieFingerMotorName).set(desiredQ);
      }
   }

   @Override
   public YoRegistry getYoRegistry()
   {
      return registry;
   }

   private void checkForNewHandDesiredConfigurationRequested()
   {
      if (handDesiredConfigurationSubscriber != null && handDesiredConfigurationSubscriber.isNewDesiredConfigurationAvailable())
      {
         HandConfiguration desiredHandConfiguration = HandConfiguration.fromByte(handDesiredConfigurationSubscriber.pollMessage()
                                                                                                                   .getDesiredHandConfiguration());

         handJointFingerSetController.clearTrajectories();
         fingerSetController.clearTrajectories();

         switch (desiredHandConfiguration)
         {
            case CLOSE:
               for (AthenaJointName handJointName : AthenaJointName.values)
               {
                  double desiredHandJoint = AthenaFingerControlParameters.getDesiredHandJoint(robotSide, handJointName, 1.0);
                  handJointFingerSetController.appendWayPoint(handJointName, getTrajectoryDuration(handJointName), desiredHandJoint);
               }

               for (AthenaFingerMotorName fingerMotorName : AthenaFingerMotorName.values)
               {
                  double desiredFingerMotor = AthenaFingerControlParameters.getDesiredFingerMotorPosition(robotSide, fingerMotorName, 1.0);
                  fingerSetController.appendWayPoint(fingerMotorName, getTrajectoryDuration(fingerMotorName), desiredFingerMotor);
               }

               break;

            case OPEN:
               for (AthenaJointName handJointName : AthenaJointName.values)
               {
                  double desiredHandJoint = AthenaFingerControlParameters.getDesiredHandJoint(robotSide, handJointName, 0.0);
                  handJointFingerSetController.appendWayPoint(handJointName, getTrajectoryDuration(handJointName), desiredHandJoint);
               }
               for (AthenaFingerMotorName fingerMotorName : AthenaFingerMotorName.values)
               {
                  double desiredFingerMotor = AthenaFingerControlParameters.getDesiredFingerMotorPosition(robotSide, fingerMotorName, 0.0);
                  fingerSetController.appendWayPoint(fingerMotorName, getTrajectoryDuration(fingerMotorName), desiredFingerMotor);
               }
               break;

            case STOP:
               for (AthenaJointName handJointName : AthenaJointName.values)
                  handJointFingerSetController.appendStopPoint(handJointName, handJointHandlers.get(handJointName).getDoubleValue());
               for (AthenaFingerMotorName fingerMotorName : AthenaFingerMotorName.values)
                  fingerSetController.appendStopPoint(fingerMotorName, fingerMotorHandlers.get(fingerMotorName).getDoubleValue());
               break;

            default:

               break;
         }
         handJointFingerSetController.executeTrajectories();
         fingerSetController.executeTrajectories();
      }
   }

   private double getTrajectoryDuration(AthenaJointName jointName)
   {
      return getTrajectoryDuration(jointName.getFingerName());
   }

   private double getTrajectoryDuration(AthenaFingerMotorName motorName)
   {
      return getTrajectoryDuration(motorName.getFingerName());
   }

   private double getTrajectoryDuration(FingerName fingerName)
   {
      return fingerName == FingerName.THUMB ? extendedTrajectoryTime : trajectoryTime;
   }

   private void checkForNewValkyrieHandFingerTrajectoryRequested()
   {
      if (handFingerTrajectorySubscriber != null && handFingerTrajectorySubscriber.isNewDesiredConfigurationAvailable())
      {
         ValkyrieHandFingerTrajectoryMessage handFingerTrajectoryMessage = handFingerTrajectorySubscriber.pollMessage();

         handJointFingerSetController.clearTrajectories();
         fingerSetController.clearTrajectories();

         for (AthenaFingerMotorName fingerMotorName : AthenaFingerMotorName.values)
         {
            int indexOfTrajectory = hasTrajectory(handFingerTrajectoryMessage.getValkyrieFingerMotorNames(), fingerMotorName);

            if (indexOfTrajectory == -1)
            {
               fingerSetController.appendStopPoint(fingerMotorName, fingerMotorHandlers.get(fingerMotorName).getDoubleValue());
               setEstimatedStopedValueOnHandJointFingerSetController(fingerMotorName);
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
                  double wayPointPosition = AthenaFingerControlParameters.getDesiredFingerMotorPosition(robotSide,
                                                                                                        fingerMotorName,
                                                                                                        trajectoryPoint1DMessage.getPosition());
                  fingerSetController.appendWayPoint(fingerMotorName, wayPointTime, wayPointPosition);
               }
               setEstimatedDesiredValueOnHandJointFingerSetController(fingerMotorName, handFingerTrajectoryMessage);
            }
         }
         handJointFingerSetController.executeTrajectories();
         fingerSetController.executeTrajectories();
      }
   }

   private int hasTrajectory(us.ihmc.idl.IDLSequence.Byte namesInMessage, AthenaFingerMotorName fingerMotorName)
   {
      if (fingerMotorName == AthenaFingerMotorName.ThumbMotorPitch2)
         return -2;

      for (int i = 0; i < namesInMessage.size(); i++)
         if (fingerMotorName == AthenaFingerMotorName.fromByte(namesInMessage.get(i)))
            return i;

      return -1;
   }

   private void setEstimatedDesiredValueOnHandJointFingerSetController(AthenaFingerMotorName fingerMotorName,
                                                                       ValkyrieHandFingerTrajectoryMessage handFingerTrajectoryMessage)
   {
      int numberOfHandJoints = 3;
      if (fingerMotorName == AthenaFingerMotorName.ThumbMotorRoll)
         numberOfHandJoints = 1;
      for (int i = 1; i <= numberOfHandJoints; i++)
      {
         AthenaJointName handJointName = fingerMotorName.getCorrespondingJointName(i);

         int indexOfTrajectory = hasTrajectory(handFingerTrajectoryMessage.getValkyrieFingerMotorNames(), fingerMotorName);

         Object<OneDoFJointTrajectoryMessage> jointTrajectoryMessages = handFingerTrajectoryMessage.getJointspaceTrajectory().getJointTrajectoryMessages();
         Object<TrajectoryPoint1DMessage> trajectoryPoints = jointTrajectoryMessages.get(indexOfTrajectory).getTrajectoryPoints();

         for (TrajectoryPoint1DMessage trajectoryPoint1DMessage : trajectoryPoints)
         {
            double wayPointTime = trajectoryPoint1DMessage.getTime();
            double wayPointPosition = AthenaFingerControlParameters.getDesiredHandJoint(robotSide, handJointName, trajectoryPoint1DMessage.getPosition());
            handJointFingerSetController.appendWayPoint(handJointName, wayPointTime, wayPointPosition);
         }
      }
   }

   private void setEstimatedStopedValueOnHandJointFingerSetController(AthenaFingerMotorName fingerMotorName)
   {
      int numberOfHandJoints = 3;
      if (fingerMotorName == AthenaFingerMotorName.ThumbMotorRoll)
         numberOfHandJoints = 1;
      for (int i = 1; i <= numberOfHandJoints; i++)
      {
         AthenaJointName handJointName = fingerMotorName.getCorrespondingJointName(i);
         handJointFingerSetController.appendStopPoint(handJointName, handJointHandlers.get(handJointName).getDoubleValue());
      }
   }

   public void cleanup()
   {
      if (jointAngleProducer != null)
      {
         jointAngleProducer.cleanup();
      }
   }
}
