package us.ihmc.valkyrie.fingers;

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
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class SimulatedValkyrieSingleHandFingerController implements RobotController
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

   private final double trajectoryTime = ValkyrieHandFingerTrajectoryMessageConversion.trajectoryTimeForSim;
   private final double extendedTrajectoryTime = trajectoryTime * ValkyrieHandFingerTrajectoryMessageConversion.extendedTimeRatioForThumb;

   private final ValkyrieFingerSetTrajectoryGenerator<ValkyrieHandJointName> handJointFingerSetController;
   private final ValkyrieFingerSetTrajectoryGenerator<ValkyrieFingerMotorName> fingerSetController;

   private final HandDesiredConfigurationMessageSubscriber handDesiredConfigurationSubscriber;
   private final ValkyrieHandFingerTrajectoryMessageSubscriber handFingerTrajectorySubscriber;

   private final EnumMap<ValkyrieHandJointName, YoDouble> handJointHandlers = new EnumMap<>(ValkyrieHandJointName.class);
   private final EnumMap<ValkyrieFingerMotorName, YoDouble> fingerMotorHandlers = new EnumMap<>(ValkyrieFingerMotorName.class);

   private final EnumMap<ValkyrieHandJointName, OneDoFJointBasics> jointsNameToJointsMap = new EnumMap<>(ValkyrieHandJointName.class);

   public SimulatedValkyrieSingleHandFingerController(RobotSide robotSide,
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

      for (ValkyrieHandJointName valkyrieHandJointName : ValkyrieHandJointName.values())
      {
         YoDouble handJointHandler = new YoDouble(valkyrieHandJointName.getJointName(robotSide) + "_handJointHandler", registry);
         double currentHandJoint = fullRobotModel.getOneDoFJointByName(valkyrieHandJointName.getJointName(robotSide)).getQ(); // FIXME That's not going to get initialized properly
         handJointHandler.set(currentHandJoint);
         handJointHandlers.put(valkyrieHandJointName, handJointHandler);
      }
      handJointFingerSetController = new ValkyrieFingerSetTrajectoryGenerator<>(ValkyrieHandJointName.class, robotSide, yoTime, handJointHandlers, registry);

      for (ValkyrieFingerMotorName valkyrieFingerMotorName : ValkyrieFingerMotorName.values())
      {
         YoDouble fingerMotorHandler = new YoDouble(valkyrieFingerMotorName.getJointName(robotSide) + "_fingerMotorHandler", registry);
         double currentFingerMotor = 0.0; // TODO : use ValkyrieRosControlFingerStateEstimator
         fingerMotorHandler.set(currentFingerMotor);
         fingerMotorHandlers.put(valkyrieFingerMotorName, fingerMotorHandler);
      }
      fingerSetController = new ValkyrieFingerSetTrajectoryGenerator<>(ValkyrieFingerMotorName.class, robotSide, yoTime, fingerMotorHandlers, registry);

      for (ValkyrieHandJointName valkyrieHandJointName : ValkyrieHandJointName.values())
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
      for (ValkyrieHandJointName valkyrieHandJointName : ValkyrieHandJointName.values())
      {
         double desiredQ = handJointFingerSetController.getDesired(valkyrieHandJointName);
         OneDoFJointBasics joint = jointsNameToJointsMap.get(valkyrieHandJointName);
         JointDesiredOutputBasics jointDesiredOutput = jointDesiredOutputList.getJointDesiredOutput(joint);
         jointDesiredOutput.setControlMode(JointDesiredControlMode.POSITION);
         jointDesiredOutput.setDesiredPosition(desiredQ);
      }

      for (ValkyrieHandJointName valkyrieHandJointName : ValkyrieHandJointName.values)
      {
         double desiredQ = handJointFingerSetController.getDesired(valkyrieHandJointName);
         handJointHandlers.get(valkyrieHandJointName).set(desiredQ);
      }
      for (ValkyrieFingerMotorName valkyrieFingerMotorName : ValkyrieFingerMotorName.values)
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
               for (ValkyrieHandJointName handJointName : ValkyrieHandJointName.values)
               {
                  double desiredHandJoint = ValkyrieFingerControlParameters.getDesiredHandJoint(robotSide, handJointName, 1.0);
                  handJointFingerSetController.appendWayPoint(handJointName, getTrajectoryDuration(handJointName), desiredHandJoint);
               }

               for (ValkyrieFingerMotorName fingerMotorName : ValkyrieFingerMotorName.values)
               {
                  double desiredFingerMotor = ValkyrieFingerControlParameters.getDesiredFingerMotorPosition(robotSide, fingerMotorName, 1.0);
                  fingerSetController.appendWayPoint(fingerMotorName, getTrajectoryDuration(fingerMotorName), desiredFingerMotor);
               }

               break;

            case OPEN:
               for (ValkyrieHandJointName handJointName : ValkyrieHandJointName.values)
               {
                  double desiredHandJoint = ValkyrieFingerControlParameters.getDesiredHandJoint(robotSide, handJointName, 0.0);
                  handJointFingerSetController.appendWayPoint(handJointName, getTrajectoryDuration(handJointName), desiredHandJoint);
               }
               for (ValkyrieFingerMotorName fingerMotorName : ValkyrieFingerMotorName.values)
               {
                  double desiredFingerMotor = ValkyrieFingerControlParameters.getDesiredFingerMotorPosition(robotSide, fingerMotorName, 0.0);
                  fingerSetController.appendWayPoint(fingerMotorName, getTrajectoryDuration(fingerMotorName), desiredFingerMotor);
               }
               break;

            case STOP:
               for (ValkyrieHandJointName handJointName : ValkyrieHandJointName.values)
                  handJointFingerSetController.appendStopPoint(handJointName, handJointHandlers.get(handJointName).getDoubleValue());
               for (ValkyrieFingerMotorName fingerMotorName : ValkyrieFingerMotorName.values)
                  fingerSetController.appendStopPoint(fingerMotorName, fingerMotorHandlers.get(fingerMotorName).getDoubleValue());
               break;

            default:

               break;
         }
         handJointFingerSetController.executeTrajectories();
         fingerSetController.executeTrajectories();
      }
   }

   private double getTrajectoryDuration(ValkyrieHandJointName jointName)
   {
      return getTrajectoryDuration(jointName.getFingerName());
   }

   private double getTrajectoryDuration(ValkyrieFingerMotorName motorName)
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

         for (ValkyrieFingerMotorName fingerMotorName : ValkyrieFingerMotorName.values)
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
                  double wayPointPosition = ValkyrieFingerControlParameters.getDesiredFingerMotorPosition(robotSide,
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

   private int hasTrajectory(us.ihmc.idl.IDLSequence.Byte namesInMessage, ValkyrieFingerMotorName fingerMotorName)
   {
      if (fingerMotorName == ValkyrieFingerMotorName.ThumbMotorPitch2)
         return -2;

      for (int i = 0; i < namesInMessage.size(); i++)
         if (fingerMotorName == ValkyrieFingerMotorName.fromByte(namesInMessage.get(i)))
            return i;

      return -1;
   }

   private void setEstimatedDesiredValueOnHandJointFingerSetController(ValkyrieFingerMotorName fingerMotorName,
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
            handJointFingerSetController.appendWayPoint(handJointName, wayPointTime, wayPointPosition);
         }
      }
   }

   private void setEstimatedStopedValueOnHandJointFingerSetController(ValkyrieFingerMotorName fingerMotorName)
   {
      int numberOfHandJoints = 3;
      if (fingerMotorName == ValkyrieFingerMotorName.ThumbMotorRoll)
         numberOfHandJoints = 1;
      for (int i = 1; i <= numberOfHandJoints; i++)
      {
         ValkyrieHandJointName handJointName = fingerMotorName.getCorrespondingJointName(i);
         handJointFingerSetController.appendStopPoint(handJointName, handJointHandlers.get(handJointName).getDoubleValue());
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
