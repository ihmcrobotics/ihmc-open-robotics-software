package us.ihmc.valkyrie.fingers;

import java.util.LinkedHashMap;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.handControl.packetsAndConsumers.HandJointAngleCommunicator;
import us.ihmc.avatar.handControl.packetsAndConsumers.HandSensorData;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.PrintTools;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandDesiredConfigurationMessage;
import us.ihmc.humanoidRobotics.communication.streamingData.HumanoidGlobalDataProducer;
import us.ihmc.humanoidRobotics.communication.subscribers.HandDesiredConfigurationMessageSubscriber;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoLong;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.RevoluteJoint;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutput;
import us.ihmc.sensorProcessing.outputData.LowLevelOneDoFJointDesiredDataHolderList;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationConstructionSetTools.robotController.MultiThreadedRobotControlElement;
import us.ihmc.tools.thread.CloseableAndDisposableRegistry;
import us.ihmc.wholeBodyController.concurrent.ThreadDataSynchronizerInterface;

public class ValkyrieFingerController implements MultiThreadedRobotControlElement
{
   private final boolean DEBUG = true;

   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);
   private final boolean isRunningOnRealRobot;
   private final FullRobotModel fullRobotModel;
   private final LowLevelOneDoFJointDesiredDataHolderList lowLevelControlOutputs;

   private final YoDouble fingerControllerTime = new YoDouble("fingerControllerTime", registry);
   private final YoLong lastEstimatorStartTime = new YoLong("lastEstimatorStartTime", registry);
   private final YoBoolean sendFingerJointGains = new YoBoolean("sendFingerJointGains", registry);
   private final YoDouble fingerTrajectoryTime = new YoDouble("FingerTrajectoryTime", registry);

   private final SideDependentList<LinkedHashMap<ValkyrieHandJointName, YoDouble>> kpMap = new SideDependentList<>();
   private final SideDependentList<LinkedHashMap<ValkyrieHandJointName, YoDouble>> kdMap = new SideDependentList<>();
   private final SideDependentList<LinkedHashMap<ValkyrieHandJointName, JointDesiredOutput>> outputs = new SideDependentList<>();

   private final long controlDTInNS;
   private final long estimatorDTInNS;

   private final ThreadDataSynchronizerInterface threadDataSynchronizer;

   private final SideDependentList<HandDesiredConfigurationMessageSubscriber> handDesiredConfigurationMessageSubscribers = new SideDependentList<>();
   private final SideDependentList<ValkyrieFingerSetController> fingerSetControllers = new SideDependentList<>();

   private final SideDependentList<HandJointAngleCommunicator> jointAngleCommunicators = new SideDependentList<>();

   /**
    * @param robotModel
    * @param fullRobotModel null if running on real robot
    * @param threadDataSynchronizer
    * @param globalDataProducer
    * @param yoVariableRegistry
    */
   public ValkyrieFingerController(DRCRobotModel robotModel, FloatingRootJointRobot simulatedRobot, ThreadDataSynchronizerInterface threadDataSynchronizer,
                                   HumanoidGlobalDataProducer globalDataProducer, YoVariableRegistry controllerRegistry,
                                   CloseableAndDisposableRegistry closeableAndDisposableRegistry)
   {
      this.isRunningOnRealRobot = robotModel.getStateEstimatorParameters().isRunningOnRealRobot();
      PrintTools.debug(DEBUG, "Running on real robot: " + isRunningOnRealRobot);
      this.threadDataSynchronizer = threadDataSynchronizer;
      this.fullRobotModel = threadDataSynchronizer.getControllerFullRobotModel();
      this.lowLevelControlOutputs = threadDataSynchronizer.getControllerDesiredJointDataHolder();
      this.controlDTInNS = Conversions.secondsToNanoseconds(robotModel.getControllerDT());
      this.estimatorDTInNS = Conversions.secondsToNanoseconds(robotModel.getEstimatorDT());
      sendFingerJointGains.set(true);
      fingerTrajectoryTime.set(0.5);

      for (RobotSide robotSide : RobotSide.values)
      {
         if (!isRunningOnRealRobot)
         {
            kpMap.put(robotSide, new LinkedHashMap<ValkyrieHandJointName, YoDouble>());
            kdMap.put(robotSide, new LinkedHashMap<ValkyrieHandJointName, YoDouble>());

            for (ValkyrieHandJointName simulatedFingerJoint : ValkyrieHandJointName.values)
            {
               YoDouble kp = new YoDouble("kp" + robotSide.getCamelCaseNameForMiddleOfExpression() + simulatedFingerJoint.name(), registry);
               YoDouble kd = new YoDouble("kd" + robotSide.getCamelCaseNameForMiddleOfExpression() + simulatedFingerJoint.name(), registry);

               double kpkp = 2.0;
               double kdkd = 0.1;

               if (simulatedFingerJoint.name().endsWith("ThumbPitch1") || simulatedFingerJoint.name().endsWith("ThumbRoll"))
               {
                  kp.set(kpkp);
                  kd.set(kdkd);
               }
               else if (simulatedFingerJoint.name().endsWith("ThumbPitch2"))
               {
                  kp.set(kpkp / 2.0);
                  kd.set(kdkd / 2.0);
               }
               else if (simulatedFingerJoint.name().endsWith("ThumbPitch3"))
               {
                  kp.set(kpkp / 10.0);
                  kd.set(kdkd / 5.0);
               }
               else if (simulatedFingerJoint.name().endsWith("Pitch1"))
               {
                  kp.set(kpkp / 2.0);
                  kd.set(kdkd / 2.0);
               }
               else if (simulatedFingerJoint.name().endsWith("Pitch2"))
               {
                  kp.set(kpkp / 4.0);
                  kd.set(kdkd / 4.0);
               }
               else if (simulatedFingerJoint.name().endsWith("Pitch3"))
               {
                  kp.set(kpkp / 20.0);
                  kd.set(kdkd / 10.0);
               }

               kpMap.get(robotSide).put(simulatedFingerJoint, kp);
               kdMap.get(robotSide).put(simulatedFingerJoint, kd);

               outputs.get(robotSide)
                      .put(simulatedFingerJoint,
                           lowLevelControlOutputs.getJointDesiredOutput(simulatedFingerJoint.getRelatedRevoluteJoint(robotSide, fullRobotModel)));
            }
         }

         handDesiredConfigurationMessageSubscribers.put(robotSide, new HandDesiredConfigurationMessageSubscriber(robotSide));
         if (globalDataProducer != null)
            globalDataProducer.attachListener(HandDesiredConfigurationMessage.class, handDesiredConfigurationMessageSubscribers.get(robotSide));
         fingerSetControllers.put(robotSide, new ValkyrieFingerSetController(robotSide, fingerControllerTime, fingerTrajectoryTime, fullRobotModel,
                                                                             lowLevelControlOutputs, isRunningOnRealRobot, registry, controllerRegistry));

         jointAngleCommunicators.put(robotSide, new HandJointAngleCommunicator(robotSide, globalDataProducer, closeableAndDisposableRegistry));
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         if (!isRunningOnRealRobot)
         {
            for (ValkyrieHandJointName simulatedFingerJoint : ValkyrieHandJointName.values)
            {
               PinJoint relatedPinJoint = simulatedFingerJoint.getRelatedPinJoint(robotSide, simulatedRobot);
               RevoluteJoint relatedRevoluteJoint = (RevoluteJoint) simulatedFingerJoint.getRelatedRevoluteJoint(robotSide, fullRobotModel);
               JointDesiredOutput output = lowLevelControlOutputs.getJointDesiredOutput(relatedRevoluteJoint);

               output.setStiffness(kpMap.get(robotSide).get(simulatedFingerJoint).getDoubleValue());
               output.setDamping(kdMap.get(robotSide).get(simulatedFingerJoint).getDoubleValue());

               //               double fullyExtensonPositionLimit = ValkyrieFingerJointLimits.getFullyExtensonPositionLimit(robotSide, simulatedFingerJoint.getRelatedRealFingerJoint());
               //               double fullyFlexedPositionLimit = ValkyrieFingerJointLimits.getFullyFlexedPositionLimit(robotSide, simulatedFingerJoint.getRelatedRealFingerJoint());
               //
               //               if (fullyExtensonPositionLimit <= fullyFlexedPositionLimit)
               //                  relatedPinJoint.setLimitStops(fullyExtensonPositionLimit - 0.1, fullyFlexedPositionLimit + 0.1, 10.0, 2.5);
               //               else
               //                  relatedPinJoint.setLimitStops(fullyFlexedPositionLimit - 0.1, fullyExtensonPositionLimit + 0.1, 10.0, 2.5);
            }
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
      long timestamp = threadDataSynchronizer.getTimestamp();
      fingerControllerTime.set(Conversions.nanosecondsToSeconds(timestamp));

      for (RobotSide robotSide : RobotSide.values)
      {
         final double[] simulatedJointValues = new double[ValkyrieHandJointName.values.length];

         for (ValkyrieHandJointName jointEnum : ValkyrieHandJointName.values)
         {
            simulatedJointValues[jointEnum.getIndex(robotSide)] = jointEnum.getRelatedRevoluteJoint(robotSide, fullRobotModel).getQ();
         }

         jointAngleCommunicators.get(robotSide).updateHandAngles(new HandSensorData()
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
               return simulatedJointValues;
            }
         });

         jointAngleCommunicators.get(robotSide).write();
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

   private void checkForNewHandDesiredConfigurationRequested()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         if (handDesiredConfigurationMessageSubscribers.get(robotSide).isNewDesiredConfigurationAvailable())
         {
            HandConfiguration handDesiredConfiguration = handDesiredConfigurationMessageSubscribers.get(robotSide).pollMessage().getHandDesiredConfiguration();

            PrintTools.debug(DEBUG, this, "Recieved new HandDesiredConfigurationMessage: " + handDesiredConfiguration);

            switch (handDesiredConfiguration)
            {
            case OPEN:
               fingerSetControllers.get(robotSide).open();
               break;

            case CLOSE:
               fingerSetControllers.get(robotSide).close();
               break;

            default:
               break;
            }
         }
      }
   }

   @Override
   public void write(long timestamp)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         if (!isRunningOnRealRobot)
         {
            for (ValkyrieHandJointName simulatedFingerJoint : ValkyrieHandJointName.values)
            {
               outputs.get(robotSide).get(simulatedFingerJoint).setStiffness(kpMap.get(robotSide).get(simulatedFingerJoint).getDoubleValue());
               outputs.get(robotSide).get(simulatedFingerJoint).setDamping(kdMap.get(robotSide).get(simulatedFingerJoint).getDoubleValue());
            }
         }

         fingerSetControllers.get(robotSide).writeDesiredJointAngles();
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
}
