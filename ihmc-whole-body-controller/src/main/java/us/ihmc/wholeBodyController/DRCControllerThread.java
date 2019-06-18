package us.ihmc.wholeBodyController;

import java.util.ArrayList;

import controller_msgs.msg.dds.ControllerCrashNotificationPacket;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextJointData;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextTools;
import us.ihmc.commonWalkingControlModules.corruptors.FullRobotModelCorruptor;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelHumanoidControllerFactory;
import us.ihmc.commonWalkingControlModules.visualizer.CommonInertiaEllipsoidsVisualizer;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.ControllerCrashLocation;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.model.CenterOfPressureDataHolder;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotDataLogger.RobotVisualizer;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.robotController.ModularRobotController;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.sensors.ForceSensorDataHolderReadOnly;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.ros2.RealtimeRos2Node;
import us.ihmc.sensorProcessing.model.RobotMotionStatus;
import us.ihmc.sensorProcessing.model.RobotMotionStatusChangedListener;
import us.ihmc.sensorProcessing.model.RobotMotionStatusHolder;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListBasics;
import us.ihmc.sensorProcessing.parameters.AvatarRobotLidarParameters;
import us.ihmc.sensorProcessing.parameters.HumanoidRobotSensorInformation;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;
import us.ihmc.simulationConstructionSetTools.robotController.MultiThreadedRobotControlElement;
import us.ihmc.simulationConstructionSetTools.util.visualizers.InverseDynamicsMechanismReferenceFrameVisualizer;
import us.ihmc.simulationConstructionSetTools.util.visualizers.JointAxisVisualizer;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.tools.thread.CloseableAndDisposableRegistry;
import us.ihmc.wholeBodyController.concurrent.ThreadDataSynchronizerInterface;
import us.ihmc.wholeBodyController.parameters.ParameterLoaderHelper;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoLong;

public class DRCControllerThread implements MultiThreadedRobotControlElement
{
   private static final boolean CREATE_DYNAMICALLY_CONSISTENT_NULLSPACE_EVALUATOR = false;
   private static final boolean SHOW_INERTIA_GRAPHICS = false;
   private static final boolean SHOW_REFERENCE_FRAMES = false;
   private static final boolean SHOW_JOINTAXIS_ZALIGN_FRAMES = false;

   private static final boolean CREATE_COM_CALIBRATION_TOOL = false;
   private static final boolean ALLOW_MODEL_CORRUPTION = true;

   private final YoVariableRegistry registry = new YoVariableRegistry("DRCControllerThread");
   private final RobotVisualizer robotVisualizer;

   private final long controlDTInNS;
   private final long estimatorDTInNS;
   private final long estimatorTicksPerControlTick;

   private final YoDouble controllerTime = new YoDouble("controllerTime", registry);
   private final YoLong controllerTimestamp = new YoLong("controllerTimestamp", registry);
   private final YoBoolean firstTick = new YoBoolean("firstTick", registry);

   private final FullHumanoidRobotModel controllerFullRobotModel;
   private final FullRobotModelCorruptor fullRobotModelCorruptor;

   private final YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
   private final ForceSensorDataHolderReadOnly forceSensorDataHolderForController;
   private final CenterOfPressureDataHolder centerOfPressureDataHolderForEstimator;

   private final ThreadDataSynchronizerInterface threadDataSynchronizer;
   private final DRCOutputProcessor outputProcessor;

   private final ModularRobotController robotController;

   private final ExecutionTimer robotVisualizerUpdateTimer;
   private final ExecutionTimer controllerTimer = new ExecutionTimer("controllerTimer", 10.0, registry);
   private final YoLong lastEstimatorStartTime = new YoLong("nextExecutionTime", registry);
   private final YoLong totalDelay = new YoLong("totalDelay", registry);
   private final YoLong expectedEstimatorTick = new YoLong("expectedEstimatorTick", registry);
   private final YoLong controllerLeadsEstimatorTicks = new YoLong("controllerLeadsEstimatorTicks", registry);
   private final YoLong controllerLagsEstimatorTicks = new YoLong("controllerLagsEstimatorTicks", registry);

   /*
    * Debug variables
    */
   private final YoLong lastExpectedEstimatorTick = new YoLong("lastExpectedEstimatorTick", registry);
   private final YoLong lastEstimatorTick = new YoLong("lastEstimatorTick", registry);
   private final YoLong lastEstimatorClockStartTime = new YoLong("lastEstimatorClockStartTime", registry);
   private final YoLong lastControllerClockTime = new YoLong("lastControllerClockTime", registry);
   private final YoLong controllerStartTime = new YoLong("controllerStartTime", registry);
   private final YoLong timePassedSinceEstimator = new YoLong("timePassedSinceEstimator", registry);
   private final YoLong timePassedBetweenEstimatorTicks = new YoLong("timePassedBetweenEstimatorTicks", registry);

   private long lastReadSystemTime = 0L;
   private final YoDouble actualControlDT = new YoDouble("actualControlDTInMillis", registry);

   private final YoBoolean runController = new YoBoolean("runController", registry);

   private final RigidBodyTransform rootToWorldTransform = new RigidBodyTransform();
   private final ReferenceFrame rootFrame;
   private final CloseableAndDisposableRegistry closeableAndDisposableRegistry = new CloseableAndDisposableRegistry();

   private final IHMCRealtimeROS2Publisher<ControllerCrashNotificationPacket> crashNotificationPublisher;

   private final HumanoidRobotContextJointData proccessedJointData = new HumanoidRobotContextJointData();

   public DRCControllerThread(String robotName, WholeBodyControllerParameters<RobotSide> robotModel, HumanoidRobotSensorInformation sensorInformation,
                              HighLevelHumanoidControllerFactory controllerFactory, ThreadDataSynchronizerInterface threadDataSynchronizer,
                              DRCOutputProcessor outputProcessor, RealtimeRos2Node realtimeRos2Node, RobotVisualizer robotVisualizer, double gravity,
                              double estimatorDT)
   {
      this.threadDataSynchronizer = threadDataSynchronizer;
      this.outputProcessor = outputProcessor;
      this.robotVisualizer = robotVisualizer;
      this.controlDTInNS = Conversions.secondsToNanoseconds(robotModel.getControllerDT());
      this.estimatorDTInNS = Conversions.secondsToNanoseconds(estimatorDT);
      this.estimatorTicksPerControlTick = this.controlDTInNS / this.estimatorDTInNS;
      this.controllerFullRobotModel = threadDataSynchronizer.getControllerFullRobotModel();
      this.rootFrame = this.controllerFullRobotModel.getRootJoint().getFrameAfterJoint();

      closeableAndDisposableRegistry.registerCloseableAndDisposable(controllerFactory);

      crashNotificationPublisher = ROS2Tools.createPublisher(realtimeRos2Node, ControllerCrashNotificationPacket.class,
                                                             ControllerAPIDefinition.getPublisherTopicNameGenerator(robotName));

      if (ALLOW_MODEL_CORRUPTION)
      {
         fullRobotModelCorruptor = new FullRobotModelCorruptor(controllerFullRobotModel, registry);
      }
      else
      {
         fullRobotModelCorruptor = null;
      }

      forceSensorDataHolderForController = threadDataSynchronizer.getControllerForceSensorDataHolder();
      centerOfPressureDataHolderForEstimator = threadDataSynchronizer.getControllerCenterOfPressureDataHolder();

      JointBasics[] arrayOfJointsToIgnore = createListOfJointsToIgnore(controllerFullRobotModel, robotModel, sensorInformation);

      robotController = createHighLevelController(controllerFullRobotModel, controllerFactory, controllerTime, robotModel.getControllerDT(), gravity,
                                                  forceSensorDataHolderForController, centerOfPressureDataHolderForEstimator, sensorInformation,
                                                  threadDataSynchronizer.getControllerDesiredJointDataHolder(), yoGraphicsListRegistry, registry,
                                                  arrayOfJointsToIgnore);

      createControllerRobotMotionStatusUpdater(controllerFactory, threadDataSynchronizer.getControllerRobotMotionStatusHolder());

      firstTick.set(true);
      registry.addChild(robotController.getYoVariableRegistry());
      if (outputProcessor != null)
      {
         outputProcessor.setLowLevelControllerCoreOutput(proccessedJointData, threadDataSynchronizer.getControllerDesiredJointDataHolder());
         outputProcessor.setForceSensorDataHolderForController(forceSensorDataHolderForController);
         registry.addChild(outputProcessor.getControllerYoVariableRegistry());
      }

      lastEstimatorStartTime.set(Long.MIN_VALUE);
      expectedEstimatorTick.set(estimatorDTInNS);

      ParameterLoaderHelper.loadParameters(this, robotModel, registry);

      if (robotVisualizer != null)
      {
         robotVisualizerUpdateTimer = new ExecutionTimer("robotVisualizerUpdateTimer", 10.0, registry);
         robotVisualizer.addRegistry(registry, yoGraphicsListRegistry);
      }
      else
      {
         robotVisualizerUpdateTimer = null;
      }
   }

   public static JointBasics[] createListOfJointsToIgnore(FullHumanoidRobotModel controllerFullRobotModel,
                                                                   WholeBodyControllerParameters<RobotSide> robotModel,
                                                                   HumanoidRobotSensorInformation sensorInformation)
   {
      ArrayList<JointBasics> listOfJointsToIgnore = new ArrayList<>();

      AvatarRobotLidarParameters lidarParameters = sensorInformation.getLidarParameters(0);
      if (lidarParameters != null)
      {
         listOfJointsToIgnore.add(controllerFullRobotModel.getOneDoFJointByName(lidarParameters.getLidarSpindleJointName()));
      }

      String[] additionalJointsToIgnore = robotModel.getWalkingControllerParameters().getJointsToIgnoreInController();
      if (additionalJointsToIgnore != null)
      {
         for (String jointToIgnore : additionalJointsToIgnore)
         {
            listOfJointsToIgnore.add(controllerFullRobotModel.getOneDoFJointByName(jointToIgnore));
         }
      }
      JointBasics[] arrayOfJointsToIgnore = listOfJointsToIgnore.toArray(new JointBasics[] {});
      return arrayOfJointsToIgnore;
   }

   private void createControllerRobotMotionStatusUpdater(HighLevelHumanoidControllerFactory controllerFactory,
                                                         final RobotMotionStatusHolder controllerRobotMotionStatusHolder)
   {
      RobotMotionStatusChangedListener controllerRobotMotionStatusUpdater = new RobotMotionStatusChangedListener()
      {
         @Override
         public void robotMotionStatusHasChanged(RobotMotionStatus newStatus, double time)
         {
            controllerRobotMotionStatusHolder.setCurrentRobotMotionStatus(newStatus);
         }
      };

      controllerFactory.attachRobotMotionStatusChangedListener(controllerRobotMotionStatusUpdater);
   }

   public FullRobotModelCorruptor getFullRobotModelCorruptor()
   {
      return fullRobotModelCorruptor;
   }

   private ModularRobotController createHighLevelController(FullHumanoidRobotModel controllerModel, HighLevelHumanoidControllerFactory controllerFactory,
                                                            YoDouble yoTime, double controlDT, double gravity,
                                                            ForceSensorDataHolderReadOnly forceSensorDataHolderForController,
                                                            CenterOfPressureDataHolder centerOfPressureDataHolderForEstimator,
                                                            HumanoidRobotSensorInformation sensorInformation, JointDesiredOutputListBasics lowLevelControllerOutput,
                                                            YoGraphicsListRegistry yoGraphicsListRegistry, YoVariableRegistry registry,
                                                            JointBasics... jointsToIgnore)
   {
      if (CREATE_COM_CALIBRATION_TOOL)
      {
         try
         {
            CenterOfMassCalibrationTool centerOfMassCalibrationTool = new CenterOfMassCalibrationTool(controllerModel, forceSensorDataHolderForController,
                                                                                                      yoGraphicsListRegistry, registry);
            controllerFactory.addUpdatable(centerOfMassCalibrationTool);
         }
         catch (Exception e)
         {
            System.err.println("Couldn't create CenterOfMassCalibrationTool");
         }
      }

      RobotController robotController = controllerFactory.getController(controllerModel, controlDT, gravity, yoTime, yoGraphicsListRegistry, sensorInformation,
                                                                        forceSensorDataHolderForController,
                                                                        centerOfPressureDataHolderForEstimator, lowLevelControllerOutput, jointsToIgnore);

      ModularRobotController modularRobotController = new ModularRobotController("DRCMomentumBasedController");
      modularRobotController.addRobotController(robotController);

      if (yoGraphicsListRegistry != null)
      {
         if (SHOW_INERTIA_GRAPHICS)
         {
            CommonInertiaEllipsoidsVisualizer commonInertiaElipsoidsVisualizer = new CommonInertiaEllipsoidsVisualizer(controllerModel.getElevator(),
                                                                                                                       yoGraphicsListRegistry);
            modularRobotController.addRobotController(commonInertiaElipsoidsVisualizer);
         }

         if (SHOW_REFERENCE_FRAMES)
         {
            InverseDynamicsMechanismReferenceFrameVisualizer inverseDynamicsMechanismReferenceFrameVisualizer = new InverseDynamicsMechanismReferenceFrameVisualizer(controllerModel.getElevator(),
                                                                                                                                                                     yoGraphicsListRegistry,
                                                                                                                                                                     0.5);
            modularRobotController.addRobotController(inverseDynamicsMechanismReferenceFrameVisualizer);
         }

         if (SHOW_JOINTAXIS_ZALIGN_FRAMES)
         {
            JointAxisVisualizer jointAxisVisualizer = new JointAxisVisualizer(controllerModel.getElevator(), yoGraphicsListRegistry, 0.3);
            modularRobotController.addRobotController(jointAxisVisualizer);
         }
      }

      if (CREATE_DYNAMICALLY_CONSISTENT_NULLSPACE_EVALUATOR)
      {
         RobotController dynamicallyConsistentNullspaceEvaluator = new ConstrainedCenterOfMassJacobianEvaluator(controllerModel);
         modularRobotController.addRobotController(dynamicallyConsistentNullspaceEvaluator);
      }

      return modularRobotController;
   }

   public static FullInverseDynamicsStructure createInverseDynamicsStructure(FullRobotModel fullRobotModel)
   {
      RigidBodyBasics elevator = fullRobotModel.getElevator();
      FloatingJointBasics rootInverseDynamicsJoint = fullRobotModel.getRootJoint();
      RigidBodyBasics estimationLink = fullRobotModel.getRootBody();

      FullInverseDynamicsStructure inverseDynamicsStructure = new FullInverseDynamicsStructure(elevator, estimationLink, rootInverseDynamicsJoint);

      return inverseDynamicsStructure;
   }

   @Override
   public void initialize()
   {
   }

   @Override
   public void read(long currentClockTime)
   {
      try
      {
         runController.set(threadDataSynchronizer.receiveEstimatorStateForController());

         if (runController.getBooleanValue())
         {
            // Skip the first estimatorTicksPerControlTick estimator ticks
            if (threadDataSynchronizer.getEstimatorTick() < estimatorTicksPerControlTick)
            {
               runController.set(false);
            }
            else
            {
               long nanoTime = System.nanoTime();
               actualControlDT.set(Conversions.nanosecondsToMilliseconds((double) (nanoTime - lastReadSystemTime)));
               lastReadSystemTime = nanoTime;

               long estimatorStartTime = threadDataSynchronizer.getEstimatorClockStartTime();
               controllerTimestamp.set(threadDataSynchronizer.getTimestamp());
               controllerTime.set(Conversions.nanosecondsToSeconds(controllerTimestamp.getLongValue()));

               if (expectedEstimatorTick.getLongValue() != threadDataSynchronizer.getEstimatorTick())
               {
                  if (expectedEstimatorTick.getLongValue() > threadDataSynchronizer.getEstimatorTick())
                  {
                     controllerLeadsEstimatorTicks.increment();
                  }
                  else if (expectedEstimatorTick.getLongValue() < threadDataSynchronizer.getEstimatorTick())
                  {
                     controllerLagsEstimatorTicks.increment();
                  }

                  lastExpectedEstimatorTick.set(expectedEstimatorTick.getLongValue());
                  lastEstimatorTick.set(threadDataSynchronizer.getEstimatorTick());
                  lastEstimatorClockStartTime.set(threadDataSynchronizer.getEstimatorClockStartTime());
                  lastControllerClockTime.set(currentClockTime);
                  timePassedSinceEstimator.set(currentClockTime - lastEstimatorStartTime.getLongValue());
                  timePassedBetweenEstimatorTicks.set(estimatorStartTime - lastEstimatorStartTime.getLongValue());
               }

               expectedEstimatorTick.set(threadDataSynchronizer.getEstimatorTick() + estimatorTicksPerControlTick);
               controllerStartTime.set(currentClockTime);
               lastEstimatorStartTime.set(estimatorStartTime);
            }
         }
      }
      catch (Exception e)
      {
         crashNotificationPublisher.publish(MessageTools.createControllerCrashNotificationPacket(ControllerCrashLocation.CONTROLLER_READ, e.getMessage()));

         throw new RuntimeException(e);
      }
   }

   @Override
   public void run()
   {
      try
      {
         if (runController.getBooleanValue())
         {
            if (firstTick.getBooleanValue())
            {
               robotController.initialize();
               if (outputProcessor != null)
               {
                  HumanoidRobotContextTools.updateContext(controllerFullRobotModel, proccessedJointData);
                  outputProcessor.initialize();
               }
               firstTick.set(false);
            }
            controllerTimer.startMeasurement();
            robotController.doControl();
            controllerTimer.stopMeasurement();
         }
      }
      catch (Exception e)
      {
         crashNotificationPublisher.publish(MessageTools.createControllerCrashNotificationPacket(ControllerCrashLocation.CONTROLLER_RUN, e.getMessage()));

         throw new RuntimeException(e);
      }
   }

   @Override
   public void write(long timestamp)
   {
      try
      {
         if (runController.getBooleanValue())
         {
            if (outputProcessor != null)
            {
               HumanoidRobotContextTools.updateContext(controllerFullRobotModel, proccessedJointData);
               outputProcessor.processAfterController(controllerTimestamp.getLongValue());
            }
            totalDelay.set(timestamp - lastEstimatorStartTime.getLongValue());

            threadDataSynchronizer.publishControllerData();
            if (robotVisualizer != null)
            {
               robotVisualizerUpdateTimer.startMeasurement();
               robotVisualizer.update(controllerTimestamp.getLongValue(), registry);
               robotVisualizerUpdateTimer.stopMeasurement();
            }

            rootFrame.getTransformToDesiredFrame(rootToWorldTransform, ReferenceFrame.getWorldFrame());
            yoGraphicsListRegistry.setControllerTransformToWorld(rootToWorldTransform);
         }

      }
      catch (Exception e)
      {
         crashNotificationPublisher.publish(MessageTools.createControllerCrashNotificationPacket(ControllerCrashLocation.CONTROLLER_WRITE, e.getMessage()));
         throw new RuntimeException(e);

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
      return getClass().getSimpleName();
   }

   @Override
   public YoGraphicsListRegistry getYoGraphicsListRegistry()
   {
      return yoGraphicsListRegistry;
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

   /**
    * For unit testing only
    *
    * @param controller
    */
   public void addRobotController(RobotController controller)
   {
      robotController.addRobotController(controller);
   }

   public FullRobotModel getFullRobotModel()
   {
      return controllerFullRobotModel;
   }

   public void dispose()
   {
      closeableAndDisposableRegistry.closeAndDispose();
   }

}
