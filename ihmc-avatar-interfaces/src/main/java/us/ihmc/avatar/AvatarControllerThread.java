package us.ihmc.avatar;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.Supplier;

import controller_msgs.msg.dds.ControllerCrashNotificationPacket;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.initialSetup.RobotInitialSetup;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextData;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextDataFactory;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextJointData;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextTools;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.corruptors.FullRobotModelCorruptor;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.HumanoidHighLevelControllerManager;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelHumanoidControllerFactory;
import us.ihmc.commonWalkingControlModules.visualizer.CommonInertiaEllipsoidsVisualizer;
import us.ihmc.commonWalkingControlModules.visualizer.InverseDynamicsMechanismReferenceFrameVisualizer;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.ControllerCrashLocation;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.model.CenterOfPressureDataHolder;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotController.ModularRobotController;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.sensors.CenterOfMassDataHolder;
import us.ihmc.robotics.sensors.CenterOfMassDataHolderReadOnly;
import us.ihmc.robotics.sensors.ForceSensorDataHolder;
import us.ihmc.robotics.sensors.ForceSensorDataHolderReadOnly;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.sensorProcessing.model.RobotMotionStatus;
import us.ihmc.sensorProcessing.model.RobotMotionStatusChangedListener;
import us.ihmc.sensorProcessing.model.RobotMotionStatusHolder;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListBasics;
import us.ihmc.sensorProcessing.parameters.AvatarRobotLidarParameters;
import us.ihmc.sensorProcessing.parameters.HumanoidRobotSensorInformation;
import us.ihmc.sensorProcessing.simulatedSensors.SensorDataContext;
import us.ihmc.simulationConstructionSetTools.util.visualizers.JointAxisVisualizer;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.wholeBodyController.CenterOfMassCalibrationTool;
import us.ihmc.wholeBodyController.ConstrainedCenterOfMassJacobianEvaluator;
import us.ihmc.wholeBodyController.DRCOutputProcessor;
import us.ihmc.wholeBodyController.WholeBodyControllerParameters;
import us.ihmc.wholeBodyController.parameters.ParameterLoaderHelper;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoLong;

public class AvatarControllerThread implements AvatarControllerThreadInterface
{
   private static final boolean CREATE_DYNAMICALLY_CONSISTENT_NULLSPACE_EVALUATOR = false;
   private static final boolean SHOW_INERTIA_GRAPHICS = false;
   private static final boolean SHOW_REFERENCE_FRAMES = false;
   private static final boolean SHOW_JOINTAXIS_ZALIGN_FRAMES = false;

   private static final boolean CREATE_COM_CALIBRATION_TOOL = false;
   private static final boolean ALLOW_MODEL_CORRUPTION = false;

   private final YoRegistry registry = new YoRegistry("DRCControllerThread");

   private final YoDouble controllerTime = new YoDouble("ControllerTime", registry);
   private final YoLong timestampOffset = new YoLong("TimestampOffsetController", registry);
   private final YoLong timestamp = new YoLong("TimestampController", registry);
   private final YoBoolean firstTick = new YoBoolean("FirstTick", registry);
   private final YoBoolean runController = new YoBoolean("RunController", registry);

   private final FullHumanoidRobotModel controllerFullRobotModel;
   private final FullRobotModelCorruptor fullRobotModelCorruptor;

   private final YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
   private final List<Supplier<YoGraphicDefinition>> scs2YoGraphicHolders = new ArrayList<>();

   private final ModularRobotController robotController;

   private final IHMCRealtimeROS2Publisher<ControllerCrashNotificationPacket> crashNotificationPublisher;

   private final HumanoidRobotContextData humanoidRobotContextData;

   private final ExecutionTimer controllerThreadTimer;

   public AvatarControllerThread(String robotName,
                                 DRCRobotModel robotModel,
                                 RobotInitialSetup<?> robotInitialSetup,
                                 HumanoidRobotSensorInformation sensorInformation,
                                 HighLevelHumanoidControllerFactory controllerFactory,
                                 HumanoidRobotContextDataFactory contextDataFactory,
                                 DRCOutputProcessor outputProcessor,
                                 RealtimeROS2Node realtimeROS2Node,
                                 double gravity)
   {
      controllerFullRobotModel = robotModel.createFullRobotModel();
      if (robotInitialSetup != null)
      {
         robotInitialSetup.initializeFullRobotModel(controllerFullRobotModel);
      }

      HumanoidRobotContextJointData processedJointData = new HumanoidRobotContextJointData(controllerFullRobotModel.getOneDoFJoints().length);
      ForceSensorDataHolder forceSensorDataHolderForController = new ForceSensorDataHolder(Arrays.asList(controllerFullRobotModel.getForceSensorDefinitions()));
      CenterOfMassDataHolder centerOfMassDataHolderForController = new CenterOfMassDataHolder();
      CenterOfPressureDataHolder centerOfPressureDataHolderForEstimator = new CenterOfPressureDataHolder(controllerFullRobotModel);
      LowLevelOneDoFJointDesiredDataHolder desiredJointDataHolder = new LowLevelOneDoFJointDesiredDataHolder(controllerFullRobotModel.getControllableOneDoFJoints());
      RobotMotionStatusHolder robotMotionStatusHolder = new RobotMotionStatusHolder();
      contextDataFactory.setForceSensorDataHolder(forceSensorDataHolderForController);
      contextDataFactory.setCenterOfMassDataHolder(centerOfMassDataHolderForController);
      contextDataFactory.setCenterOfPressureDataHolder(centerOfPressureDataHolderForEstimator);
      contextDataFactory.setRobotMotionStatusHolder(robotMotionStatusHolder);
      contextDataFactory.setJointDesiredOutputList(desiredJointDataHolder);
      contextDataFactory.setProcessedJointData(processedJointData);
      contextDataFactory.setSensorDataContext(new SensorDataContext(controllerFullRobotModel));
      humanoidRobotContextData = contextDataFactory.createHumanoidRobotContextData();

      if (realtimeROS2Node != null)
      {
         crashNotificationPublisher = ROS2Tools.createPublisher(realtimeROS2Node,
                                                                ControllerAPIDefinition.getTopic(ControllerCrashNotificationPacket.class, robotName));
      }
      else
      {
         crashNotificationPublisher = null;
      }

      if (ALLOW_MODEL_CORRUPTION)
      {
         fullRobotModelCorruptor = new FullRobotModelCorruptor(controllerFullRobotModel, registry);
      }
      else
      {
         fullRobotModelCorruptor = null;
      }

      JointBasics[] arrayOfJointsToIgnore = createListOfJointsToIgnore(controllerFullRobotModel, robotModel, sensorInformation);

      robotController = createHighLevelController(controllerFullRobotModel,
                                                  controllerFactory,
                                                  controllerTime,
                                                  robotModel.getControllerDT(),
                                                  gravity,
                                                  forceSensorDataHolderForController,
                                                  centerOfMassDataHolderForController,
                                                  centerOfPressureDataHolderForEstimator,
                                                  sensorInformation,
                                                  desiredJointDataHolder,
                                                  yoGraphicsListRegistry,
                                                  registry,
                                                  arrayOfJointsToIgnore);

      createControllerRobotMotionStatusUpdater(controllerFactory, robotMotionStatusHolder);

      controllerThreadTimer = new ExecutionTimer("controllerThreadTimer", registry);

      firstTick.set(true);
      registry.addChild(robotController.getYoRegistry());
      if (outputProcessor != null)
      {
         outputProcessor.setLowLevelControllerCoreOutput(processedJointData, desiredJointDataHolder);
         outputProcessor.setForceSensorDataHolderForController(forceSensorDataHolderForController);
         registry.addChild(outputProcessor.getControllerYoVariableRegistry());
      }

      ParameterLoaderHelper.loadParameters(this, robotModel, registry);
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

   private ModularRobotController createHighLevelController(FullHumanoidRobotModel controllerModel,
                                                            HighLevelHumanoidControllerFactory controllerFactory,
                                                            YoDouble yoTime,
                                                            double controlDT,
                                                            double gravity,
                                                            ForceSensorDataHolderReadOnly forceSensorDataHolderForController,
                                                            CenterOfMassDataHolderReadOnly centerOfMassDataHolderForController,
                                                            CenterOfPressureDataHolder centerOfPressureDataHolderForEstimator,
                                                            HumanoidRobotSensorInformation sensorInformation,
                                                            JointDesiredOutputListBasics lowLevelControllerOutput,
                                                            YoGraphicsListRegistry yoGraphicsListRegistry,
                                                            YoRegistry registry,
                                                            JointBasics... jointsToIgnore)
   {
      if (CREATE_COM_CALIBRATION_TOOL)
      {
         try
         {
            CenterOfMassCalibrationTool centerOfMassCalibrationTool = new CenterOfMassCalibrationTool(controllerModel,
                                                                                                      forceSensorDataHolderForController,
                                                                                                      yoGraphicsListRegistry,
                                                                                                      registry);
            controllerFactory.addUpdatable(centerOfMassCalibrationTool);
         }
         catch (Exception e)
         {
            System.err.println("Couldn't create CenterOfMassCalibrationTool");
         }
      }

      HumanoidHighLevelControllerManager robotController = controllerFactory.getController(controllerModel,
                                                                                           controlDT,
                                                                                           gravity,
                                                                                           yoTime,
                                                                                           yoGraphicsListRegistry,
                                                                                           sensorInformation,
                                                                                           forceSensorDataHolderForController,
                                                                                           centerOfMassDataHolderForController,
                                                                                           centerOfPressureDataHolderForEstimator,
                                                                                           lowLevelControllerOutput,
                                                                                           jointsToIgnore);
      scs2YoGraphicHolders.add(() -> robotController.getSCS2YoGraphics());

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

   public void initialize()
   {
      firstTick.set(true);
      humanoidRobotContextData.setControllerRan(false);
      humanoidRobotContextData.setEstimatorRan(false);

      LowLevelOneDoFJointDesiredDataHolder jointDesiredOutputList = humanoidRobotContextData.getJointDesiredOutputList();

      for (int i = 0; i < jointDesiredOutputList.getNumberOfJointsWithDesiredOutput(); i++)
      {
         jointDesiredOutputList.getJointDesiredOutput(i).clear();
      }
   }

   @Override
   public void run()
   {
      controllerThreadTimer.startMeasurement();

      runController.set(humanoidRobotContextData.getEstimatorRan());
      if (!runController.getValue())
      {
         return;
      }

      try
      {
         HumanoidRobotContextTools.updateRobot(controllerFullRobotModel, humanoidRobotContextData.getProcessedJointData());
         timestamp.set(humanoidRobotContextData.getTimestamp());
         if (firstTick.getValue())
         {
            // Record this to have time start at 0.0 on the real robot for viewing pleasure.
            timestampOffset.set(timestamp.getValue());
         }
         controllerTime.set(Conversions.nanosecondsToSeconds(timestamp.getValue() - timestampOffset.getValue()));

         if (firstTick.getValue())
         {
            robotController.initialize();
            firstTick.set(false);
         }

         robotController.doControl();
         humanoidRobotContextData.setControllerRan(true);
      }
      catch (Exception e)
      {
         if (crashNotificationPublisher != null)
         {
            crashNotificationPublisher.publish(MessageTools.createControllerCrashNotificationPacket(ControllerCrashLocation.CONTROLLER_RUN, e));
         }

         throw new RuntimeException(e);
      }

      controllerThreadTimer.stopMeasurement();
   }

   @Override
   public YoRegistry getYoVariableRegistry()
   {
      return registry;
   }

   public YoGraphicsListRegistry getSCS1YoGraphicsListRegistry()
   {
      return yoGraphicsListRegistry;
   }

   @Override
   public YoGraphicGroupDefinition getSCS2YoGraphics()
   {
      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(getClass().getSimpleName());
      for (int i = 0; i < scs2YoGraphicHolders.size(); i++)
      {
         group.addChild(scs2YoGraphicHolders.get(i).get());
      }
      return group;
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

   @Override
   public FullHumanoidRobotModel getFullRobotModel()
   {
      return controllerFullRobotModel;
   }

   @Override
   public HumanoidRobotContextData getHumanoidRobotContextData()
   {
      return humanoidRobotContextData;
   }

   public JointDesiredOutputListBasics getDesiredJointDataHolder()
   {
      return humanoidRobotContextData.getJointDesiredOutputList();
   }

}
