package us.ihmc.avatar;

import java.util.ArrayList;
import java.util.Arrays;

import controller_msgs.msg.dds.ControllerCrashNotificationPacket;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextData;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextDataFactory;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextJointData;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextTools;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.corruptors.FullRobotModelCorruptor;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelHumanoidControllerFactory;
import us.ihmc.commonWalkingControlModules.visualizer.CommonInertiaEllipsoidsVisualizer;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.ControllerCrashLocation;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.model.CenterOfPressureDataHolder;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.robotController.ModularRobotController;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.sensors.ForceSensorDataHolder;
import us.ihmc.robotics.sensors.ForceSensorDataHolderReadOnly;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.ros2.RealtimeRos2Node;
import us.ihmc.sensorProcessing.model.RobotMotionStatus;
import us.ihmc.sensorProcessing.model.RobotMotionStatusChangedListener;
import us.ihmc.sensorProcessing.model.RobotMotionStatusHolder;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListBasics;
import us.ihmc.sensorProcessing.parameters.DRCRobotLidarParameters;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorInformation;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;
import us.ihmc.simulationConstructionSetTools.util.visualizers.InverseDynamicsMechanismReferenceFrameVisualizer;
import us.ihmc.simulationConstructionSetTools.util.visualizers.JointAxisVisualizer;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.wholeBodyController.CenterOfMassCalibrationTool;
import us.ihmc.wholeBodyController.ConstrainedCenterOfMassJacobianEvaluator;
import us.ihmc.wholeBodyController.DRCOutputProcessor;
import us.ihmc.wholeBodyController.WholeBodyControllerParameters;
import us.ihmc.wholeBodyController.parameters.ParameterLoaderHelper;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoLong;

public class AvatarControllerThread
{
   private static final boolean CREATE_DYNAMICALLY_CONSISTENT_NULLSPACE_EVALUATOR = false;
   private static final boolean SHOW_INERTIA_GRAPHICS = false;
   private static final boolean SHOW_REFERENCE_FRAMES = false;
   private static final boolean SHOW_JOINTAXIS_ZALIGN_FRAMES = false;

   private static final boolean CREATE_COM_CALIBRATION_TOOL = false;
   private static final boolean ALLOW_MODEL_CORRUPTION = true;

   private final YoVariableRegistry registry = new YoVariableRegistry("DRCControllerThread");

   private final YoDouble controllerTime = new YoDouble("controllerTime", registry);
   private final YoLong controllerTimestamp = new YoLong("controllerTimestamp", registry);
   private final YoBoolean firstTick = new YoBoolean("firstTick", registry);

   private final FullHumanoidRobotModel controllerFullRobotModel;
   private final FullRobotModelCorruptor fullRobotModelCorruptor;

   private final YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

   private final DRCOutputProcessor outputProcessor;

   private final ModularRobotController robotController;

   private final ExecutionTimer controllerTimer = new ExecutionTimer("controllerTimer", 10.0, registry);

   private final YoBoolean runController = new YoBoolean("runController", registry);

   private final IHMCRealtimeROS2Publisher<ControllerCrashNotificationPacket> crashNotificationPublisher;

   private final HumanoidRobotContextData humanoidRobotContextData;

   public AvatarControllerThread(String robotName, DRCRobotModel robotModel, DRCRobotSensorInformation sensorInformation,
                                 HighLevelHumanoidControllerFactory controllerFactory, HumanoidRobotContextDataFactory contextDataFactory,
                                 DRCOutputProcessor outputProcessor, RealtimeRos2Node realtimeRos2Node, double gravity, double estimatorDT)
   {
      this.outputProcessor = outputProcessor;
      this.controllerFullRobotModel = robotModel.createFullRobotModel();

      HumanoidRobotContextJointData processedJointData = new HumanoidRobotContextJointData(controllerFullRobotModel.getOneDoFJoints().length);
      ForceSensorDataHolder forceSensorDataHolderForController = new ForceSensorDataHolder(Arrays.asList(controllerFullRobotModel.getForceSensorDefinitions()));
      CenterOfPressureDataHolder centerOfPressureDataHolderForEstimator = new CenterOfPressureDataHolder(controllerFullRobotModel);
      LowLevelOneDoFJointDesiredDataHolder desiredJointDataHolder = new LowLevelOneDoFJointDesiredDataHolder(controllerFullRobotModel.getControllableOneDoFJoints());
      RobotMotionStatusHolder robotMotionStatusHolder = new RobotMotionStatusHolder();
      contextDataFactory.setForceSensorDataHolder(forceSensorDataHolderForController);
      contextDataFactory.setCenterOfPressureDataHolder(centerOfPressureDataHolderForEstimator);
      contextDataFactory.setRobotMotionStatusHolder(robotMotionStatusHolder);
      contextDataFactory.setJointDesiredOutputList(desiredJointDataHolder);
      contextDataFactory.setProcessedJointData(processedJointData);
      humanoidRobotContextData = contextDataFactory.createHumanoidRobotContextData();

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


      JointBasics[] arrayOfJointsToIgnore = createListOfJointsToIgnore(controllerFullRobotModel, robotModel, sensorInformation);

      robotController = createHighLevelController(controllerFullRobotModel, controllerFactory, controllerTime, robotModel.getControllerDT(), gravity,
                                                  forceSensorDataHolderForController, centerOfPressureDataHolderForEstimator, sensorInformation,
                                                  desiredJointDataHolder, yoGraphicsListRegistry, registry, arrayOfJointsToIgnore);

      createControllerRobotMotionStatusUpdater(controllerFactory, robotMotionStatusHolder);

      firstTick.set(true);
      registry.addChild(robotController.getYoVariableRegistry());
      if (outputProcessor != null)
      {
         outputProcessor.setLowLevelControllerCoreOutput(controllerFullRobotModel, desiredJointDataHolder);
         outputProcessor.setForceSensorDataHolderForController(forceSensorDataHolderForController);
         registry.addChild(outputProcessor.getControllerYoVariableRegistry());
      }

      ParameterLoaderHelper.loadParameters(this, robotModel, registry);
   }

   public static JointBasics[] createListOfJointsToIgnore(FullHumanoidRobotModel controllerFullRobotModel, WholeBodyControllerParameters<RobotSide> robotModel,
                                                          DRCRobotSensorInformation sensorInformation)
   {
      ArrayList<JointBasics> listOfJointsToIgnore = new ArrayList<>();

      DRCRobotLidarParameters lidarParameters = sensorInformation.getLidarParameters(0);
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
                                                            DRCRobotSensorInformation sensorInformation, JointDesiredOutputListBasics lowLevelControllerOutput,
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
                                                                        forceSensorDataHolderForController, centerOfPressureDataHolderForEstimator,
                                                                        lowLevelControllerOutput, jointsToIgnore);

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

   public void read()
   {
   }

   public void run()
   {
      runController.set(humanoidRobotContextData.getEstimatorRan());
      if (!runController.getValue())
      {
         return;
      }

      try
      {
         HumanoidRobotContextTools.updateRobot(controllerFullRobotModel, humanoidRobotContextData.getProcessedJointData());
         controllerTimestamp.set(humanoidRobotContextData.getTimestamp());
         controllerTime.set(Conversions.nanosecondsToSeconds(controllerTimestamp.getLongValue()));

         if (firstTick.getBooleanValue())
         {
            robotController.initialize();
            if (outputProcessor != null)
            {
               outputProcessor.initialize();
            }
            firstTick.set(false);
         }

         controllerTimer.startMeasurement();
         robotController.doControl();
         humanoidRobotContextData.setControllerRan(true);
         controllerTimer.stopMeasurement();
      }
      catch (Exception e)
      {
         crashNotificationPublisher.publish(MessageTools.createControllerCrashNotificationPacket(ControllerCrashLocation.CONTROLLER_RUN, e.getMessage()));

         throw new RuntimeException(e);
      }
   }

   public void write()
   {
      if (!runController.getValue())
      {
         return;
      }

      try
      {
         if (outputProcessor != null)
         {
            outputProcessor.processAfterController(controllerTimestamp.getLongValue());
         }
      }
      catch (Exception e)
      {
         crashNotificationPublisher.publish(MessageTools.createControllerCrashNotificationPacket(ControllerCrashLocation.CONTROLLER_WRITE, e.getMessage()));
         throw new RuntimeException(e);

      }
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   public YoGraphicsListRegistry getYoGraphicsListRegistry()
   {
      return yoGraphicsListRegistry;
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

   public FullHumanoidRobotModel getFullRobotModel()
   {
      return controllerFullRobotModel;
   }

   public HumanoidRobotContextData getHumanoidRobotContextData()
   {
      return humanoidRobotContextData;
   }

   public JointDesiredOutputListBasics getDesiredJointDataHolder()
   {
      return humanoidRobotContextData.getJointDesiredOutputList();
   }

}
