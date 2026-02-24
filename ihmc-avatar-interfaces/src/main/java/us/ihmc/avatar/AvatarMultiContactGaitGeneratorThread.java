package us.ihmc.avatar;

import controller_msgs.msg.dds.CapturabilityBasedStatus;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.multiContact.pushRecovery.ReactiveBracingPlanner;
import us.ihmc.avatar.multiContact.pushRecovery.ReducedOrderRobotModel;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextData;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextDataFactory;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextJointData;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextTools;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.ControllerNetworkSubscriber;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning.BipedTimedStep;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.MultiContactGaitGenerator;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.HandContactCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PlanarRegionsListCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.TerrainMapCommand;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.humanoidRobotics.model.CenterOfPressureDataHolder;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.algorithms.CenterOfMassJacobian;
import us.ihmc.perception.gpuMapping.HeightMapTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.sensors.CenterOfMassDataHolder;
import us.ihmc.robotics.sensors.ForceSensorDataHolder;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory.DefaultPoint2DGraphic;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.sensorProcessing.model.RobotMotionStatusHolder;
import us.ihmc.sensorProcessing.simulatedSensors.SensorDataContext;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector2D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

public class AvatarMultiContactGaitGeneratorThread implements AvatarControllerThreadInterface
{
   private static final boolean VISUALIZE_HEIGHT_MAP = false;
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final double dt;
   private final FullHumanoidRobotModel fullRobotModel;
   private final HumanoidRobotContextData humanoidRobotContextData;
   private final CenterOfPressureDataHolder centerOfPressureDataHolder;
   private final HumanoidReferenceFrames humanoidReferenceFrames;
   private final CenterOfMassJacobian centerOfMassJacobian;

   private final CommandInputManager commandInputManager;
   private final StatusMessageOutputManager statusOutputManager;

   private final CommandInputManager walkingCommandInputManager;
   private final StatusMessageOutputManager walkingOutputManager;

   private final AvatarBipedalGaitGenerator bipedalGaitGenerator;

   private final YoBoolean isHandRecoveryContactEnabled = new YoBoolean("isHandRecoveryContactEnabled", registry);
   private final YoBoolean isFalling = new YoBoolean("isFalling", registry);
   private final YoBoolean triggerFall = new YoBoolean("triggerFall", registry);
   private final YoBoolean sendHandContactMessage = new YoBoolean("sendHandContactMessage", registry);
   private boolean hasSentHandTrajectory = false;
   private final ReactiveBracingPlanner planner;

   private final FramePoint3D centerOfMassPosition = new FramePoint3D();
   private final FrameVector3D centerOfMassVelocity = new FrameVector3D();

   private final CenterOfMassDataHolder centerOfMassDataHolderForController = new CenterOfMassDataHolder();
   private final YoFramePoint2D currentCapturePoint = new YoFramePoint2D("currentCapturePointMCGG", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint2D desiredCapturePoint = new YoFramePoint2D("desiredCapturePointMCGG", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector2D capturePointError = new YoFrameVector2D("capturePointError", ReferenceFrame.getWorldFrame(), registry);

   private final List<BipedTimedStep> plannedFootSteps = new ArrayList<>();
   private final SideDependentList<HandContactCommand> plannedHandContacts = new SideDependentList<>();
   private final ReducedOrderRobotModel reducedOrderRobotModel;
   private final AtomicReference<CapturabilityBasedStatus> capturabilityBasedStatus = new AtomicReference<>();

   private final YoFramePoint2D[] yoCapturePointWaypoints = new YoFramePoint2D[25];
   private final YoFramePoint3D[] heights;

   public AvatarMultiContactGaitGeneratorThread(double dt,
                                                ROS2Node ros2Node,
                                                DRCRobotModel robotModel,
                                                HumanoidRobotContextDataFactory contextDataFactory,
                                                StatusMessageOutputManager walkingOutputManager,
                                                CommandInputManager walkingCommandInputManager)
   {
      this.dt = dt;
      this.fullRobotModel = robotModel.createFullRobotModel();

      String robotName = robotModel.getSimpleRobotName();
      ROS2Topic<?> inputTopic = MultiContactGaitGenerator.getInputTopic(robotName);
      ROS2Topic<?> outputTopic = MultiContactGaitGenerator.getOutputTopic(robotName);

      planner = robotModel.getReactiveBracingPlanner();
      reducedOrderRobotModel = new ReducedOrderRobotModel(robotModel.getContactPointParameters(), registry);

      this.commandInputManager = new CommandInputManager(MultiContactGaitGenerator.getSupportedCommands());
      this.statusOutputManager = new StatusMessageOutputManager(MultiContactGaitGenerator.getSupportedStatusMessages());
      new ControllerNetworkSubscriber(inputTopic, commandInputManager, outputTopic, statusOutputManager, ros2Node);

      HumanoidRobotContextJointData processedJointData = new HumanoidRobotContextJointData(fullRobotModel.getOneDoFJoints().length);
      ForceSensorDataHolder forceSensorDataHolderForController = new ForceSensorDataHolder(Arrays.asList(fullRobotModel.getForceSensorDefinitions()));
      centerOfPressureDataHolder = new CenterOfPressureDataHolder(fullRobotModel);
      LowLevelOneDoFJointDesiredDataHolder desiredJointDataHolder = new LowLevelOneDoFJointDesiredDataHolder(fullRobotModel.getControllableOneDoFJoints());
      RobotMotionStatusHolder robotMotionStatusHolder = new RobotMotionStatusHolder();

      contextDataFactory.setForceSensorDataHolder(forceSensorDataHolderForController);
      contextDataFactory.setCenterOfMassDataHolder(centerOfMassDataHolderForController);
      contextDataFactory.setCenterOfPressureDataHolder(centerOfPressureDataHolder);
      contextDataFactory.setRobotMotionStatusHolder(robotMotionStatusHolder);
      contextDataFactory.setJointDesiredOutputList(desiredJointDataHolder);
      contextDataFactory.setProcessedJointData(processedJointData);
      contextDataFactory.setSensorDataContext(new SensorDataContext(fullRobotModel));
      this.humanoidRobotContextData = contextDataFactory.createHumanoidRobotContextData();

      this.humanoidReferenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      this.walkingOutputManager = walkingOutputManager;
      this.walkingCommandInputManager = walkingCommandInputManager;

      this.centerOfMassJacobian = new CenterOfMassJacobian(fullRobotModel.getElevator(), ReferenceFrame.getWorldFrame());

      isHandRecoveryContactEnabled.set(true);

      humanoidRobotContextData.setControllerRan(false);
      humanoidRobotContextData.setEstimatorRan(false);

      bipedalGaitGenerator = new AvatarBipedalGaitGenerator(commandInputManager,
                                                            statusOutputManager,
                                                            robotModel,
                                                            fullRobotModel,
                                                            humanoidReferenceFrames,
                                                            walkingCommandInputManager,
                                                            walkingOutputManager,
                                                            registry);

      walkingOutputManager.attachStatusMessageListener(CapturabilityBasedStatus.class, capturabilityBasedStatus::set);

      for (int i = 0; i < yoCapturePointWaypoints.length; i++)
      {
         yoCapturePointWaypoints[i] = new YoFramePoint2D("capturePointWP" + i, ReferenceFrame.getWorldFrame(), registry);
      }

      if (VISUALIZE_HEIGHT_MAP)
      {
         heights = new YoFramePoint3D[8000];
         for (int i = 0; i < heights.length; i++)
         {
            heights[i] = new YoFramePoint3D("height" + i, ReferenceFrame.getWorldFrame(), registry);
         }
      }
      else
      {
         heights = null;
      }
   }

   @Override
   public void run()
   {
      if (!humanoidRobotContextData.getEstimatorRan())
         return;

      // Update capture point preview trajectory
      RecyclingArrayList<FramePoint2D> capturePointPositionWaypoints = centerOfPressureDataHolder.getCapturePointPositionWaypoints();
      for (int i = 0; i < yoCapturePointWaypoints.length; i++)
      {
         yoCapturePointWaypoints[i].setToNaN();
      }
      for (int i = 0; i < capturePointPositionWaypoints.size(); i++)
      {
         yoCapturePointWaypoints[i].set(capturePointPositionWaypoints.get(i));
      }

      // Update robot model
      HumanoidRobotContextTools.updateRobot(fullRobotModel, humanoidRobotContextData.getProcessedJointData());
      humanoidReferenceFrames.updateFrames();
      centerOfMassJacobian.reset();

      // Update bipedal gait generator
      bipedalGaitGenerator.update();

      // Update capturability status
      CapturabilityBasedStatus capturabilityBasedStatus = this.capturabilityBasedStatus.get();
      if (capturabilityBasedStatus != null)
         desiredCapturePoint.set(capturabilityBasedStatus.getDesiredCapturePoint2d());
      else
         desiredCapturePoint.setToNaN();
      updateCentroidalValues();

      currentCapturePoint.setX(centerOfMassPosition.getX() + centerOfMassVelocity.getX() / ReducedOrderRobotModel.OMEGA);
      currentCapturePoint.setY(centerOfMassPosition.getY() + centerOfMassVelocity.getY() / ReducedOrderRobotModel.OMEGA);

      capturePointError.sub(currentCapturePoint, desiredCapturePoint);

      double capturePointErrorThreshold = 0.09;
      isFalling.set(capturePointError.norm() > capturePointErrorThreshold);

      if (commandInputManager.isNewCommandAvailable(PlanarRegionsListCommand.class))
      {
         PlanarRegionsListCommand planarRegionsListCommand = commandInputManager.pollNewestCommand(PlanarRegionsListCommand.class);
//         LogTools.info("Received planar regions command! number of regions: " + planarRegionsListCommand.getNumberOfPlanarRegions());
         planner.setPlanarRegions(planarRegionsListCommand);
      }
      if (commandInputManager.isNewCommandAvailable(TerrainMapCommand.class))
      {
         TerrainMapCommand terrainMapCommand = commandInputManager.pollNewestCommand(TerrainMapCommand.class);
         bipedalGaitGenerator.setTerrainMapCommand(terrainMapCommand);
         if (VISUALIZE_HEIGHT_MAP)
            visualizeHeightMap(terrainMapCommand);
      }

      if (triggerFall.getValue() || (!hasSentHandTrajectory && isFalling.getValue() && isHandRecoveryContactEnabled.getValue()))
      {
         triggerFall.set(false);
         hasSentHandTrajectory = true;
         sendHandContactMessage.set(false);

         reducedOrderRobotModel.initialize(fullRobotModel, humanoidReferenceFrames, centerOfMassVelocity);

         plannedHandContacts.clear();
         planner.plan(reducedOrderRobotModel, plannedFootSteps, plannedHandContacts);

         for (RobotSide robotSide : RobotSide.values)
         {
            HandContactCommand handContactCommand = plannedHandContacts.get(robotSide);
            if (handContactCommand != null)
            {
               LogTools.info("Sending " + robotSide + " hand bracing command!");
               walkingCommandInputManager.submitCommand(handContactCommand);
            }
         }
      }
   }

   private void updateCentroidalValues()
   {
      if (centerOfMassDataHolderForController.hasCenterOfMassPosition())
      {
         centerOfMassPosition.set(centerOfMassDataHolderForController.getCenterOfMassPosition());
      }
      else
      {
         centerOfMassPosition.setFromReferenceFrame(humanoidReferenceFrames.getCenterOfMassFrame());
      }

      if (centerOfMassDataHolderForController.hasCenterOfMassVelocity())
      {
         centerOfMassVelocity.set(centerOfMassDataHolderForController.getCenterOfMassVelocity());
      }
      else
      {
         centerOfMassJacobian.reset();
         centerOfMassVelocity.set(centerOfMassJacobian.getCenterOfMassVelocity());
         centerOfMassVelocity.changeFrame(ReferenceFrame.getWorldFrame());
      }
   }

   @Override
   public double getCurrentDT()
   {
      return dt;
   }

   @Override
   public YoRegistry getYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public FullHumanoidRobotModel getFullRobotModel()
   {
      return fullRobotModel;
   }

   @Override
   public HumanoidRobotContextData getHumanoidRobotContextData()
   {
      return humanoidRobotContextData;
   }

   private boolean hasUpdatedHeightMapVisualization = false;

   private void visualizeHeightMap(TerrainMapCommand terrainMapCommand)
   {
      if (!hasUpdatedHeightMapVisualization)
      {
         Point2DReadOnly gridCenter = terrainMapCommand.getGridCenter();
         double cellSize = terrainMapCommand.getCellSize();
         double gridWidth = terrainMapCommand.getGridWidth();
         int centerIndex = HeightMapTools.computeCenterIndex(gridWidth, cellSize);

         for (int i = 0; i < heights.length; i++)
         {
            int key = 3 * i;
            double x = HeightMapTools.keyToXCoordinate(key, gridCenter.getX(), cellSize, centerIndex);
            double y = HeightMapTools.keyToYCoordinate(key, gridCenter.getY(), cellSize, centerIndex);
            double z = terrainMapCommand.getHeightAt(key);
            heights[i].set(x, y, z);
         }
         hasUpdatedHeightMapVisualization = true;
      }
   }

   @Override
   public YoGraphicGroupDefinition getSCS2YoGraphics()
   {
      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(getClass().getSimpleName());

      for (int i = 0; i < yoCapturePointWaypoints.length; i++)
      {
         group.addChild(YoGraphicDefinitionFactory.newYoGraphicPoint2D("capturePointWP" + i, yoCapturePointWaypoints[i], 0.003, ColorDefinitions.DarkBlue(), DefaultPoint2DGraphic.PLUS));
      }

      if (VISUALIZE_HEIGHT_MAP)
      {
         for (int i = 0; i < heights.length; i++)
         {
            group.addChild(YoGraphicDefinitionFactory.newYoGraphicPoint3D("height" + i, heights[i], 0.003, ColorDefinitions.DarkBlue()));
         }
      }

      return group;
   }
}
