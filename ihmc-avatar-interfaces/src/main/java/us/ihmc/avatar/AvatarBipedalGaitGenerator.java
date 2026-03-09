package us.ihmc.avatar;

import controller_msgs.msg.dds.FootstepStatusMessage;
import controller_msgs.msg.dds.WalkingStatusMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.YoContinuousStepGeneratorParameters;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootstepDataCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootstepDataListCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PauseWalkingCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.TerrainMapCommand;
import us.ihmc.humanoidRobotics.communication.packets.walking.WalkingStatus;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.log.LogTools;
import us.ihmc.perception.gpuMapping.HeightMapTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePose3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

import java.util.concurrent.atomic.AtomicReference;

import static us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.ContinuousStepGenerator.calculateNextFootstepPose2D;

public class AvatarBipedalGaitGenerator
{
   private static final int NUMBER_OF_STEPS_TO_PLAN = 5;
   private static final int NULL_KEY = -1;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final String variableNameSuffix = "MCGG";

   private final FullHumanoidRobotModel fullRobotModel;
   private final HumanoidReferenceFrames humanoidReferenceFrames;

   private final CommandInputManager commandInputManager;
   private final StatusMessageOutputManager statusOutputManager;
   private final CommandInputManager walkingCommandInputManager;
   private final PauseWalkingCommand pauseWalkingCommand = new PauseWalkingCommand();

   private final YoContinuousStepGeneratorParameters parameters = new YoContinuousStepGeneratorParameters(variableNameSuffix, registry);
   private final YoBoolean walkPrev = new YoBoolean("walkMCGSPrev", registry);
   private final YoBoolean walk = new YoBoolean("walkMCGS", registry);
   private final YoDouble desiredWalkingVelocityX = new YoDouble("desiredWalkingVelocityX", registry);
   private final YoDouble desiredWalkingVelocityY = new YoDouble("desiredWalkingVelocityY", registry);
   private final YoDouble desiredWalkingVelocityYaw = new YoDouble("desiredWalkingVelocityYaw", registry);
   private final YoDouble[] traversabilityValues = new YoDouble[NUMBER_OF_STEPS_TO_PLAN];
   private final YoFramePose3D[] stepPoses = new YoFramePose3D[NUMBER_OF_STEPS_TO_PLAN];

   private final YoEnum<RobotSide> currentSupportSide = new YoEnum<>("currentSupportSide", registry, RobotSide.class, false);
   private final YoFramePose3D currentSupportPose = new YoFramePose3D("currentSupportPose", ReferenceFrame.getWorldFrame(), registry);
   private final AtomicReference<WalkingStatusMessage> walkingStatusMessage = new AtomicReference<>();
   private final AtomicReference<FootstepStatusMessage> footstepStatusMessage = new AtomicReference<>();

   private final FootstepDataListCommand footstepDataListCommand = new FootstepDataListCommand();
   private final FootstepDataListCommand previousFootstepDataListCommand = new FootstepDataListCommand();

   private final FramePose2D footstepPose2D = new FramePose2D();
   private final FramePose2D nextFootstepPose2D = new FramePose2D();
   private final FramePose3D nextFootstepPose3D = new FramePose3D();
   private final FramePose3D previousFootstepPose = new FramePose3D();
   private final FramePose3D nextFootstepPose3DViz = new FramePose3D();

   private TerrainMapCommand terrainMapCommand;

   public AvatarBipedalGaitGenerator(CommandInputManager commandInputManager,
                                     StatusMessageOutputManager statusOutputManager,
                                     DRCRobotModel robotModel,
                                     FullHumanoidRobotModel fullRobotModel,
                                     HumanoidReferenceFrames humanoidReferenceFrames,
                                     CommandInputManager walkingCommandInputManager,
                                     StatusMessageOutputManager walkingOutputManager,
                                     YoRegistry parentRegistry)
   {
      this.fullRobotModel = fullRobotModel;
      this.commandInputManager = commandInputManager;
      this.statusOutputManager = statusOutputManager;
      this.humanoidReferenceFrames = humanoidReferenceFrames;
      this.walkingCommandInputManager = walkingCommandInputManager;
      this.parameters.set(robotModel.getWalkingControllerParameters());

      walkingOutputManager.attachStatusMessageListener(WalkingStatusMessage.class, walkingStatusMessage::set);
      walkingOutputManager.attachStatusMessageListener(FootstepStatusMessage.class, footstepStatusMessage::set);

      for (int i = 0; i < NUMBER_OF_STEPS_TO_PLAN; i++)
      {
         traversabilityValues[i] = new YoDouble("traversabilityStep" + i, registry);
         stepPoses[i] = new YoFramePose3D("step" + i, ReferenceFrame.getWorldFrame(), registry);
      }

      currentSupportSide.addListener(v ->
                                     {
                                        if (!footstepDataListCommand.getFootsteps().isEmpty())
                                        {
                                           footstepDataListCommand.removeFootstep(0);
                                        }
                                     });

      parentRegistry.addChild(registry);
   }

   public void update()
   {
      boolean walk = this.walk.getValue();
      boolean walkPrev = this.walkPrev.getValue();
      this.walkPrev.set(walk);

      if (!walk)
      {
         if (walkPrev)
         {
            pauseWalkingCommand.setPauseRequested(true);
            pauseWalkingCommand.setClearRemainingFootstepQueue(true);
            walkingCommandInputManager.submitCommand(pauseWalkingCommand);
         }
         
         footstepDataListCommand.clear();
         return;
      }

      FootstepStatusMessage footstepStatus = footstepStatusMessage.getAndSet(null);
      if (footstepStatus != null && footstepStatus.getFootstepStatus() == FootstepStatusMessage.FOOTSTEP_STATUS_STARTED)
      {
         currentSupportSide.set(RobotSide.fromByte(footstepStatus.getRobotSide()).getOppositeSide());
      }
      if (footstepStatus != null && footstepStatus.getFootstepStatus() == FootstepStatusMessage.FOOTSTEP_STATUS_COMPLETED)
      {
         currentSupportSide.set(RobotSide.fromByte(footstepStatus.getRobotSide()));
      }

      currentSupportPose.setFromReferenceFrame(humanoidReferenceFrames.getSoleFrame(currentSupportSide.getValue()));
      previousFootstepDataListCommand.set(footstepDataListCommand);
      footstepDataListCommand.getFootsteps().clear();
      RobotSide swingSide = currentSupportSide.getValue().getOppositeSide();
      footstepPose2D.setIncludingFrame(currentSupportPose);

      double maxStepLengthForwards = parameters.getMaxStepLengthForwards();
      double maxStepLengthBackwards = parameters.getMaxStepLengthBackwards();
      double maxStepWidth = parameters.getMaxStepWidth();
      double minStepWidth = parameters.getMinStepWidth();
      double defaultStepWidth = parameters.getDefaultStepWidth();
      double turnMaxAngleInward = parameters.getTurnMaxAngleInward();
      double turnMaxAngleOutward = parameters.getTurnMaxAngleOutward();
      double stepTime = getStepTime();

      for (int i = 0; i < NUMBER_OF_STEPS_TO_PLAN; i++)
      {
         calculateNextFootstepPose2D(stepTime,
                                     desiredWalkingVelocityX.getValue(),
                                     desiredWalkingVelocityY.getValue(),
                                     desiredWalkingVelocityYaw.getValue(),
                                     swingSide,
                                     maxStepLengthForwards,
                                     maxStepLengthBackwards,
                                     maxStepWidth,
                                     defaultStepWidth,
                                     minStepWidth,
                                     turnMaxAngleInward,
                                     turnMaxAngleOutward,
                                     footstepPose2D,
                                     nextFootstepPose2D);

         nextFootstepPose3D.set(nextFootstepPose2D);
         FootstepDataCommand footstep = footstepDataListCommand.getFootsteps().add();

         footstep.getPosition().set(nextFootstepPose2D.getPosition());
         footstep.getOrientation().set(nextFootstepPose2D.getOrientation());
         footstep.setRobotSide(swingSide);
         adjustStep(i, footstep);

         footstepPose2D.set(footstep.getPosition().getX(), footstep.getPosition().getY(), footstep.getOrientation().getYaw());
         swingSide = swingSide.getOppositeSide();
      }

      footstepDataListCommand.setDefaultSwingDuration(parameters.getSwingDuration());
      footstepDataListCommand.setDefaultTransferDuration(parameters.getTransferDuration());
      footstepDataListCommand.setFinalTransferDuration(parameters.getTransferDuration());
      footstepDataListCommand.setAreFootstepsAdjustable(parameters.getStepsAreAdjustable());
      footstepDataListCommand.setOffsetFootstepsHeightWithExecutionError(parameters.getAccountForGroundDrift());
      footstepDataListCommand.setOffsetFootstepsWithExecutionError(parameters.getShiftUpcomingStepsWithTouchdown());
      footstepDataListCommand.setExecutionMode(ExecutionMode.OVERRIDE);
      walkingCommandInputManager.submitCommand(footstepDataListCommand);
   }

   public FootstepDataListCommand getFootstepDataListCommand()
   {
      return footstepDataListCommand;
   }

   private double getStepTime()
   {
      return parameters.getSwingDuration() + parameters.getTransferDuration();
   }

   public void setTerrainMapCommand(TerrainMapCommand terrainMapCommand)
   {
      this.terrainMapCommand = terrainMapCommand;
      LogTools.info("Received terrain map command");
   }

   private void adjustStep(int stepIndex, FootstepDataCommand footstep)
   {
      boolean success = adjustToTerrain(stepIndex, footstep);
      if (!success)
         snapToStanceFoot(stepIndex, footstep);
      stepPoses[stepIndex].set(footstep.getPosition(), footstep.getOrientation());
   }

   private boolean adjustToTerrain(int stepIndex, FootstepDataCommand footstep)
   {
      if (terrainMapCommand == null)
         return false;

      int key = findOptimalFoothold(stepIndex, footstep);
      if (key == NULL_KEY)
         return false;

      // Footstep position
      footstep.getPosition().setX(keyToXCoordinate(key));
      footstep.getPosition().setY(keyToYCoordinate(key));
      footstep.getPosition().setZ(terrainMapCommand.getHeightAt(key));

      // Footstep orientation
      EuclidGeometryTools.orientation3DFromFirstToSecondVector3D(Axis3D.Z, terrainMapCommand.getNormalAt(key), footstep.getOrientation());

      // Set traversability
      double traversability = terrainMapCommand.getTraversabilityAt(key);
      traversabilityValues[stepIndex].set(traversability);

      return true;
   }

   private final Point2D tempPoint = new Point2D();

   private int findOptimalFoothold(int stepIndex, FootstepDataCommand footstep)
   {
      double nominalX = footstep.getPosition().getX();
      double nominalY = footstep.getPosition().getY();
      double nominalYaw = footstep.getOrientation().getYaw();

      int sampleWindowX = 4;
      int sampleWindowY = 3;
      double sampleDiscretization = 0.06;

      int optimalKey = NULL_KEY;
      double optimalScore = Double.NEGATIVE_INFINITY;

      boolean hasPreviousStep = previousFootstepDataListCommand.getFootsteps().size() >= stepIndex + 1;
      double previousStepWeight = -8.0 * Math.pow(2.5, -stepIndex);
      
      if (hasPreviousStep && previousFootstepDataListCommand.getFootsteps().get(stepIndex).getRobotSide() != footstep.getRobotSide())
      {
         throw new RuntimeException("Invalid step sides");
      }

      for (int di_x = -sampleWindowX; di_x <= sampleWindowX; di_x++)
      {
         for (int di_y = -sampleWindowY; di_y <= sampleWindowY; di_y++)
         {
            double dxLocal = di_x * sampleDiscretization;
            double dyLocal = di_y * sampleDiscretization;

            double dxWorld = dxLocal * Math.cos(nominalYaw) - dyLocal * Math.sin(nominalYaw);
            double dyWorld = dxLocal * Math.sin(nominalYaw) + dyLocal * Math.cos(nominalYaw);

            double xQuery = nominalX + dxWorld;
            double yQuery = nominalY + dyWorld;

            int key = coordinateToKey(xQuery, yQuery);
            if (key == NULL_KEY)
               continue;

            double traversability = terrainMapCommand.getTraversabilityAt(key);
            double offset = EuclidCoreTools.norm(dxWorld, dyWorld);
            double offsetDotProduct = Math.abs(offset) < 1e-3 ? 0.0 : (desiredWalkingVelocityX.getValue() * dxWorld + desiredWalkingVelocityY.getValue() * dyWorld) / offset;

            double traversabilityWeight = 3.0;
            double offsetWeight = -1.0;
            double dotProductWeight = 0.5;

            double score = traversabilityWeight * traversability + offsetWeight * offset + dotProductWeight * offsetDotProduct;
            if (hasPreviousStep)
            {
               tempPoint.set(xQuery, yQuery);
               score += previousStepWeight * previousFootstepDataListCommand.getFootsteps().get(stepIndex).getPosition().distanceXY(tempPoint);
            }

            if (score > optimalScore)
            {
               optimalScore = score;
               optimalKey = key;
            }
         }
      }

      return optimalKey;
   }

   private int coordinateToKey(double x, double y)
   {
      Point2DReadOnly gridCenter = terrainMapCommand.getGridCenter();
      double cellSize = terrainMapCommand.getCellSize();
      double gridWidth = terrainMapCommand.getGridWidth();
      int centerIndex = HeightMapTools.computeCenterIndex(gridWidth, cellSize);
      int key = HeightMapTools.coordinateToKey(x, y, gridCenter.getX(), gridCenter.getY(), cellSize, centerIndex);
      if (key < 0 || key >= terrainMapCommand.getMapSize())
         return NULL_KEY;
      else
         return key;
   }

   private double keyToXCoordinate(int key)
   {
      Point2DReadOnly gridCenter = terrainMapCommand.getGridCenter();
      double cellSize = terrainMapCommand.getCellSize();
      double gridWidth = terrainMapCommand.getGridWidth();
      int centerIndex = HeightMapTools.computeCenterIndex(gridWidth, cellSize);
      return HeightMapTools.keyToXCoordinate(key, gridCenter.getX(), cellSize, centerIndex);
   }

   private double keyToYCoordinate(int key)
   {
      Point2DReadOnly gridCenter = terrainMapCommand.getGridCenter();
      double cellSize = terrainMapCommand.getCellSize();
      double gridWidth = terrainMapCommand.getGridWidth();
      int centerIndex = HeightMapTools.computeCenterIndex(gridWidth, cellSize);
      return HeightMapTools.keyToYCoordinate(key, gridCenter.getY(), cellSize, centerIndex);
   }

   private void snapToStanceFoot(int stepIndex, FootstepDataCommand footstep)
   {
      FramePoint3D position = footstep.getPosition();
      position.setZ(currentSupportPose.getZ());
      traversabilityValues[stepIndex].setToNaN();
   }

   private boolean isWalking()
   {
      WalkingStatusMessage walkingStatusMessage = this.walkingStatusMessage.get();
      if (walkingStatusMessage == null)
         return false;
      WalkingStatus walkingStatus = WalkingStatus.fromByte(walkingStatusMessage.getWalkingStatus());
      return walkingStatus == WalkingStatus.STARTED || walkingStatus == WalkingStatus.RESUMED;
   }

   public YoRegistry getYoVariableRegistry()
   {
      return registry;
   }
}