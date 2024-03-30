package us.ihmc.rdx.perception;

import behavior_msgs.msg.dds.ContinuousWalkingCommandMessage;
import com.badlogic.gdx.controllers.Controller;
import com.badlogic.gdx.controllers.Controllers;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.WalkingControllerFailureStatusMessage;
import ihmc_common_msgs.msg.dds.PoseListMessage;
import imgui.ImGui;
import imgui.type.ImBoolean;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.activeMapping.ContinuousWalkingParameters;
import us.ihmc.behaviors.activeMapping.StancePoseCalculator;
import us.ihmc.commonWalkingControlModules.configurations.SwingTrajectoryParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.commonWalkingControlModules.trajectories.PositionOptimizedTrajectoryGenerator;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.footstepPlanning.FootstepDataMessageConverter;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.MonteCarloFootstepPlannerParameters;
import us.ihmc.footstepPlanning.communication.ContinuousWalkingAPI;
import us.ihmc.footstepPlanning.swing.SwingPlannerParametersBasics;
import us.ihmc.footstepPlanning.tools.SwingPlannerTools;
import us.ihmc.log.LogTools;
import us.ihmc.perception.HumanoidActivePerceptionModule;
import us.ihmc.perception.heightMap.TerrainMapData;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.robotics.math.trajectories.interfaces.PolynomialReadOnly;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SegmentDependentList;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

public class RDXContinuousWalkingPanel extends RDXPanel implements RenderableProvider
{
   private final ContinuousWalkingCommandMessage commandMessage = new ContinuousWalkingCommandMessage();
   private final AtomicReference<FootstepDataListMessage> footstepDataListMessage = new AtomicReference<>(null);
   private final AtomicReference<FootstepDataListMessage> monteCarloPlanDataListMessage = new AtomicReference<>(null);
   private final SideDependentList<FramePose3D> startStancePose = new SideDependentList<>(new FramePose3D(), new FramePose3D());
   private final SideDependentList<FramePose3D> goalStancePose = new SideDependentList<>(new FramePose3D(), new FramePose3D());

   private final ImBoolean renderEnabled = new ImBoolean(true);
   private final ImBoolean localRenderMode = new ImBoolean(false);
   private final ImBoolean useMonteCarloReference = new ImBoolean(false);
   private final ImBoolean useHybridPlanner = new ImBoolean(false);
   private final ImBoolean useAStarFootstepPlanner = new ImBoolean(false);
   private final ImBoolean useMonteCarloFootstepPlanner = new ImBoolean(true);

   private RDXStancePoseSelectionPanel stancePoseSelectionPanel;
   private final StancePoseCalculator stancePoseCalculator;
   private final IHMCROS2Publisher<ContinuousWalkingCommandMessage> commandPublisher;

   private static final int numberOfKnotPoints = 12;
   private static final int maxIterationsOptimization = 100;
   private final PositionOptimizedTrajectoryGenerator positionTrajectoryGenerator = new PositionOptimizedTrajectoryGenerator(numberOfKnotPoints,
                                                                                                                             maxIterationsOptimization);

   private final ROS2Helper ros2Helper;
   private RDXTerrainPlanningDebugger terrainPlanningDebugger;
   private HumanoidActivePerceptionModule activePerceptionModule;
   private SwingPlannerParametersBasics swingPlannerParameters;
   private SwingTrajectoryParameters swingTrajectoryParameters;
   private ContinuousWalkingParameters continuousWalkingParameters;
   private Controller currentController;

   private boolean currentControllerConnected;

   public RDXContinuousWalkingPanel(ROS2Helper ros2Helper,
                                    HumanoidActivePerceptionModule activePerceptionModule,
                                    ROS2SyncedRobotModel syncedRobot,
                                    ContinuousWalkingParameters continuousWalkingParameters,
                                    SwingPlannerParametersBasics swingPlannerParameters,
                                    SwingTrajectoryParameters swingTrajectoryParameters,
                                    MonteCarloFootstepPlannerParameters monteCarloPlannerParameters)
   {
      super("Continuous Planning");
      setRenderMethod(this::renderImGuiWidgets);

      this.ros2Helper = ros2Helper;
      this.activePerceptionModule = activePerceptionModule;
      this.swingPlannerParameters = swingPlannerParameters;
      this.swingTrajectoryParameters = swingTrajectoryParameters;
      this.continuousWalkingParameters = continuousWalkingParameters;

      ros2Helper.subscribeViaCallback(ContinuousWalkingAPI.START_AND_GOAL_FOOTSTEPS, this::onStartAndGoalPosesReceived);
      ros2Helper.subscribeViaCallback(ContinuousWalkingAPI.PLANNED_FOOTSTEPS, this::onPlannedFootstepsReceived);
      ros2Helper.subscribeViaCallback(ContinuousWalkingAPI.MONTE_CARLO_FOOTSTEP_PLAN, this::onMonteCarloPlanReceived);
      ros2Helper.subscribeViaCallback(ControllerAPIDefinition.getTopic(FootstepDataListMessage.class, syncedRobot.getRobotModel().getSimpleRobotName()),
                                      this::onControllerFootstepsReceived);

      commandPublisher = ROS2Tools.createPublisher(ros2Helper.getROS2NodeInterface(), ContinuousWalkingAPI.CONTINUOUS_WALKING_COMMAND);

      SegmentDependentList<RobotSide, ArrayList<Point2D>> groundContactPoints = syncedRobot.getRobotModel().getContactPointParameters().getControllerFootGroundContactPoints();
      SideDependentList<ConvexPolygon2D> defaultContactPoints = new SideDependentList<>();
      for (RobotSide robotSide : RobotSide.values)
      {
         ConvexPolygon2D defaultFoothold = new ConvexPolygon2D();
         groundContactPoints.get(robotSide).forEach(defaultFoothold::addVertex);
         defaultFoothold.update();
         defaultContactPoints.put(robotSide, defaultFoothold);
      }
      stancePoseCalculator = new StancePoseCalculator(0.5f, 0.5f, 0.1f, defaultContactPoints);

      stancePoseSelectionPanel = new RDXStancePoseSelectionPanel(stancePoseCalculator);
      terrainPlanningDebugger = new RDXTerrainPlanningDebugger(ros2Helper,
                                                               monteCarloPlannerParameters,
                                                               syncedRobot.getRobotModel().getContactPointParameters().getControllerFootGroundContactPoints());


      ros2Helper.subscribeViaCallback(ControllerAPIDefinition.getTopic(WalkingControllerFailureStatusMessage.class,
                                                                       syncedRobot.getRobotModel().getSimpleRobotName()), message ->
                                      {
                                         terrainPlanningDebugger.reset();
                                      });
   }

   public RDXContinuousWalkingPanel(ROS2Helper ros2Helper, DRCRobotModel robotModel, MonteCarloFootstepPlannerParameters monteCarloPlannerParameters)
   {
      super("Continuous Planning");
      setRenderMethod(this::renderImGuiWidgets);

      SegmentDependentList<RobotSide, ArrayList<Point2D>> groundContactPoints = robotModel.getContactPointParameters().getControllerFootGroundContactPoints();
      SideDependentList<ConvexPolygon2D> defaultContactPoints = new SideDependentList<>();
      for (RobotSide robotSide : RobotSide.values)
      {
         ConvexPolygon2D defaultFoothold = new ConvexPolygon2D();
         groundContactPoints.get(robotSide).forEach(defaultFoothold::addVertex);
         defaultFoothold.update();
         defaultContactPoints.put(robotSide, defaultFoothold);
      }

      stancePoseCalculator = new StancePoseCalculator(0.5f, 0.5f, 0.1f, defaultContactPoints);

      this.ros2Helper = ros2Helper;
      this.swingPlannerParameters = robotModel.getSwingPlannerParameters();
      this.swingTrajectoryParameters = robotModel.getWalkingControllerParameters().getSwingTrajectoryParameters();
      this.commandPublisher = ROS2Tools.createPublisher(ros2Helper.getROS2NodeInterface(), ContinuousWalkingAPI.CONTINUOUS_WALKING_COMMAND);
      this.stancePoseSelectionPanel = new RDXStancePoseSelectionPanel(stancePoseCalculator);
      this.terrainPlanningDebugger = new RDXTerrainPlanningDebugger(ros2Helper,
                                                                    monteCarloPlannerParameters,
                                                                    robotModel.getContactPointParameters().getControllerFootGroundContactPoints());

      ros2Helper.subscribeViaCallback(ContinuousWalkingAPI.START_AND_GOAL_FOOTSTEPS, this::onStartAndGoalPosesReceived);
      ros2Helper.subscribeViaCallback(ContinuousWalkingAPI.PLANNED_FOOTSTEPS, this::onPlannedFootstepsReceived);
      ros2Helper.subscribeViaCallback(ContinuousWalkingAPI.MONTE_CARLO_FOOTSTEP_PLAN, this::onMonteCarloPlanReceived);
      ros2Helper.subscribeViaCallback(ControllerAPIDefinition.getTopic(FootstepDataListMessage.class, robotModel.getSimpleRobotName()),
                                      this::onControllerFootstepsReceived);
      ros2Helper.subscribeViaCallback(ControllerAPIDefinition.getTopic(WalkingControllerFailureStatusMessage.class, robotModel.getSimpleRobotName()), message ->
      {
         terrainPlanningDebugger.reset();
      });
   }

   public void update(TerrainMapData terrainMapData, HeightMapData heightMapData)
   {
      if (!renderEnabled.get())
         return;

      updateFootstepGraphics();

      terrainPlanningDebugger.generateStartAndGoalFootstepGraphics(startStancePose, goalStancePose);
      terrainPlanningDebugger.update(terrainMapData);
//      stancePoseSelectionPanel.update(goalStancePose, terrainMapData, heightMapData);
   }

   public void updateFootstepGraphics()
   {
      if (footstepDataListMessage.get() != null)
      {
         terrainPlanningDebugger.generateFootstepPlanGraphic(footstepDataListMessage.get());
         if (activePerceptionModule != null && localRenderMode.get())
         {
            terrainPlanningDebugger.generateSwingGraphics(activePerceptionModule.getContinuousPlannerSchedulingTask()
                                                                                .getContinuousPlanner()
                                                                                .getLatestFootstepPlan(),
                                                          activePerceptionModule.getContinuousPlannerSchedulingTask()
                                                                                .getContinuousPlanner()
                                                                                .getLatestSwingTrajectories());
         }
         else
         {
            FootstepPlan plan = FootstepDataMessageConverter.convertToFootstepPlan(footstepDataListMessage.get());
            List<EnumMap<Axis3D, List<PolynomialReadOnly>>> swingTrajectories = SwingPlannerTools.computeTrajectories(swingTrajectoryParameters,
                                                                                                                      positionTrajectoryGenerator,
                                                                                                                      startStancePose,
                                                                                                                      plan);
            terrainPlanningDebugger.generateSwingGraphics(plan, swingTrajectories);
         }
         footstepDataListMessage.set(null);
      }

      if (monteCarloPlanDataListMessage.get() != null)
      {
         terrainPlanningDebugger.generateMonteCarloPlanGraphic(monteCarloPlanDataListMessage.get());
         monteCarloPlanDataListMessage.set(null);
      }
   }

   public void renderImGuiWidgets()
   {
      ImGui.checkbox("Render", renderEnabled);
      ImGui.checkbox("Local Render Mode", localRenderMode);
      ImGui.checkbox("Use A* Footstep Planner", useAStarFootstepPlanner);
      ImGui.checkbox("Use Monte-Carlo Footstep Planner", useMonteCarloFootstepPlanner);
      ImGui.checkbox("Use Monte-Carlo Reference", useMonteCarloReference);
      ImGui.separator();
      terrainPlanningDebugger.renderImGuiWidgets();
      publishInputCommandMessage();
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (renderEnabled.get())
      {
         stancePoseSelectionPanel.getRenderables(renderables, pool);
         terrainPlanningDebugger.getRenderables(renderables, pool);
      }
   }

   public void onPlannedFootstepsReceived(FootstepDataListMessage message)
   {
      LogTools.debug("Received footstep plan: {}", message.getFootstepDataList().size());
      this.footstepDataListMessage.set(message);
   }

   public void onMonteCarloPlanReceived(FootstepDataListMessage message)
   {
      LogTools.debug("Received Monte-Carlo Plan: {}", message.getFootstepDataList().size());
      this.monteCarloPlanDataListMessage.set(message);
   }

   public void onControllerFootstepsReceived(FootstepDataListMessage message)
   {
      //LogTools.warn("Received footstep plan: {}", message.getFootstepDataList().size());
      //this.footstepDataListMessage.set(message);
   }

   public void onStartAndGoalPosesReceived(PoseListMessage poseListMessage)
   {
      List<Pose3D> poses = MessageTools.unpackPoseListMessage(poseListMessage);
      startStancePose.get(RobotSide.LEFT).set(poses.get(0));
      startStancePose.get(RobotSide.RIGHT).set(poses.get(1));
      goalStancePose.get(RobotSide.LEFT).set(poses.get(2));
      goalStancePose.get(RobotSide.RIGHT).set(poses.get(3));
   }

   private void publishInputCommandMessage()
   {
      currentController = Controllers.getCurrent();
      currentControllerConnected = currentController != null;

      boolean walkingEnabled = ImGui.getIO().getKeyCtrl();
      double forwardJoystickValue = 0.0;
      double lateralJoystickValue = 0.0;
      double turningJoystickValue = 0.0;

      if (currentControllerConnected)
      {
         walkingEnabled |= currentController.getButton(currentController.getMapping().buttonR1);
         forwardJoystickValue = -currentController.getAxis(currentController.getMapping().axisLeftY);
         lateralJoystickValue = -currentController.getAxis(currentController.getMapping().axisLeftX);
         turningJoystickValue = -currentController.getAxis(currentController.getMapping().axisRightX);
      }

      // Only allow Continuous Walking if the CTRL key is held and the check box is checked
      if (continuousWalkingParameters != null && continuousWalkingParameters.getEnableContinuousWalking())
      {
         commandMessage.setEnableContinuousWalking(walkingEnabled);
         commandMessage.setPublishToController(ImGui.getIO().getKeyAlt());
         commandMessage.setForwardValue(forwardJoystickValue);
         commandMessage.setLateralValue(lateralJoystickValue);
         commandMessage.setTurningValue(turningJoystickValue);
         commandMessage.setUseMonteCarloFootstepPlanner(useMonteCarloFootstepPlanner.get());
         commandMessage.setUseAstarFootstepPlanner(useAStarFootstepPlanner.get());
         commandMessage.setUseMonteCarloPlanAsReference(useMonteCarloReference.get());
         commandMessage.setUsePreviousPlanAsReference(!useMonteCarloReference.get());
         commandMessage.setUseHybridPlanner(useHybridPlanner.get());

         commandPublisher.publish(commandMessage);
      }
   }

   public void destroy()
   {
      stancePoseSelectionPanel.destroy();
      terrainPlanningDebugger.destroy();
   }

   public RDXStancePoseSelectionPanel getStancePoseSelectionPanel()
   {
      return stancePoseSelectionPanel;
   }
}
