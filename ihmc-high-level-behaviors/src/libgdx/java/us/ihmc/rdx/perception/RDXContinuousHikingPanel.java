package us.ihmc.rdx.perception;

import behavior_msgs.msg.dds.ContinuousHikingCommandMessage;
import com.badlogic.gdx.controllers.Controller;
import com.badlogic.gdx.controllers.Controllers;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import com.studiohartman.jamepad.ControllerButton;
import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepStatusMessage;
import controller_msgs.msg.dds.PlanOffsetStatus;
import controller_msgs.msg.dds.WalkingControllerFailureStatusMessage;
import ihmc_common_msgs.msg.dds.PoseListMessage;
import imgui.ImGui;
import imgui.type.ImBoolean;
import std_msgs.msg.dds.Empty;
import std_msgs.msg.dds.Float32;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commons.thread.Throttler;
import us.ihmc.behaviors.activeMapping.ContinuousHikingParameters;
import us.ihmc.humanoidRobotics.communication.ControllerFootstepQueueMonitor;
import us.ihmc.behaviors.activeMapping.StancePoseCalculator;
import us.ihmc.commonWalkingControlModules.configurations.SwingTrajectoryParameters;
import us.ihmc.commonWalkingControlModules.trajectories.PositionOptimizedTrajectoryGenerator;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.property.StoredPropertySetROS2TopicPair;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.footstepPlanning.FootstepDataMessageConverter;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.MonteCarloFootstepPlannerParameters;
import us.ihmc.footstepPlanning.communication.ContinuousHikingAPI;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.swing.SwingPlannerParametersBasics;
import us.ihmc.footstepPlanning.tools.SwingPlannerTools;
import us.ihmc.log.LogTools;
import us.ihmc.perception.comms.PerceptionComms;
import us.ihmc.perception.filters.DepthImageFilteringParameters;
import us.ihmc.perception.gpuMapping.TerrainMapData;
import us.ihmc.perception.gpuMapping.TerrainMapParameters;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.ui.ImGuiRemoteROS2StoredPropertySetGroup;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.RDXStoredPropertySetTuner;
import us.ihmc.robotics.trajectories.interfaces.PolynomialReadOnly;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SegmentDependentList;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Publisher;
import us.ihmc.perception.gpuMapping.HeightMapParameters;
import us.ihmc.tools.property.StoredPropertySetBasics;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;

import static us.ihmc.communication.HumanoidControllerAPI.getTopic;

public class RDXContinuousHikingPanel extends RDXPanel implements RenderableProvider
{
   /**
    * This defines the update thread rate for information being sent over ROS2. The reason this is a very low frequency is because
    * we only have a limited amount of networking bandwidth when using WIFI.
    */
   private static final double THROTTLER_THREAD_HERTZ = 2.0;
   private static final boolean DEBUG = true;

   private final Throttler ros2Throttler;

   private final RDXStancePoseSelectionPanel stancePoseSelectionPanel;
   private final RDXTerrainPlanningDebugger terrainPlanningDebugger;

   private static final int numberOfKnotPoints = 12;
   private static final int maxIterationsOptimization = 100;
   private final PositionOptimizedTrajectoryGenerator positionTrajectoryGenerator = new PositionOptimizedTrajectoryGenerator(numberOfKnotPoints,
                                                                                                                             maxIterationsOptimization);
   private final ImGuiRemoteROS2StoredPropertySetGroup hostStoredPropertySets;
   private final RDXStoredPropertySetTuner continuousHikingParametersPanel = new RDXStoredPropertySetTuner("Continuous Hiking Parameters (CH)");
   private final ContinuousHikingParameters continuousHikingParameters;
   private final SwingTrajectoryParameters swingTrajectoryParameters;

   private final ImBoolean squareUpToGoal = new ImBoolean(true);
   private final ImBoolean enableContinuousHiking = new ImBoolean(false);
   private final ImBoolean useAStarFootstepPlanner = new ImBoolean(true);
   private final ImBoolean useMonteCarloReference = new ImBoolean(false);
   private final ImBoolean useMonteCarloFootstepPlanner = new ImBoolean(false);

   private final ControllerFootstepQueueMonitor controllerFootstepQueueMonitorUI;
   private final ContinuousHikingCommandMessage commandMessage = new ContinuousHikingCommandMessage();
   private final ROS2Publisher<ContinuousHikingCommandMessage> commandPublisher;
   private final ROS2Publisher<Empty> squareUpPublisher;
   private final ROS2Publisher<Empty> clearGoalFootstepsPublisher;
   private final ROS2Publisher<Empty> resetStateMachinePublisher;
   private final ROS2Publisher<Float32> turn90DegreesPublisher;
   private final ROS2Publisher<PlanOffsetStatus> planOffsetStatusPublisher;
   private final ROS2Publisher<FootstepStatusMessage> footstepStatusMessagePublisher;
   private final ROS2Publisher<WalkingControllerFailureStatusMessage> walkingControllerFailureStatusPublisher;

   private SideDependentList<FramePose3D> startStancePose = new SideDependentList<>(new FramePose3D(), new FramePose3D());
   private List<EnumMap<Axis3D, List<PolynomialReadOnly>>> swingTrajectories;
   private FootstepPlan latestFootstepPlan;

   // When running in simulation only, these fields allow running the Continuous Hiking Process locally
   private double simulatedDriftInMeters = -0.1;
   private boolean previousRightBumper;
   private boolean previousLeftBumper;
   private boolean previousYButton;
   private boolean previousStartButton;

   public RDXContinuousHikingPanel(RDXBaseUI baseUI, ROS2Node ros2Node, DRCRobotModel robotModel)
   {
      super("Continuous Hiking");
      setRenderMethod(this::renderImGuiWidgets);

      footstepStatusMessagePublisher = ros2Node.createPublisher(getTopic(FootstepStatusMessage.class, robotModel.getSimpleRobotName()));
      walkingControllerFailureStatusPublisher = ros2Node.createPublisher(getTopic(WalkingControllerFailureStatusMessage.class,
                                                                                  robotModel.getSimpleRobotName()));
      planOffsetStatusPublisher = ros2Node.createPublisher(getTopic(PlanOffsetStatus.class, robotModel.getSimpleRobotName()));
      clearGoalFootstepsPublisher = ros2Node.createPublisher(ContinuousHikingAPI.CLEAR_GOAL_FOOTSTEPS);
      resetStateMachinePublisher = ros2Node.createPublisher(ContinuousHikingAPI.RESET_STATE_MACHINE);

      MonteCarloFootstepPlannerParameters monteCarloPlannerParameters = new MonteCarloFootstepPlannerParameters();
      terrainPlanningDebugger = new RDXTerrainPlanningDebugger(ros2Node,
                                                               monteCarloPlannerParameters,
                                                               robotModel.getContactPointParameters().getControllerFootGroundContactPoints());

      ros2Node.createSubscription(getTopic(WalkingControllerFailureStatusMessage.class, robotModel.getSimpleRobotName()),
                                  (s) -> terrainPlanningDebugger.reset());

      ros2Node.createSubscription2(ContinuousHikingAPI.START_AND_GOAL_FOOTSTEPS, this::onStartAndGoalPosesReceived);
      ros2Node.createSubscription2(ContinuousHikingAPI.PLANNED_FOOTSTEPS, this::onPlannedFootstepsReceived);
      ros2Node.createSubscription2(ContinuousHikingAPI.MONTE_CARLO_FOOTSTEP_PLAN, this::onMonteCarloPlanReceived);

      commandPublisher = ros2Node.createPublisher(ContinuousHikingAPI.CONTINUOUS_HIKING_COMMAND);
      squareUpPublisher = ros2Node.createPublisher(ContinuousHikingAPI.SQUARE_UP_STEP);

      SegmentDependentList<RobotSide, ArrayList<Point2D>> groundContactPoints = robotModel.getContactPointParameters().getControllerFootGroundContactPoints();
      SideDependentList<ConvexPolygon2D> defaultContactPoints = new SideDependentList<>();
      for (RobotSide robotSide : RobotSide.values)
      {
         ConvexPolygon2D defaultFoothold = new ConvexPolygon2D();
         groundContactPoints.get(robotSide).forEach(defaultFoothold::addVertex);
         defaultFoothold.update();
         defaultContactPoints.put(robotSide, defaultFoothold);
      }

      turn90DegreesPublisher = ros2Node.createPublisher(ContinuousHikingAPI.ROTATE_90_DEGREES);

      StancePoseCalculator stancePoseCalculator = new StancePoseCalculator(defaultContactPoints);
      stancePoseSelectionPanel = new RDXStancePoseSelectionPanel(baseUI, ros2Node, stancePoseCalculator);
      addChild(stancePoseSelectionPanel);

      DefaultFootstepPlannerParametersBasics footstepPlannerParameters = robotModel.getFootstepPlannerParameters("ForContinuousWalking");
      SwingPlannerParametersBasics swingPlannerParameters = robotModel.getSwingPlannerParameters("ForContinuousWalking");
      this.swingTrajectoryParameters = robotModel.getWalkingControllerParameters().getSwingTrajectoryParameters();
      DepthImageFilteringParameters depthImageFilteringParameters = new DepthImageFilteringParameters();

      hostStoredPropertySets = new ImGuiRemoteROS2StoredPropertySetGroup(ros2Node);
      continuousHikingParameters = new ContinuousHikingParameters();
      HeightMapParameters heightMapParameters = new HeightMapParameters();
      TerrainMapParameters terrainMapParameters = new TerrainMapParameters();
      createParametersPanel(continuousHikingParameters,
                            continuousHikingParametersPanel,
                            hostStoredPropertySets,
                            ContinuousHikingAPI.CONTINUOUS_HIKING_PARAMETERS);
      RDXStoredPropertySetTuner monteCarloPlannerParametersPanel = new RDXStoredPropertySetTuner("Monte Carlo Footstep Planner Parameters (CH)");
      createParametersPanel(monteCarloPlannerParameters,
                            monteCarloPlannerParametersPanel,
                            hostStoredPropertySets,
                            ContinuousHikingAPI.MONTE_CARLO_PLANNER_PARAMETERS);
      RDXStoredPropertySetTuner footstepPlanningParametersPanel = new RDXStoredPropertySetTuner("Footstep Planner Parameters (CH)");
      createParametersPanel(footstepPlannerParameters,
                            footstepPlanningParametersPanel,
                            hostStoredPropertySets,
                            ContinuousHikingAPI.FOOTSTEP_PLANNING_PARAMETERS);
      RDXStoredPropertySetTuner swingPlannerParametersPanel = new RDXStoredPropertySetTuner("Swing Planner Parameters (CH)");
      createParametersPanel(swingPlannerParameters, swingPlannerParametersPanel, hostStoredPropertySets, ContinuousHikingAPI.SWING_PLANNING_PARAMETERS);
      RDXStoredPropertySetTuner heightMapParametersPanel = new RDXStoredPropertySetTuner("Height Map Parameters (CH)");
      createParametersPanel(heightMapParameters, heightMapParametersPanel, hostStoredPropertySets, PerceptionComms.HEIGHT_MAP_PARAMETERS);
      RDXStoredPropertySetTuner terrainMapParametersPanel = new RDXStoredPropertySetTuner("Terrain Map Parameters (CH)");
      createParametersPanel(terrainMapParameters, terrainMapParametersPanel, hostStoredPropertySets, PerceptionComms.TERRAIN_MAP_PARAMETERS);

      RDXStoredPropertySetTuner depthImageFilteringParametersPanel = new RDXStoredPropertySetTuner("Depth Image Filtering Parameters");
      createParametersPanel(depthImageFilteringParameters,
                            depthImageFilteringParametersPanel,
                            hostStoredPropertySets,
                            ContinuousHikingAPI.DEPTH_IMAGE_FILTERING_PARAMETERS);

      controllerFootstepQueueMonitorUI = new ControllerFootstepQueueMonitor(ros2Node, robotModel.getSimpleRobotName());

      ros2Throttler = new Throttler().setFrequency(THROTTLER_THREAD_HERTZ);
   }

   /**
    * Sets up an ImGui Panel with the given StoredPropertySet
    */
   private void createParametersPanel(StoredPropertySetBasics storedPropertySetParameters,
                                      RDXStoredPropertySetTuner storedPropertySetPanel,
                                      ImGuiRemoteROS2StoredPropertySetGroup remotePropertySets,
                                      StoredPropertySetROS2TopicPair topicName)
   {
      storedPropertySetPanel.create(storedPropertySetParameters, false);
      remotePropertySets.registerRemotePropertySet(storedPropertySetParameters, topicName);
      this.addChild(storedPropertySetPanel);
   }

   public void update(TerrainMapData terrainMapData)
   {
      if (latestFootstepPlan != null)
      {
         terrainPlanningDebugger.generateSwingGraphics(latestFootstepPlan, swingTrajectories);
      }
      latestFootstepPlan = null;
      terrainPlanningDebugger.update(terrainMapData);
      stancePoseSelectionPanel.update(terrainMapData);

      if (ros2Throttler.run())
      {
         updateRos2StoredPropertySets();
      }
   }

   /**
    * This method handles updating the stored property sets used in Continuous Hiking.
    * These are all the parameters that are getting synced back and forth between the remote process and the local process.
    * There are three situations that can occur when trying to use Continuous Hiking.
    */
   private void updateRos2StoredPropertySets()
   {
      hostStoredPropertySets.setPropertyChanged();
   }

   public void renderImGuiWidgets()
   {
      if (ImGui.button("Stop Walking"))
      {
         enableContinuousHiking.set(false);
         publishStopContinuousHiking(squareUpToGoal.get());
      }
      ImGuiTools.previousWidgetTooltip("ALT");
      ImGui.sameLine();
      if (ImGui.button("Abort Walking"))
      {
         publishStopContinuousHiking(false);
      }
      ImGuiTools.previousWidgetTooltip("ESC");
      ImGui.sameLine();
      if (ImGui.button("Reset State Machine"))
      {
         enableContinuousHiking.set(false);
         resetStateMachinePublisher.publish(new Empty());
         publishStopContinuousHiking(false);
      }

      if (ImGui.checkbox("Enable Continuous Hiking", enableContinuousHiking))
      {
         if (!enableContinuousHiking.get())
         {
            publishStopContinuousHiking(false);
         }
      }
      ImGui.checkbox("Square Up To Goal", squareUpToGoal);
      ImGui.sameLine();
      if (ImGui.button("Square Up"))
      {
         if (controllerFootstepQueueMonitorUI.getControllerFootstepQueue().isEmpty())
            squareUpPublisher.publish(new Empty());
      }
      ImGui.separator();

      if (ImGui.button("Walk Forward"))
      {
         publishStartContinuousHiking(true, false);
      }
      ImGuiTools.previousWidgetTooltip("CTRL + SHIFT");
      ImGui.sameLine();

      if (ImGui.button("Walk Backwards"))
      {
         publishStartContinuousHiking(false, true);
      }

      if (ImGui.button("Turn Left 90°") && enableContinuousHiking.get())
      {
         turnRobot((float) (Math.PI / 2.0));
      }
      ImGui.sameLine();
      if (ImGui.button("Turn Right 90°") && enableContinuousHiking.get())
      {
         turnRobot((float) (-Math.PI / 2.0));
      }

      if (ImGui.button("Clear Planned footsteps"))
      {
         clearGoalFootstepsPublisher.publish(new Empty());
      }

      continuousHikingParameters.setStepPublisherEnabled(enableContinuousHiking.get());

      ImGui.indent();
      ImGui.checkbox("Use A* Planner", useAStarFootstepPlanner);
      ImGui.checkbox("Monte Carlo Planner", useMonteCarloFootstepPlanner);
      ImGui.unindent();

      if (ImGui.collapsingHeader("Continuous Hiking Parameters"))
      {
         continuousHikingParametersPanel.renderImGuiWidgets();
      }

      if (DEBUG)
      {
         ImGui.separator();

         if (ImGui.button("Fake Controller Drift"))
         {
            // Simulate that the controller started a step, this part triggers the drift offset kernel
            FootstepStatusMessage footstepStatusMessage = new FootstepStatusMessage();
            footstepStatusMessage.setFootstepStatus(FootstepStatusMessage.FOOTSTEP_STATUS_STARTED);
            footstepStatusMessagePublisher.publish(footstepStatusMessage);

            // Simulate that the controller has drifted by some z value
            PlanOffsetStatus planOffsetStatus = new PlanOffsetStatus();
            Vector3D planOffset = new Vector3D(0, 0, simulatedDriftInMeters);
            planOffsetStatus.getOffsetVector().set(planOffset);
            LogTools.info("Plan Offset Status: " + planOffsetStatus.getOffsetVector());
            planOffsetStatusPublisher.publish(planOffsetStatus);

            // The amount of drift that we want to simulation and adjust for if we do this over and over
            if (simulatedDriftInMeters > -1.0)
            {
               simulatedDriftInMeters -= 0.1;
            }
            else
            {
               simulatedDriftInMeters += 0.1;
            }
         }

         ImGui.sameLine();

         if (ImGui.button("Fake Robot Falling"))
         {
            WalkingControllerFailureStatusMessage walkingControllerFailureStatusMessage = new WalkingControllerFailureStatusMessage();
            walkingControllerFailureStatusPublisher.publish(walkingControllerFailureStatusMessage);
         }
      }

      // Check to see if a controller is plugged into the computer
      Controller joystickController = Controllers.getCurrent();
      // Here we check against null rather then .isConnected() because if the controller is unplugged, that method won't work
      boolean controllerConnected = joystickController != null;

//      if (controllerConnected)
//      {
//         performJoystickControllerAction(joystickController);
//      }
   }

   private void performJoystickControllerAction(Controller joystickController)
   {
      boolean currentYButtonPressed = joystickController.getButton(ControllerButton.Y.ordinal());
      boolean currentLeftBumper = joystickController.getButton(ControllerButton.LEFTBUMPER.ordinal());
      boolean currentRightBumper = joystickController.getButton(ControllerButton.RIGHTBUMPER.ordinal());
      boolean currentStartButton = joystickController.getButton(ControllerButton.START.ordinal());

      if (previousStartButton && !currentStartButton)
      {
         // This sets the value to the opposite value
         enableContinuousHiking.set(!enableContinuousHiking.get());
      }

      if (previousYButton && !currentYButtonPressed)
      {
         squareUpPublisher.publish(new Empty());
      }

      if (previousLeftBumper && !currentLeftBumper)
      {
         turnRobot((float) (Math.PI / 2.0));
      }

      if (previousRightBumper && !currentRightBumper)
      {
         turnRobot((float) (-Math.PI / 2.0));
      }

      if (joystickController.getButton(joystickController.getMapping().buttonA))
      {
         publishJoystickStatus(joystickController);
      }

      if (joystickController.getButton(joystickController.getMapping().buttonB) && joystickController.getButton(ControllerButton.DPAD_DOWN.ordinal()))
      {
         publishStopContinuousHiking(squareUpToGoal.get());
      }
      else if (joystickController.getButton(joystickController.getMapping().buttonB))
      {
         publishStopContinuousHiking(false);
      }

      previousStartButton = joystickController.getButton(ControllerButton.START.ordinal());
      previousYButton = joystickController.getButton(ControllerButton.Y.ordinal());
      previousLeftBumper = joystickController.getButton(ControllerButton.LEFTBUMPER.ordinal());
      previousRightBumper = joystickController.getButton(ControllerButton.RIGHTBUMPER.ordinal());
   }

   public void processImGui3DViewInput(ImGui3DViewInput input)
   {
      stancePoseSelectionPanel.processImGui3DViewInput(input);
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      stancePoseSelectionPanel.getRenderables(renderables, pool);
      terrainPlanningDebugger.getRenderables(renderables, pool);
   }

   public void turnRobot(float rotationRadians)
   {
      Float32 rotationInRadians = new Float32();
      rotationInRadians.setData(rotationRadians);
      turn90DegreesPublisher.publish(rotationInRadians);
   }

   /**
    * We have received the start and goal pose from the process, lets unpack this message and visualize the start and goal on the UI.
    */
   public void onStartAndGoalPosesReceived(PoseListMessage poseListMessage)
   {
      SideDependentList<FramePose3D> goalStancePose = new SideDependentList<>(new FramePose3D(), new FramePose3D());
      SideDependentList<FramePose3D> startStancePose = new SideDependentList<>(new FramePose3D(), new FramePose3D());

      if (!poseListMessage.getPoses().isEmpty())
      {
         List<Pose3D> poses = MessageTools.unpackPoseListMessage(poseListMessage);
         startStancePose.get(RobotSide.LEFT).set(poses.get(0));
         startStancePose.get(RobotSide.RIGHT).set(poses.get(1));
         goalStancePose.get(RobotSide.LEFT).set(poses.get(2));
         goalStancePose.get(RobotSide.RIGHT).set(poses.get(3));
      }

      // Visualize the start and goal poses on the UI
      terrainPlanningDebugger.generateStartAndGoalFootstepGraphics(startStancePose, goalStancePose);

      this.startStancePose = startStancePose;
   }

   public void onPlannedFootstepsReceived(FootstepDataListMessage footstepDataListMessage)
   {
      FootstepPlan footstepPLan = new FootstepPlan();
      List<EnumMap<Axis3D, List<PolynomialReadOnly>>> swingTrajectories = new ArrayList<>();

      if (!footstepDataListMessage.getFootstepDataList().isEmpty())
      {
         LogTools.info("Received footstep plan: {}", footstepDataListMessage.getFootstepDataList().size());

         footstepPLan = FootstepDataMessageConverter.convertToFootstepPlan(footstepDataListMessage);
         swingTrajectories = SwingPlannerTools.computeTrajectories(swingTrajectoryParameters, positionTrajectoryGenerator, startStancePose, footstepPLan);
      }

      terrainPlanningDebugger.generateFootstepPlanGraphic(footstepDataListMessage);

      this.latestFootstepPlan = footstepPLan;
      this.swingTrajectories = swingTrajectories;
   }

   public void onMonteCarloPlanReceived(FootstepDataListMessage message)
   {
      LogTools.debug("Received Monte-Carlo Plan: {}", message.getFootstepDataList().size());
      terrainPlanningDebugger.generateMonteCarloPlanGraphic(message);
   }

   /**
    * Publish the status of the joystick controller. We define different buttons to perform different actions which get sent with the message.
    */
   private void publishJoystickStatus(Controller joystickController)
   {
      // Setup variables to be published in the message
      boolean walkBackwards;
      double forwardJoystickValue;
      double lateralJoystickValue;
      double turningJoystickValue;

      walkBackwards = joystickController.getButton(joystickController.getMapping().buttonX);
      forwardJoystickValue = -joystickController.getAxis(joystickController.getMapping().axisLeftY);
      lateralJoystickValue = -joystickController.getAxis(joystickController.getMapping().axisLeftX);
      turningJoystickValue = -joystickController.getAxis(joystickController.getMapping().axisRightX);

      commandMessage.setEnableContinuousHiking(true);
      commandMessage.setUseJoystickController(true);
      commandMessage.setForwardValue(forwardJoystickValue);
      commandMessage.setWalkBackwards(walkBackwards);
      commandMessage.setLateralValue(lateralJoystickValue);
      commandMessage.setTurningValue(turningJoystickValue);

      commandPublisher.publish(commandMessage);
   }

   /**
    * This publishes and tells the state machine that we want to start walking. Setting the enable Continuous Hiking to true
    */
   private void publishStartContinuousHiking(boolean walkForward, boolean walkBackwards)
   {
      commandMessage.setEnableContinuousHiking(true);
      commandMessage.setWalkForwards(walkForward);
      commandMessage.setSideStep(false);
      commandMessage.setLeftDirection(false);
      commandMessage.setSquareUpToGoal(squareUpToGoal.get());
      commandMessage.setUseAstarFootstepPlanner(useAStarFootstepPlanner.get());
      commandMessage.setUseMonteCarloFootstepPlanner(useMonteCarloFootstepPlanner.get());
      commandMessage.setUseMonteCarloPlanAsReference(useMonteCarloReference.get());
      commandMessage.setUsePreviousPlanAsReference(!useMonteCarloReference.get());

      commandMessage.setUseJoystickController(false);
      commandMessage.setForwardValue(0.0);
      commandMessage.setWalkBackwards(walkBackwards);
      commandMessage.setLateralValue(0.0);
      commandMessage.setTurningValue(0.0);

      commandPublisher.publish(commandMessage);
   }

   /**
    * Stop Continuous Hiking. Tells the state machine that we want to stop walking
    */
   private void publishStopContinuousHiking(boolean squareUpAfterLastStep)
   {
      commandMessage.setEnableContinuousHiking(false);
      commandMessage.setSquareUpToGoal(squareUpAfterLastStep);
      commandPublisher.publish(commandMessage);
   }

   private void publishContinuousHikingCommandSideStepEnabled(boolean leftDirection)
   {
      commandMessage.setEnableContinuousHiking(true);
      commandMessage.setWalkForwards(true);
      commandMessage.setSideStep(true);
      commandMessage.setLeftDirection(leftDirection);
      commandMessage.setSquareUpToGoal(squareUpToGoal.get());
      commandMessage.setUseAstarFootstepPlanner(useAStarFootstepPlanner.get());
      commandMessage.setUseMonteCarloFootstepPlanner(useMonteCarloFootstepPlanner.get());
      commandMessage.setUseMonteCarloPlanAsReference(useMonteCarloReference.get());
      commandMessage.setUsePreviousPlanAsReference(!useMonteCarloReference.get());

      commandMessage.setUseJoystickController(false);
      commandMessage.setForwardValue(0.0);
      commandMessage.setWalkBackwards(false);
      commandMessage.setLateralValue(0.0);
      commandMessage.setTurningValue(0.0);

      commandPublisher.publish(commandMessage);
   }

   public void destroy()
   {
      commandPublisher.remove();
      stancePoseSelectionPanel.destroy();
      terrainPlanningDebugger.destroy();
   }

   public RDXStancePoseSelectionPanel getStancePoseSelectionPanel()
   {
      return stancePoseSelectionPanel;
   }
}