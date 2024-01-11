package us.ihmc.rdx.perception;

import behavior_msgs.msg.dds.ContinuousWalkingCommandMessage;
import com.badlogic.gdx.controllers.Controller;
import com.badlogic.gdx.controllers.Controllers;
import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.WalkingControllerFailureStatusMessage;
import ihmc_common_msgs.msg.dds.PoseListMessage;
import imgui.ImGui;
import imgui.type.ImBoolean;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.activeMapping.ContinuousWalkingParameters;
import us.ihmc.commonWalkingControlModules.configurations.SwingTrajectoryParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.commonWalkingControlModules.trajectories.PositionOptimizedTrajectoryGenerator;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.communication.video.ContinuousPlanningAPI;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.footstepPlanning.FootstepDataMessageConverter;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.swing.SwingPlannerParametersBasics;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.footstepPlanning.tools.SwingPlannerTools;
import us.ihmc.log.LogTools;
import us.ihmc.perception.HumanoidActivePerceptionModule;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.ui.graphics.RDXFootstepGraphic;
import us.ihmc.rdx.ui.graphics.RDXFootstepPlanGraphic;
import us.ihmc.robotics.math.trajectories.interfaces.PolynomialReadOnly;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SegmentDependentList;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

public class RDXContinuousWalkingPanel extends RDXPanel implements RenderableProvider
{
   private ContinuousWalkingCommandMessage commandMessage = new ContinuousWalkingCommandMessage();
   private final IHMCROS2Publisher<ContinuousWalkingCommandMessage> commandPublisher;
   private final RDXFootstepPlanGraphic footstepPlanGraphic = new RDXFootstepPlanGraphic(PlannerTools.createFootPolygons(0.2, 0.1, 0.08));
   private final RDXFootstepPlanGraphic monteCarloPlanGraphic = new RDXFootstepPlanGraphic(PlannerTools.createFootPolygons(0.2, 0.1, 0.08));
   private final SideDependentList<RDXFootstepGraphic> goalFootstepGraphics;
   private final SideDependentList<RDXFootstepGraphic> startFootstepGraphics;
   private final ImBoolean renderEnabled = new ImBoolean(true);
   private final ImBoolean showMonteCarloPlan = new ImBoolean(false);
   private final ImBoolean showContinuousWalkingPlan = new ImBoolean(true);
   private final ImBoolean localRenderMode = new ImBoolean(false);
   private final ImBoolean useMonteCarloReference = new ImBoolean(true);

   private final AtomicReference<FootstepDataListMessage> footstepDataListMessage = new AtomicReference<>(null);
   private final AtomicReference<FootstepDataListMessage> monteCarloPlanDataListMessage = new AtomicReference<>(null);
   private final SideDependentList<FramePose3D> startStancePose = new SideDependentList<>(new FramePose3D(), new FramePose3D());
   private final SideDependentList<FramePose3D> goalStancePose = new SideDependentList<>(new FramePose3D(), new FramePose3D());

   private static final int numberOfKnotPoints = 12;
   private static final int maxIterationsOptimization = 100;
   private final PositionOptimizedTrajectoryGenerator positionTrajectoryGenerator = new PositionOptimizedTrajectoryGenerator(numberOfKnotPoints,
                                                                                                                             maxIterationsOptimization);

   private final ROS2Helper ros2Helper;
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
                                    SwingTrajectoryParameters swingTrajectoryParameters)
   {
      super("Continuous Planning");
      setRenderMethod(this::renderImGuiWidgets);

      this.ros2Helper = ros2Helper;
      this.activePerceptionModule = activePerceptionModule;
      this.swingPlannerParameters = swingPlannerParameters;
      this.swingTrajectoryParameters = swingTrajectoryParameters;
      this.continuousWalkingParameters = continuousWalkingParameters;

      SegmentDependentList<RobotSide, ArrayList<Point2D>> contactPoints = syncedRobot.getRobotModel()
                                                                                     .getContactPointParameters()
                                                                                     .getControllerFootGroundContactPoints();
      goalFootstepGraphics = new SideDependentList<>(new RDXFootstepGraphic(contactPoints, RobotSide.LEFT),
                                                     new RDXFootstepGraphic(contactPoints, RobotSide.RIGHT));
      startFootstepGraphics = new SideDependentList<>(new RDXFootstepGraphic(contactPoints, RobotSide.LEFT),
                                                      new RDXFootstepGraphic(contactPoints, RobotSide.RIGHT));

      goalFootstepGraphics.get(RobotSide.RIGHT).setColor(new Color(1.0f, 1.0f, 1.0f, 0.5f));
      goalFootstepGraphics.get(RobotSide.RIGHT).create();

      goalFootstepGraphics.get(RobotSide.LEFT).setColor(new Color(1.0f, 1.0f, 1.0f, 0.5f));
      goalFootstepGraphics.get(RobotSide.LEFT).create();

      startFootstepGraphics.get(RobotSide.RIGHT).setColor(new Color(0.0f, 0.0f, 0.0f, 1.0f));
      startFootstepGraphics.get(RobotSide.RIGHT).create();

      startFootstepGraphics.get(RobotSide.LEFT).setColor(new Color(0.0f, 0.0f, 0.0f, 1.0f));
      startFootstepGraphics.get(RobotSide.LEFT).create();

      footstepPlanGraphic.setColor(RobotSide.LEFT, Color.GRAY);
      footstepPlanGraphic.setColor(RobotSide.RIGHT, Color.BLUE);

      monteCarloPlanGraphic.setColor(RobotSide.LEFT, Color.TEAL);
      monteCarloPlanGraphic.setColor(RobotSide.RIGHT, Color.GREEN);

      ros2Helper.subscribeViaCallback(ContinuousPlanningAPI.START_AND_GOAL_FOOTSTEPS, this::onStartAndGoalPosesReceived);
      ros2Helper.subscribeViaCallback(ContinuousPlanningAPI.PLANNED_FOOTSTEPS, this::onPlannedFootstepsReceived);
      ros2Helper.subscribeViaCallback(ContinuousPlanningAPI.MONTE_CARLO_FOOTSTEP_PLAN, this::onMonteCarloPlanReceived);
      ros2Helper.subscribeViaCallback(ControllerAPIDefinition.getTopic(FootstepDataListMessage.class, syncedRobot.getRobotModel().getSimpleRobotName()), this::onControllerFootstepsReceived);

      commandPublisher = ROS2Tools.createPublisher(ros2Helper.getROS2NodeInterface(), ContinuousPlanningAPI.CONTINUOUS_WALKING_COMMAND);

      ros2Helper.subscribeViaCallback(ControllerAPIDefinition.getTopic(WalkingControllerFailureStatusMessage.class, syncedRobot.getRobotModel().getSimpleRobotName()), message ->
      {
         reset();
      });
   }

   public void generateStartAndGoalFootstepGraphics()
   {
      for (RobotSide side : RobotSide.values)
      {
         startFootstepGraphics.get(side).setPose(startStancePose.get(side));
         goalFootstepGraphics.get(side).setPose(goalStancePose.get(side));
      }
   }

   public void generateSwingGraphics(FootstepPlan plan, List<EnumMap<Axis3D, List<PolynomialReadOnly>>> swingTrajectories)
   {
      if (plan != null)
      {
         footstepPlanGraphic.updateTrajectoriesFromPlan(plan, swingTrajectories);
      }
   }

   public void generateFootstepPlanGraphic(FootstepDataListMessage message)
   {
      footstepPlanGraphic.generateMeshesAsync(message, "Continuous Walking");
      footstepPlanGraphic.update();
   }

   public void generateMonteCarloPlanGraphic(FootstepDataListMessage message)
   {
      monteCarloPlanGraphic.generateMeshesAsync(message, "Monte-Carlo Plan");
      monteCarloPlanGraphic.update();
   }

   public void update()
   {
      if (!renderEnabled.get())
         return;

      if (footstepDataListMessage.get() != null)
      {
         generateFootstepPlanGraphic(footstepDataListMessage.get());
         if (activePerceptionModule != null && localRenderMode.get())
         {
            generateSwingGraphics(activePerceptionModule.getContinuousPlannerSchedulingTask().getContinuousPlanner().getLatestFootstepPlan(),
                                  activePerceptionModule.getContinuousPlannerSchedulingTask().getContinuousPlanner().getLatestSwingTrajectories());
         }
         else
         {
            FootstepPlan plan = FootstepDataMessageConverter.convertToFootstepPlan(footstepDataListMessage.get());
            List<EnumMap<Axis3D, List<PolynomialReadOnly>>> swingTrajectories = SwingPlannerTools.computeTrajectories(swingPlannerParameters,
                                                                                                                      swingTrajectoryParameters,
                                                                                                                      positionTrajectoryGenerator,
                                                                                                                      startStancePose,
                                                                                                                      plan);
            generateSwingGraphics(plan, swingTrajectories);
         }
         footstepDataListMessage.set(null);
      }

      if (monteCarloPlanDataListMessage.get() != null)
      {
         generateMonteCarloPlanGraphic(monteCarloPlanDataListMessage.get());
         monteCarloPlanDataListMessage.set(null);
      }

      generateStartAndGoalFootstepGraphics();
   }

   public void renderImGuiWidgets()
   {
      ImGui.checkbox("Render", renderEnabled);
      ImGui.checkbox("Use Monte-Carlo Reference", useMonteCarloReference);
      ImGui.checkbox("Local Render Mode", localRenderMode);
      ImGui.checkbox("Show Monte-Carlo Plan", showMonteCarloPlan);
      ImGui.checkbox("Show Continuous Walking Plan", showContinuousWalkingPlan);

      publishInputCommandMessage();
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (renderEnabled.get())
      {
         if (showMonteCarloPlan.get())
            monteCarloPlanGraphic.getRenderables(renderables, pool);

         if (showContinuousWalkingPlan.get())
            footstepPlanGraphic.getRenderables(renderables, pool);

         goalFootstepGraphics.get(RobotSide.LEFT).getRenderables(renderables, pool);
         goalFootstepGraphics.get(RobotSide.RIGHT).getRenderables(renderables, pool);
         startFootstepGraphics.get(RobotSide.LEFT).getRenderables(renderables, pool);
         startFootstepGraphics.get(RobotSide.RIGHT).getRenderables(renderables, pool);
      }
   }

   public void onPlannedFootstepsReceived(FootstepDataListMessage message)
   {
      LogTools.warn("Received footstep plan: {}", message.getFootstepDataList().size());
      this.footstepDataListMessage.set(message);
   }

   public void onMonteCarloPlanReceived(FootstepDataListMessage message)
   {
      LogTools.warn("Received Monte-Carlo Plan: {}", message.getFootstepDataList().size());
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

   public void reset()
   {
      footstepPlanGraphic.clear();
      monteCarloPlanGraphic.clear();

      goalFootstepGraphics.get(RobotSide.LEFT).setPose(new FramePose3D());
      goalFootstepGraphics.get(RobotSide.RIGHT).setPose(new FramePose3D());
      startFootstepGraphics.get(RobotSide.LEFT).setPose(new FramePose3D());
      startFootstepGraphics.get(RobotSide.RIGHT).setPose(new FramePose3D());
   }

   public void destroy()
   {
      footstepPlanGraphic.destroy();
      monteCarloPlanGraphic.destroy();
      goalFootstepGraphics.get(RobotSide.LEFT).destroy();
      goalFootstepGraphics.get(RobotSide.RIGHT).destroy();
      startFootstepGraphics.get(RobotSide.LEFT).destroy();
      startFootstepGraphics.get(RobotSide.RIGHT).destroy();
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
      if (continuousWalkingParameters.getEnableContinuousWalking())
      {
         commandMessage.setEnableContinuousWalking(walkingEnabled);
         commandMessage.setPublishToController(ImGui.getIO().getKeyAlt());
         commandMessage.setForwardValue(forwardJoystickValue);
         commandMessage.setLateralValue(lateralJoystickValue);
         commandMessage.setTurningValue(turningJoystickValue);
         commandPublisher.publish(commandMessage);
      }
   }
}
