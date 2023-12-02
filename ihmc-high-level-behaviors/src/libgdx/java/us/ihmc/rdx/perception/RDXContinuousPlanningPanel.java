package us.ihmc.rdx.perception;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import controller_msgs.msg.dds.FootstepDataListMessage;
import ihmc_common_msgs.msg.dds.PoseListMessage;
import imgui.type.ImBoolean;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.activeMapping.ContinuousWalkingParameters;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.communication.video.ContinuousPlanningAPI;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.footstepPlanning.FootstepDataMessageConverter;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.perception.HumanoidActivePerceptionModule;
import us.ihmc.rdx.ui.graphics.RDXFootstepGraphic;
import us.ihmc.rdx.ui.graphics.RDXFootstepPlanGraphic;
import us.ihmc.rdx.ui.graphics.RDXSwingTrajectoryGraphic;
import us.ihmc.robotics.math.trajectories.interfaces.PolynomialReadOnly;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SegmentDependentList;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

public class RDXContinuousPlanningPanel implements RenderableProvider
{
   private final RDXSwingTrajectoryGraphic swingTrajectoryGraphic = new RDXSwingTrajectoryGraphic();
   private final RDXFootstepPlanGraphic footstepPlanGraphic = new RDXFootstepPlanGraphic(PlannerTools.createFootPolygons(0.2, 0.1, 0.08));
   private final ImBoolean renderEnabled = new ImBoolean(true);
   private final SideDependentList<RDXFootstepGraphic> goalFootstepGraphics;
   private final SideDependentList<RDXFootstepGraphic> startFootstepGraphics;

   private final AtomicReference<FootstepDataListMessage> footstepDataListMessage = new AtomicReference<>(null);
   private final SideDependentList<FramePose3D> startStancePose = new SideDependentList<>(new FramePose3D(), new FramePose3D());
   private final SideDependentList<FramePose3D> goalStancePose = new SideDependentList<>(new FramePose3D(), new FramePose3D());

   private final ROS2Helper ros2Helper;
   private HumanoidActivePerceptionModule activePerceptionModule;

   public RDXContinuousPlanningPanel(ROS2Helper ros2Helper, HumanoidActivePerceptionModule activePerceptionModule, ROS2SyncedRobotModel syncedRobot)
   {
      this.ros2Helper = ros2Helper;
      this.activePerceptionModule = activePerceptionModule;

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

      ros2Helper.subscribeViaCallback(ContinuousPlanningAPI.START_AND_GOAL_FOOTSTEPS, this::onStartAndGoalPosesReceived);
      ros2Helper.subscribeViaCallback(ContinuousPlanningAPI.PLANNED_FOOTSTEPS, this::onPlannedFootstepsReceived);
   }

   public void generateStartAndGoalFootstepGraphics()
   {
      for (RobotSide side : RobotSide.values)
      {
         startFootstepGraphics.get(side).setPose(startStancePose.get(side));
         goalFootstepGraphics.get(side).setPose(goalStancePose.get(side));
      }
   }

   public void generateSwingGraphics(FootstepDataListMessage message)
   {
      FootstepPlan plan = FootstepDataMessageConverter.convertToFootstepPlan(message);

      //swingTrajectoryGraphic.updateFromPlan(plan);
   }

   public void generateSwingGraphics(FootstepPlan plan, List<EnumMap<Axis3D, List<PolynomialReadOnly>>> swingTrajectories)
   {
      if (plan != null)
      {
         swingTrajectoryGraphic.updateFromPlan(plan, swingTrajectories);
      }
   }

   public void render()
   {
      if (activePerceptionModule != null)
      {
         generateSwingGraphics(activePerceptionModule.getContinuousPlannerSchedulingTask().getContinuousPlanner().getLatestFootstepPlan(),
                               activePerceptionModule.getContinuousPlannerSchedulingTask().getContinuousPlanner().getLatestSwingTrajectories());
      }

      generateStartAndGoalFootstepGraphics();
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (renderEnabled.get())
      {
         swingTrajectoryGraphic.getRenderables(renderables, pool);
         footstepPlanGraphic.getRenderables(renderables, pool);
         goalFootstepGraphics.get(RobotSide.LEFT).getRenderables(renderables, pool);
         goalFootstepGraphics.get(RobotSide.RIGHT).getRenderables(renderables, pool);
         startFootstepGraphics.get(RobotSide.LEFT).getRenderables(renderables, pool);
         startFootstepGraphics.get(RobotSide.RIGHT).getRenderables(renderables, pool);
      }
   }

   public void onPlannedFootstepsReceived(FootstepDataListMessage message)
   {
      this.footstepDataListMessage.set(message);
   }

   public void onStartAndGoalPosesReceived(PoseListMessage poseListMessage)
   {
      List<Pose3D> poses = MessageTools.unpackPoseListMessage(poseListMessage);
      startStancePose.get(RobotSide.LEFT).set(poses.get(0));
      startStancePose.get(RobotSide.RIGHT).set(poses.get(1));
      goalStancePose.get(RobotSide.LEFT).set(poses.get(2));
      goalStancePose.get(RobotSide.RIGHT).set(poses.get(3));
   }

   public void destroy()
   {
      swingTrajectoryGraphic.destroy();
      footstepPlanGraphic.destroy();
      goalFootstepGraphics.get(RobotSide.LEFT).destroy();
      goalFootstepGraphics.get(RobotSide.RIGHT).destroy();
      startFootstepGraphics.get(RobotSide.LEFT).destroy();
      startFootstepGraphics.get(RobotSide.RIGHT).destroy();
   }
}
