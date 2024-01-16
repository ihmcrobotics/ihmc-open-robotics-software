package us.ihmc.rdx.perception;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import controller_msgs.msg.dds.FootstepDataListMessage;
import ihmc_common_msgs.msg.dds.PoseListMessage;
import imgui.ImGui;
import imgui.type.ImBoolean;
import us.ihmc.commons.MathTools;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.footstepPlanning.FootstepDataMessageConverter;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.MonteCarloFootstepPlannerParameters;
import us.ihmc.footstepPlanning.communication.ContinuousWalkingAPI;
import us.ihmc.footstepPlanning.monteCarloPlanning.MonteCarloPlannerTools;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.log.LogTools;
import us.ihmc.perception.heightMap.TerrainMapData;
import us.ihmc.rdx.tools.RDXModelBuilder;
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

public class RDXTerrainPlanningDebugger implements RenderableProvider
{
   private final RDXFootstepPlanGraphic footstepPlanGraphic = new RDXFootstepPlanGraphic(PlannerTools.createFootPolygons(0.2, 0.1, 0.08));
   private final RDXFootstepPlanGraphic monteCarloPlanGraphic = new RDXFootstepPlanGraphic(PlannerTools.createFootPolygons(0.2, 0.1, 0.08));
   private final AtomicReference<FootstepDataListMessage> monteCarloPlanDataListMessage = new AtomicReference<>(null);
   private final SideDependentList<ArrayList<ModelInstance>> expansionSpheres = new SideDependentList<>(new ArrayList<>(), new ArrayList<>());
   private final ArrayList<ModelInstance> stateSpheres = new ArrayList<>();
   private final ImBoolean showMonteCarloPlan = new ImBoolean(true);
   private final ImBoolean showContinuousWalkingPlan = new ImBoolean(true);
   private final ImBoolean showStateSpheres = new ImBoolean(true);
   private final ImBoolean showExpansionSpheres = new ImBoolean(true);
   private final ImBoolean showStartPoses = new ImBoolean(true);
   private final ImBoolean showGoalPoses = new ImBoolean(true);
   private final ImBoolean showRightActionSet = new ImBoolean(false);
   private final ImBoolean showLeftActionSet = new ImBoolean(false);

   private MonteCarloFootstepPlannerParameters monteCarloFootstepPlannerParameters;
   private SideDependentList<RDXFootstepGraphic> goalFootstepGraphics;
   private SideDependentList<RDXFootstepGraphic> startFootstepGraphics;
   private List<Pose3D> monteCarloTreeNodeStates;
   private TerrainMapData terrainMapData;

   private int leftIndex = 0;
   private int rightIndex = 0;

   public RDXTerrainPlanningDebugger(ROS2Helper ros2Helper,
                                     MonteCarloFootstepPlannerParameters monteCarloFootstepPlannerParameters,
                                     SegmentDependentList<RobotSide, ArrayList<Point2D>> contactPoints)
   {

      this.monteCarloFootstepPlannerParameters = monteCarloFootstepPlannerParameters;
      ros2Helper.subscribeViaCallback(ContinuousWalkingAPI.MONTE_CARLO_TREE_NODES, this::onMonteCarloTreeNodesReceived);
      ros2Helper.subscribeViaCallback(ContinuousWalkingAPI.MONTE_CARLO_FOOTSTEP_PLAN, this::onMonteCarloPlanReceived);

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

      monteCarloPlanGraphic.setColor(RobotSide.LEFT, Color.CYAN);
      monteCarloPlanGraphic.setColor(RobotSide.RIGHT, Color.RED);

      for (int i = 0; i < 500; i++)
      {
         expansionSpheres.get(RobotSide.LEFT).add(RDXModelBuilder.createSphere(0.015f, Color.CYAN));
         expansionSpheres.get(RobotSide.RIGHT).add(RDXModelBuilder.createSphere(0.015f, Color.RED));
      }
   }

   public void generateStartAndGoalFootstepGraphics(SideDependentList<FramePose3D> startStancePose, SideDependentList<FramePose3D> goalStancePose)
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
      FootstepPlan plan = FootstepDataMessageConverter.convertToFootstepPlan(message);
      for (int i = 0; i < plan.getNumberOfSteps(); i++)
      {
         LogTools.info("({})[A* Footstep: {}, {}]", i, plan.getFootstep(i).getRobotSide(), plan.getFootstep(i).getFootstepPose());
      }

      footstepPlanGraphic.generateMeshesAsync(message, "Continuous Walking");
      footstepPlanGraphic.update();
   }

   public void generateMonteCarloPlanGraphic(FootstepDataListMessage message)
   {
      FootstepPlan plan = FootstepDataMessageConverter.convertToFootstepPlan(message);
      for (int i = 0; i < plan.getNumberOfSteps(); i++)
      {
         LogTools.info("({})[Monte-Carlo Footstep: {}, {}]", i, plan.getFootstep(i).getRobotSide(), plan.getFootstep(i).getFootstepPose());
      }

      monteCarloPlanGraphic.generateMeshesAsync(message, "Monte-Carlo Plan");
      monteCarloPlanGraphic.update();
   }

   public void onMonteCarloTreeNodesReceived(PoseListMessage poseListMessage)
   {
      LogTools.info("Received Monte-Carlo Tree Nodes: {}", poseListMessage.getPoses().size());
      if (monteCarloTreeNodeStates == null)
      {
         monteCarloTreeNodeStates = MessageTools.unpackPoseListMessage(poseListMessage);
      }
   }

   public void onMonteCarloPlanReceived(FootstepDataListMessage message)
   {
      LogTools.warn("Received Monte-Carlo Plan: {}", message.getFootstepDataList().size());
      this.monteCarloPlanDataListMessage.set(message);
   }

   public void render(TerrainMapData terrainMapData)
   {
      if (monteCarloTreeNodeStates != null && showStateSpheres.get())
      {
         updateStateSpheres(monteCarloTreeNodeStates);
         monteCarloTreeNodeStates = null;
      }

      if (monteCarloPlanDataListMessage.get() != null && showExpansionSpheres.get())
      {
         updateExpansionSpheres(monteCarloPlanDataListMessage.getAndSet(null), terrainMapData);
      }
   }

   public void updateExpansionSpheres(FootstepDataListMessage monteCarloFootstepsMessage, TerrainMapData terrainMapData)
   {
      resetExpansionSpherePositions();
      FootstepPlan footstepPlan = FootstepDataMessageConverter.convertToFootstepPlan(monteCarloFootstepsMessage);

      for (int i = 0; i < footstepPlan.getNumberOfSteps(); i++)
      {
         addExpansionSpheresFromActionSet(footstepPlan.getFootstep(i).getFootstepPose(), footstepPlan.getFootstep(i).getRobotSide(), terrainMapData);
      }
   }

   public void updateExpansionSpheres(FramePose3D footstepPose, RobotSide side, TerrainMapData terrainMapData)
   {
      addExpansionSpheresFromActionSet(footstepPose, side, terrainMapData);
   }

   public void addExpansionSpheresFromActionSet(FramePose3D footstepPose, RobotSide side, TerrainMapData terrainMapData)
   {
      ArrayList<Vector3D> actions = new ArrayList<>();
      MonteCarloPlannerTools.getFootstepActionSet(monteCarloFootstepPlannerParameters, actions, (float) footstepPose.getYaw(), side == RobotSide.LEFT ? -1 : 1);

      Vector3D footstepPosition = new Vector3D(footstepPose.getX(), footstepPose.getY(), 0.0f);

      for (Vector3D action : actions)
      {
         Vector3D spherePosition = new Vector3D(footstepPosition);
         action.scale(1 / 50.0f);
         spherePosition.add(action);

         float height = terrainMapData.getHeightInWorld((float) spherePosition.getX(), (float) spherePosition.getY());
         spherePosition.setZ(height + 0.01);

         if (side == RobotSide.LEFT && leftIndex < expansionSpheres.get(side).size())
         {
            ModelInstance sphere = expansionSpheres.get(side).get(leftIndex++);
            sphere.transform.setTranslation((float) spherePosition.getX(), (float) spherePosition.getY(), (float) spherePosition.getZ());
         }
         else if (side == RobotSide.RIGHT && rightIndex < expansionSpheres.get(side).size())
         {
            ModelInstance sphere = expansionSpheres.get(side).get(rightIndex++);
            sphere.transform.setTranslation((float) spherePosition.getX(), (float) spherePosition.getY(), (float) spherePosition.getZ());
         }
      }
   }

   public void resetExpansionSpherePositions()
   {
      for (RobotSide side : RobotSide.values)
      {
         for (ModelInstance expansionSphere : expansionSpheres.get(side))
         {
            expansionSphere.transform.setTranslation(0.0f, 0.0f, 0.0f);
         }
      }

      leftIndex = 0;
      rightIndex = 0;
   }

   public void updateStateSpheres(List<Pose3D> poses)
   {
      stateSpheres.clear();
      for (Pose3D pose : poses)
      {
         RobotSide side = pose.getPitch() > 0.0 ? RobotSide.LEFT : RobotSide.RIGHT;

         double radius = 0.01f + MathTools.clamp(pose.getYaw(), 0.0f, 10.0f) * 0.01f;
         //LogTools.warn("SPHERE RADIUS: " + radius);

         ModelInstance sphere;
         if (side == RobotSide.LEFT)
         {
            sphere = RDXModelBuilder.createSphere((float) radius, Color.CYAN);
         }
         else
         {
            sphere = RDXModelBuilder.createSphere((float) radius, Color.GREEN);
         }

         sphere.transform.setTranslation((float) pose.getX(), (float) pose.getY(), (float) pose.getZ());
         stateSpheres.add(sphere);
      }
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (showStateSpheres.get())
      {
         for (ModelInstance stateSphere : stateSpheres)
         {
            stateSphere.getRenderables(renderables, pool);
         }
      }

      if (showExpansionSpheres.get())
      {
         for (RobotSide side : RobotSide.values)
         {
            for (ModelInstance expansionSphere : expansionSpheres.get(side))
            {
               expansionSphere.getRenderables(renderables, pool);
            }
         }
      }

      if (showMonteCarloPlan.get())
         monteCarloPlanGraphic.getRenderables(renderables, pool);

      if (showContinuousWalkingPlan.get())
         footstepPlanGraphic.getRenderables(renderables, pool);

      goalFootstepGraphics.get(RobotSide.LEFT).getRenderables(renderables, pool);
      goalFootstepGraphics.get(RobotSide.RIGHT).getRenderables(renderables, pool);
      startFootstepGraphics.get(RobotSide.LEFT).getRenderables(renderables, pool);
      startFootstepGraphics.get(RobotSide.RIGHT).getRenderables(renderables, pool);
   }

   public void renderImGuiWidgets()
   {
      ImGui.checkbox("Show Monte-Carlo Plan", showMonteCarloPlan);
      ImGui.checkbox("Show Continuous Walking Plan", showContinuousWalkingPlan);
      ImGui.checkbox("Show State Spheres", showStateSpheres);
      ImGui.checkbox("Show Expansion Spheres", showExpansionSpheres);
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
}
