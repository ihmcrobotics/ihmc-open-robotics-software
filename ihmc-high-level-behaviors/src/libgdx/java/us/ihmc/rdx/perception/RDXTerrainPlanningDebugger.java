package us.ihmc.rdx.perception;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import controller_msgs.msg.dds.FootstepDataListMessage;
import ihmc_common_msgs.msg.dds.PoseListMessage;
import us.ihmc.commons.MathTools;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.footstepPlanning.FootstepDataMessageConverter;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.MonteCarloFootstepPlannerParameters;
import us.ihmc.footstepPlanning.communication.ContinuousWalkingAPI;
import us.ihmc.footstepPlanning.monteCarloPlanning.MonteCarloPlannerTools;
import us.ihmc.log.LogTools;
import us.ihmc.perception.heightMap.TerrainMapData;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

public class RDXTerrainPlanningDebugger implements RenderableProvider
{
   private final AtomicReference<FootstepDataListMessage> monteCarloPlanDataListMessage = new AtomicReference<>(null);
   private MonteCarloFootstepPlannerParameters monteCarloFootstepPlannerParameters;

   private SideDependentList<ArrayList<ModelInstance>> expansionSpheres = new SideDependentList<>(new ArrayList<>(), new ArrayList<>());
   private ArrayList<ModelInstance> stateSpheres = new ArrayList<>();
   private ArrayList<ModelInstance> stepCylinders = new ArrayList<>();
   private List<Pose3D> monteCarloTreeNodeStates;
   private TerrainMapData terrainMapData;

   private int leftIndex = 0;
   private int rightIndex = 0;
   private boolean showStateSpheres = true;
   private boolean showExpansionSpheres = true;

   public RDXTerrainPlanningDebugger(ROS2Helper ros2Helper, MonteCarloFootstepPlannerParameters monteCarloFootstepPlannerParameters)
   {
      this.monteCarloFootstepPlannerParameters = monteCarloFootstepPlannerParameters;
      ros2Helper.subscribeViaCallback(ContinuousWalkingAPI.MONTE_CARLO_TREE_NODES, this::onMonteCarloTreeNodesReceived);
      ros2Helper.subscribeViaCallback(ContinuousWalkingAPI.MONTE_CARLO_FOOTSTEP_PLAN, this::onMonteCarloPlanReceived);

      for (int i = 0; i < 500; i++)
      {
         expansionSpheres.get(RobotSide.LEFT).add(RDXModelBuilder.createSphere(0.015f, Color.CYAN));
         expansionSpheres.get(RobotSide.RIGHT).add(RDXModelBuilder.createSphere(0.015f, Color.RED));
      }
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

   public void render(TerrainMapData terrainMapData, boolean showStateSpheres, boolean showExpansionSpheres)
   {
      this.showStateSpheres = showStateSpheres;
      this.showExpansionSpheres = showExpansionSpheres;

      if (monteCarloTreeNodeStates != null && showStateSpheres)
      {
         updateStateSpheres(monteCarloTreeNodeStates);
         monteCarloTreeNodeStates = null;
      }

      if (monteCarloPlanDataListMessage.get() != null && showExpansionSpheres)
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
      if (showStateSpheres)
      {
         for (ModelInstance stateSphere : stateSpheres)
         {
            stateSphere.getRenderables(renderables, pool);
         }
      }

      if (showExpansionSpheres)
      {
         for (RobotSide side : RobotSide.values)
         {
            for (ModelInstance expansionSphere : expansionSpheres.get(side))
            {
               expansionSphere.getRenderables(renderables, pool);
            }
         }
      }
   }

   public void destroy()
   {
   }

   public void setShowStateSpheres(boolean showStateSpheres)
   {
      this.showStateSpheres = showStateSpheres;
   }

   public void setShowExpansionSpheres(boolean showExpansionSpheres)
   {
      this.showExpansionSpheres = showExpansionSpheres;
   }
}
