package us.ihmc.rdx.perception;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import ihmc_common_msgs.msg.dds.PoseListMessage;
import us.ihmc.commons.MathTools;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.communication.video.ContinuousPlanningAPI;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.log.LogTools;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.ArrayList;
import java.util.List;

public class RDXContinuousWalkingDebugger implements RenderableProvider
{
   private ArrayList<ModelInstance> expansionSpheres = new ArrayList<>();
   private ArrayList<ModelInstance> stateSpheres = new ArrayList<>();
   private ArrayList<ModelInstance> stepCylinders = new ArrayList<>();
   private ImGui3DViewInput latestInput;
   private List<Pose3D> monteCarloTreeNodeStates;

   public RDXContinuousWalkingDebugger(ROS2Helper ros2Helper)
   {
      ros2Helper.subscribeViaCallback(ContinuousPlanningAPI.MONTE_CARLO_TREE_NODES, this::onMonteCarloTreeNodesReceived);
   }

   public void onMonteCarloTreeNodesReceived(PoseListMessage poseListMessage)
   {
      LogTools.info("Received Monte-Carlo Tree Nodes: {}", poseListMessage.getPoses().size());
      if (monteCarloTreeNodeStates == null)
      {
         monteCarloTreeNodeStates = MessageTools.unpackPoseListMessage(poseListMessage);
      }
   }

   public void render()
   {
      if (monteCarloTreeNodeStates != null)
      {
         update(monteCarloTreeNodeStates);
         monteCarloTreeNodeStates = null;
      }
   }

   public void update(List<Pose3D> poses)
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
      for (ModelInstance stateSphere : stateSpheres)
      {
         stateSphere.getRenderables(renderables, pool);
      }
   }

   public void destroy()
   {
      for (ModelInstance sphere : stateSpheres)
      {
         sphere = null;
      }
   }
}
