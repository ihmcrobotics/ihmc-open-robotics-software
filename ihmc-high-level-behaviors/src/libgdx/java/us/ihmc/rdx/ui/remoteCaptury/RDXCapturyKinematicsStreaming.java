package us.ihmc.rdx.ui.remoteCaptury;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.motionRetargeting.RetargetingParameters;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.vr.RDXVRContext;

import java.util.Set;

public class RDXCapturyKinematicsStreaming
{
   private final ROS2SyncedRobotModel syncedRobot;
   private final DRCRobotModel robotModel;
   private final RetargetingParameters retargetingParameters;
   private final SceneGraph sceneGraph;
   private ROS2ControllerHelper ros2ControllerHelper;

   public RDXCapturyKinematicsStreaming(ROS2SyncedRobotModel syncedRobot, ROS2ControllerHelper controllerHelper, RetargetingParameters retargetingParameters, SceneGraph sceneGraph)
   {
      this.syncedRobot = syncedRobot;
      this.robotModel = syncedRobot.getRobotModel();
      this.ros2ControllerHelper = ros2ControllerHelper;
      this.retargetingParameters = retargetingParameters;
      this.sceneGraph = sceneGraph;
   }

   public void getVirtualRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {

   }

   public void destroy()
   {
   }

   public void create(boolean createKinematicsStreamingToolboxModule)
   {
   }

   public void update()
   {
   }

   public boolean isStreaming()
   {
      return true;
   }

   public void visualizeIKPreviewGraphic(boolean b)
   {
   }
}
