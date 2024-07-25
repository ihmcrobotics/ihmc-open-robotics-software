package us.ihmc.rdx.ui.remoteCaptury;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.flag.ImGuiCol;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.motionRetargeting.DefaultRetargetingParameters;
import us.ihmc.motionRetargeting.RetargetingParameters;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.imgui.RDX3DSituatedImGuiPanel;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.RDXJoystickBasedStepping;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2RobotVisualizer;
import us.ihmc.rdx.ui.vr.RDXVRHandPlacedFootstepMode;
import us.ihmc.rdx.ui.vr.RDXVRKinematicsStreamingMode;
import us.ihmc.rdx.ui.vr.RDXVRMode;
import us.ihmc.rdx.ui.vr.RDXVRModeControls;
import us.ihmc.rdx.ui.vr.RDXVRStereoVision;
import us.ihmc.rdx.vr.RDXVRContext;
import us.ihmc.robotics.robotSide.RobotSide;

import javax.annotation.Nullable;
import java.util.Set;
import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.flag.ImGuiCol;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.motionRetargeting.DefaultRetargetingParameters;
import us.ihmc.motionRetargeting.RetargetingParameters;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.imgui.RDX3DSituatedImGuiPanel;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.RDXJoystickBasedStepping;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2RobotVisualizer;
import us.ihmc.rdx.vr.RDXVRContext;
import us.ihmc.robotics.robotSide.RobotSide;

import javax.annotation.Nullable;
import java.util.Set;


public class RDXCapturyManager
{
      private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
      @Nullable
      private RDXCapturyKinematicsStreaming kinematicsStreamingMode;
      private RDXROS2RobotVisualizer robotVisualizer;
      private boolean wasStreamingWithStereo = false;
      private RDXVRMode mode = RDXVRMode.INPUTS_DISABLED;

      public void create(RDXBaseUI baseUI,
                         ROS2SyncedRobotModel syncedRobot,
                         RDXROS2RobotVisualizer robotVisualizer,
                         ROS2ControllerHelper controllerHelper,
                         boolean createKinematicsStreamingToolboxModule)
      {
         create(baseUI,
                syncedRobot,
                robotVisualizer,
                controllerHelper,
                new DefaultRetargetingParameters(),
                new SceneGraph(),
                createKinematicsStreamingToolboxModule);
      }

      public void create(RDXBaseUI baseUI,
                         ROS2SyncedRobotModel syncedRobot,
                         RDXROS2RobotVisualizer robotVisualizer,
                         ROS2ControllerHelper controllerHelper,
                         RetargetingParameters retargetingParameters,
                         boolean createKinematicsStreamingToolboxModule)
      {
         create(baseUI, syncedRobot, robotVisualizer, controllerHelper, retargetingParameters, new SceneGraph(), createKinematicsStreamingToolboxModule);
      }

      public void create(RDXBaseUI baseUI,
                         ROS2SyncedRobotModel syncedRobot,
                         RDXROS2RobotVisualizer robotVisualizer,
                         ROS2ControllerHelper controllerHelper,
                         RetargetingParameters retargetingParameters,
                         SceneGraph sceneGraph,
                         boolean createKinematicsStreamingToolboxModule)
      {
         this.robotVisualizer = robotVisualizer;

         if (syncedRobot.getRobotModel().getRobotVersion().hasArm(RobotSide.LEFT) || syncedRobot.getRobotModel().getRobotVersion().hasArm(RobotSide.RIGHT))
         {
            kinematicsStreamingMode = new RDXCapturyKinematicsStreaming(syncedRobot, controllerHelper, retargetingParameters, sceneGraph);
            kinematicsStreamingMode.create(createKinematicsStreamingToolboxModule);
         }
      }

      public void update()
      {
         kinematicsStreamingMode.update();

         // fade robot graphics if in stereo vision mode
         boolean streamingWithStereo = kinematicsStreamingMode.isStreaming();
         boolean changed = streamingWithStereo != wasStreamingWithStereo;
         wasStreamingWithStereo = streamingWithStereo;
         if (changed)
         {
            if (streamingWithStereo)
            {
               kinematicsStreamingMode.visualizeIKPreviewGraphic(false);
               robotVisualizer.fadeVisuals(0.0f, 0.01f);
            }
            else
            {
               kinematicsStreamingMode.visualizeIKPreviewGraphic(true);
               robotVisualizer.fadeVisuals(1.0f, 0.01f);
            }
         }
      }

      public void renderImGuiWidgets()
      {
         if (kinematicsStreamingMode == null)
         {
            ImGui.pushStyleColor(ImGuiCol.Text, ImGuiTools.DARK_RED);
         }
         if (ImGui.radioButton(labels.get("Whole Body IK Streaming"), mode == RDXVRMode.WHOLE_BODY_IK_STREAMING))
         {
            mode = RDXVRMode.WHOLE_BODY_IK_STREAMING;
         }
         if (kinematicsStreamingMode == null)
         {
            ImGui.popStyleColor();
         }
      }

      public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
      {
         if (sceneLevels.contains(RDXSceneLevel.VIRTUAL))
         {
            if (kinematicsStreamingMode != null)
               kinematicsStreamingMode.getVirtualRenderables(renderables, pool, sceneLevels);
         }
      }

      public void destroy()
      {
         kinematicsStreamingMode.destroy();
      }



      @Nullable
      public RDXCapturyKinematicsStreaming getKinematicsStreamingMode()
      {
         return kinematicsStreamingMode;
      }
}