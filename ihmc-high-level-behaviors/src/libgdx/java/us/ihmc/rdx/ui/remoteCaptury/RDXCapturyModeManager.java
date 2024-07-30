package us.ihmc.rdx.ui.remoteCaptury;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.flag.ImGuiCol;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.motionRetargeting.DefaultRetargetingParameters;
import us.ihmc.motionRetargeting.RetargetingParameters;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2RobotVisualizer;
import us.ihmc.rdx.ui.vr.RDXVRStereoVision;
import us.ihmc.rdx.vr.RDXVRContext;
import us.ihmc.robotics.robotSide.RobotSide;

import javax.annotation.Nullable;
import java.util.Set;

public class RDXCapturyModeManager
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private RDXCapturyMode mode = RDXCapturyMode.INPUTS_DISABLED;
   private RDXCapturyKinematicsStreaming kinematicsStreamingMode;
   private RDXROS2RobotVisualizer robotVisualizer;
   private boolean wasStreamingWithStereo = false;
   private boolean wasCapturyReady = false;
   private static final String PANEL_NAME = "Captury mode controls";
   private RDXVRStereoVision stereoVision;

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
         kinematicsStreamingMode.create(baseUI.getVRManager().getContext(), createKinematicsStreamingToolboxModule);
      }
      stereoVision = new RDXVRStereoVision(syncedRobot.getReferenceFrames());
   }

   public void processVRInput(RDXVRContext vrContext)
   {
      if(kinematicsStreamingMode != null)
      {
         kinematicsStreamingMode.processCapturyInput();
      }
   }
   public void update()
   {
      RDXBaseUI baseUI = RDXBaseUI.getInstance();
      boolean isCapturyReady = baseUI.getCapturyManager().getCapturyEnabled().get();
      if(isCapturyReady && !wasCapturyReady)
      {
         baseUI.getPrimary3DPanel().addOverlayPanel(PANEL_NAME, this::renderImGuiWidgets);
      }
      else if (!isCapturyReady && wasCapturyReady)
      {
         baseUI.getPrimary3DPanel().removeOverlayPanel(PANEL_NAME);
      }
      wasCapturyReady = isCapturyReady;
      if (kinematicsStreamingMode != null)
         kinematicsStreamingMode.update(mode == RDXCapturyMode.WHOLE_BODY_IK_STREAMING);


      // fade robot graphics if in stereo vision mode
      boolean streamingWithStereo = kinematicsStreamingMode.isStreaming() && stereoVision.isEnabled();
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
      if (ImGui.radioButton(labels.get(RDXCapturyMode.INPUTS_DISABLED.getReadableName()), mode == RDXCapturyMode.INPUTS_DISABLED))
      {
         mode = RDXCapturyMode.INPUTS_DISABLED;
      }
      if (kinematicsStreamingMode == null)
      {
         ImGui.pushStyleColor(ImGuiCol.Text, ImGuiTools.DARK_RED);
      }
      if (ImGui.radioButton(labels.get(RDXCapturyMode.WHOLE_BODY_IK_STREAMING.getReadableName()), mode == RDXCapturyMode.WHOLE_BODY_IK_STREAMING))
      {
         mode = RDXCapturyMode.WHOLE_BODY_IK_STREAMING;
      }

      if(getMode() == RDXCapturyMode.WHOLE_BODY_IK_STREAMING)
      {
         ImGuiTools.separatorText(getMode().getReadableName() + " options");
         if (getKinematicsStreamingMode() != null)
         {
            getKinematicsStreamingMode().renderImGuiWidgets();
         }
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
         switch (mode)
         {
            case WHOLE_BODY_IK_STREAMING ->
            {
               if (kinematicsStreamingMode != null)
                  kinematicsStreamingMode.getVirtualRenderables(renderables, pool, sceneLevels);
            }
         }
      }
   }

   public void destroy()
   {
      if (kinematicsStreamingMode != null)
         kinematicsStreamingMode.destroy();
      stereoVision.getDualBlackflySphericalProjection().shutdown();
   }

   public RDXCapturyMode getMode()
   {
      return mode;
   }

   @Nullable
   public RDXCapturyKinematicsStreaming getKinematicsStreamingMode()
   {
      return kinematicsStreamingMode;
   }
   
}
