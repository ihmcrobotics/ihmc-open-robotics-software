package us.ihmc.gdx.ui.vr;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.internal.ImGui;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.gdx.GDXSingleContext3DSituatedImGuiPanel;
import us.ihmc.gdx.imgui.ImGuiPanel;
import us.ihmc.gdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.gdx.sceneManager.GDX3DScene;
import us.ihmc.gdx.vr.GDXVRContext;
import us.ihmc.gdx.vr.GDXVRTeleporter;

import java.util.Map;

public class GDXVRModeManager
{
   private final GDXVRTeleporter teleporter = new GDXVRTeleporter();
   private GDXVRHandPlacedFootstepMode handPlacedFootstepMode;
   private GDXVRKinematicsStreamingMode kinematicsStreamingMode;
   private GDXSingleContext3DSituatedImGuiPanel imGuiPanel;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private int mode = 0;
   private boolean render3DSituatedImGuiPanel;
   private ImGuiPanel panel = new ImGuiPanel("VR Manager", this::renderImGuiWidgets);

   public void create(GDXVRContext vrContext,
                      GDX3DScene scene,
                      DRCRobotModel robotModel,
                      Map<String, Double> initialConfiguration,
                      ROS2ControllerHelper controllerHelper,
                      boolean render3DSituatedImGuiPanel)
   {
      this.render3DSituatedImGuiPanel = render3DSituatedImGuiPanel;
      teleporter.create();

      handPlacedFootstepMode = new GDXVRHandPlacedFootstepMode();
      handPlacedFootstepMode.create(robotModel, controllerHelper);

      kinematicsStreamingMode = new GDXVRKinematicsStreamingMode(robotModel, initialConfiguration, controllerHelper);
      kinematicsStreamingMode.create(vrContext);

      if (render3DSituatedImGuiPanel)
      {
         imGuiPanel = new GDXSingleContext3DSituatedImGuiPanel();
         imGuiPanel.create(400, 500, this::renderImGuiWidgets, vrContext);
         imGuiPanel.updateDesiredPose(transform ->
         {
            transform.getTranslation().set(1.0f, 0.0f, 1.0f);
         });
         scene.addRenderableProvider(imGuiPanel);
      }
   }

   public void processVRInput(GDXVRContext vrContext)
   {
      if (render3DSituatedImGuiPanel)
         imGuiPanel.processVRInput(vrContext);
      teleporter.processVRInput(vrContext);
      if (mode == 0)
      {
         handPlacedFootstepMode.processVRInput(vrContext);
      }
      else if (mode == 1)
      {
         kinematicsStreamingMode.processVRInput(vrContext);
      }
   }

   public void update()
   {
      kinematicsStreamingMode.update();
      if (render3DSituatedImGuiPanel)
         imGuiPanel.render();
   }

   public void renderImGuiWidgets()
   {
      if (ImGui.radioButton(labels.get("Rough Terrain"), mode == 0))
      {
         mode = 0;
      }
      if (ImGui.radioButton(labels.get("Manipulation"), mode == 1))
      {
         mode = 1;
      }
//      if (ImGui.radioButton(labels.get("Joystick"), mode == 2))
//      {
//         mode = 2;
//      }

      kinematicsStreamingMode.renderImGuiWidgets();
      imgui.ImGui.text("VR Controller Input Mappings:");
      imgui.ImGui.text("Teleport - Right controller B button");
      imgui.ImGui.text("Adjust VR Z height - Slide up and down on the right controller touchpad");
      imgui.ImGui.text("Left/Right Triggers: Hold and release to place a footstep");
      imgui.ImGui.text("Clear footsteps - Left controller B button");
      imgui.ImGui.text("Walk - Right controller A button");
      imgui.ImGui.text("Move ImGui panel - Grip right controller and drag the panel around");
      imgui.ImGui.text("Click buttons on the ImGui panel - Right controller trigger click");
   }

   public void getVirtualRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      teleporter.getRenderables(renderables, pool);
      handPlacedFootstepMode.getRenderables(renderables, pool);
      kinematicsStreamingMode.getVirtualRenderables(renderables, pool);
      if (render3DSituatedImGuiPanel)
         imGuiPanel.getRenderables(renderables, pool);
   }

   public void destroy()
   {
      if (render3DSituatedImGuiPanel)
         imGuiPanel.dispose();
      kinematicsStreamingMode.destroy();
   }

   public GDXVRKinematicsStreamingMode getKinematicsStreamingMode()
   {
      return kinematicsStreamingMode;
   }
}
