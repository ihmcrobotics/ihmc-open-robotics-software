package us.ihmc.gdx.ui.affordances;

import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.flag.ImGuiMouseButton;
import imgui.internal.ImGui;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.gdx.FocusBasedGDXCamera;
import us.ihmc.gdx.input.ImGui3DViewInput;
import us.ihmc.gdx.tools.GDXModelLoader;
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.gdx.ui.gizmo.GDXPose3DGizmo;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;

public class GDXInteractablePelvis
{
   private final GDXRobotCollisionLink collisionLink;
   private final ReferenceFrame syncedRobotPelvisFrame;
   private final ROS2ControllerHelper ros2Helper;
   private final Model pelvisModel;
   private final ModelInstance pelvisModelInstance;
   private boolean selected = false;
   private boolean modified = false;
   private final GDXPose3DGizmo poseGizmo = new GDXPose3DGizmo();
   private boolean mouseIntersects;

   public GDXInteractablePelvis(GDXRobotCollisionLink collisionLink, ReferenceFrame syncedRobotPelvisFrame, ROS2ControllerHelper ros2Helper)
   {
      this.collisionLink = collisionLink;
      this.syncedRobotPelvisFrame = syncedRobotPelvisFrame;
      this.ros2Helper = ros2Helper;

      pelvisModel = GDXModelLoader.loadG3DModel("pelvis.g3dj");
      pelvisModelInstance = new ModelInstance(pelvisModel);
      pelvisModelInstance.transform.scale(1.01f, 1.01f, 1.01f);

      GDXTools.setTransparency(pelvisModelInstance, 0.5f);
   }

   public void create(FocusBasedGDXCamera camera3D)
   {
      poseGizmo.create(camera3D);
   }

   public void process3DViewInput(ImGui3DViewInput input)
   {
      mouseIntersects = collisionLink.getIntersects();
      boolean leftMouseReleasedWithoutDrag = input.mouseReleasedWithoutDrag(ImGuiMouseButton.Left);

      if (!modified && mouseIntersects)
      {
         GDXTools.toGDX(syncedRobotPelvisFrame.getTransformToWorldFrame(), pelvisModelInstance.transform);

         if (leftMouseReleasedWithoutDrag)
         {
            selected = true;
            modified = true;
            collisionLink.overrideTransform(true);
            poseGizmo.getTransform().set(syncedRobotPelvisFrame.getTransformToWorldFrame());
         }
      }

      if (selected && !mouseIntersects && leftMouseReleasedWithoutDrag)
      {
         selected = false;
      }
      if (modified && mouseIntersects && leftMouseReleasedWithoutDrag)
      {
         selected = true;
      }
      if (modified && !selected && mouseIntersects)
      {
         GDXTools.setTransparency(pelvisModelInstance, 0.7f);
      }
      else
      {
         GDXTools.setTransparency(pelvisModelInstance, 0.5f);
      }

      if (modified)
      {
         collisionLink.overrideTransform(true).set(poseGizmo.getTransform());
         GDXTools.toGDX(poseGizmo.getTransform(), pelvisModelInstance.transform);
      }

      if (selected)
      {
         poseGizmo.process3DViewInput(input);

         if (ImGui.isKeyReleased(input.getSpaceKey()))
         {
            // TODO: Trajectory time in ImGui panel
            ros2Helper.publishToController(HumanoidMessageTools.createPelvisTrajectoryMessage(1.2, poseGizmo.getPose()));
         }
      }

      if (modified && selected && ImGui.isKeyReleased(input.getDeleteKey()))
      {
         selected = false;
         modified = false;
         collisionLink.overrideTransform(false);
      }

      if (selected && ImGui.isKeyReleased(input.getEscapeKey()))
      {
         selected = false;
      }
   }

   public void getVirtualRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (modified || mouseIntersects)
      {
         pelvisModelInstance.getRenderables(renderables, pool);
      }
      if (selected)
      {
         poseGizmo.getRenderables(renderables, pool);
      }
   }

   public void destroy()
   {
      pelvisModel.dispose();
   }
}
