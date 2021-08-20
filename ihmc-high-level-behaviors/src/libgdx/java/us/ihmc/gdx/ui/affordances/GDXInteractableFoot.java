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
import us.ihmc.robotics.robotSide.RobotSide;

public class GDXInteractableFoot
{
   private final GDXRobotCollisionLink collisionLink;
   private final RobotSide side;
   private final ReferenceFrame syncedRobotFootFrame;
   private final ROS2ControllerHelper ros2Helper;
   private final Model footModel;
   private final ModelInstance footModelInstance;
   private boolean selected = false;
   private boolean modified = false;
   private final GDXPose3DGizmo poseGizmo = new GDXPose3DGizmo();
   private boolean mouseIntersects;

   public GDXInteractableFoot(GDXRobotCollisionLink collisionLink, RobotSide side, ReferenceFrame syncedRobotFootFrame, ROS2ControllerHelper ros2Helper)
   {
      this.collisionLink = collisionLink;
      this.side = side;
      this.syncedRobotFootFrame = syncedRobotFootFrame;
      this.ros2Helper = ros2Helper;

      String robotSidePrefix = (side == RobotSide.LEFT) ? "l_" : "r_";
      String modelFileName = robotSidePrefix + "foot.g3dj";
      footModel = GDXModelLoader.loadG3DModel(modelFileName);
      footModelInstance = new ModelInstance(footModel);
      footModelInstance.transform.scale(1.01f, 1.01f, 1.01f);

      GDXTools.setTransparency(footModelInstance, 0.5f);
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
         GDXTools.toGDX(syncedRobotFootFrame.getTransformToWorldFrame(), footModelInstance.transform);

         if (leftMouseReleasedWithoutDrag)
         {
            selected = true;
            modified = true;
            collisionLink.overrideTransform(true);
            poseGizmo.getTransform().set(syncedRobotFootFrame.getTransformToWorldFrame());
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
         GDXTools.setTransparency(footModelInstance, 0.7f);
      }
      else
      {
         GDXTools.setTransparency(footModelInstance, 0.5f);
      }

      if (modified)
      {
         collisionLink.overrideTransform(true).set(poseGizmo.getTransform());
         GDXTools.toGDX(poseGizmo.getTransform(), footModelInstance.transform);
      }

      if (selected)
      {
         poseGizmo.process3DViewInput(input);

         if (ImGui.isKeyReleased(input.getSpaceKey()))
         {
            // TODO: Trajectory time in ImGui panel
            ros2Helper.publishToController(HumanoidMessageTools.createFootTrajectoryMessage(side, 1.2, poseGizmo.getPose()));
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
         footModelInstance.getRenderables(renderables, pool);
      }
      if (selected)
      {
         poseGizmo.getRenderables(renderables, pool);
      }
   }

   public void destroy()
   {
      footModel.dispose();
   }
}
