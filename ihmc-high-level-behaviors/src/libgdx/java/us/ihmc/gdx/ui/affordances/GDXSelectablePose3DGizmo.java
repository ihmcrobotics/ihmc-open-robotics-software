package us.ihmc.gdx.ui.affordances;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.flag.ImGuiMouseButton;
import imgui.internal.ImGui;
import us.ihmc.gdx.FocusBasedGDXCamera;
import us.ihmc.gdx.input.ImGui3DViewInput;
import us.ihmc.gdx.ui.gizmo.GDXPose3DGizmo;

public class GDXSelectablePose3DGizmo
{
   private final GDXPose3DGizmo poseGizmo = new GDXPose3DGizmo();
   private boolean selected = false;
   private boolean mouseIntersects;

   public void create(FocusBasedGDXCamera camera3D)
   {
      poseGizmo.create(camera3D);
   }

   public void setMouseIntersects(boolean mouseIntersects)
   {
      this.mouseIntersects = mouseIntersects;
   }

   public void process3DViewInput(ImGui3DViewInput input)
   {
      boolean leftMouseReleasedWithoutDrag = input.mouseReleasedWithoutDrag(ImGuiMouseButton.Left);

      if (mouseIntersects)
      {
         if (leftMouseReleasedWithoutDrag)
         {
            selected = true;
         }
      }

      if (selected && !mouseIntersects && leftMouseReleasedWithoutDrag)
      {
         selected = false;
      }

      if (selected)
      {
         poseGizmo.process3DViewInput(input);
      }

      if (selected && ImGui.isKeyReleased(input.getDeleteKey()) && ImGui.isKeyReleased(input.getEscapeKey()))
      {
         selected = false;
      }
   }

   public void getVirtualRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (selected)
      {
         poseGizmo.getRenderables(renderables, pool);
      }
   }

   public GDXPose3DGizmo getPoseGizmo()
   {
      return poseGizmo;
   }
}
