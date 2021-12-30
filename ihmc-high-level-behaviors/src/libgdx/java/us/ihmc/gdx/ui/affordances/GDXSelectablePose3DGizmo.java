package us.ihmc.gdx.ui.affordances;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.flag.ImGuiMouseButton;
import imgui.internal.ImGui;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.gdx.FocusBasedGDXCamera;
import us.ihmc.gdx.input.ImGui3DViewInput;
import us.ihmc.gdx.ui.gizmo.GDXPose3DGizmo;

public class GDXSelectablePose3DGizmo
{
   private final GDXPose3DGizmo poseGizmo;
   private boolean selected = false;

   public GDXSelectablePose3DGizmo()
   {
      poseGizmo = new GDXPose3DGizmo();
   }

   public GDXSelectablePose3DGizmo(ReferenceFrame parentReferenceFrame)
   {
      poseGizmo = new GDXPose3DGizmo(parentReferenceFrame);
   }

   public GDXSelectablePose3DGizmo(ReferenceFrame gizmoFrame, RigidBodyTransform gizmoTransformToParentFrameToModify)
   {
      poseGizmo = new GDXPose3DGizmo(gizmoFrame, gizmoTransformToParentFrameToModify);
   }

   public void create(FocusBasedGDXCamera camera3D)
   {
      poseGizmo.create(camera3D);
   }

   public void process3DViewInput(ImGui3DViewInput input, boolean mouseIntersects)
   {
      // Process input
      boolean leftMouseReleasedWithoutDrag = input.mouseReleasedWithoutDrag(ImGuiMouseButton.Left);
      boolean isClickedOn = mouseIntersects && leftMouseReleasedWithoutDrag;
      boolean somethingElseIsClickedOn = !mouseIntersects && leftMouseReleasedWithoutDrag;
      boolean deselectionKeyPressed = ImGui.isKeyReleased(input.getDeleteKey()) || ImGui.isKeyReleased(input.getEscapeKey());

      // Determine selectedness
      if (isClickedOn)
      {
         selected = true;
      }
      if (somethingElseIsClickedOn || deselectionKeyPressed)
      {
         selected = false;
      }

      // Act
      if (selected)
      {
         poseGizmo.process3DViewInput(input);
      }
      else
      {
         poseGizmo.updateTransforms();
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

   public boolean isSelected()
   {
      return selected;
   }
}
