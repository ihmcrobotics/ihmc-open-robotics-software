package us.ihmc.gdx.ui.affordances;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.flag.ImGuiMouseButton;
import imgui.internal.ImGui;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.gdx.GDXFocusBasedCamera;
import us.ihmc.gdx.imgui.ImGuiTools;
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

   public void create(GDXFocusBasedCamera camera3D)
   {
      poseGizmo.create(camera3D);
   }

   public void calculate3DViewPick(ImGui3DViewInput input)
   {
      if (selected)
      {
         poseGizmo.calculate3DViewPick(input);
      }
   }

   public void process3DViewInput(ImGui3DViewInput input, boolean isPickSelected)
   {
      // Process input
      boolean leftMouseReleasedWithoutDrag = input.mouseReleasedWithoutDrag(ImGuiMouseButton.Left);
      boolean isClickedOn = isPickSelected && leftMouseReleasedWithoutDrag;
      boolean somethingElseIsClickedOn = !isPickSelected && leftMouseReleasedWithoutDrag;
      boolean deselectionKeyPressed = ImGui.isKeyReleased(ImGuiTools.getDeleteKey()) || ImGui.isKeyReleased(ImGuiTools.getEscapeKey());

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
