package us.ihmc.rdx.ui.gizmo;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.flag.ImGuiMouseButton;
import imgui.internal.ImGui;
import imgui.type.ImBoolean;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.RDX3DPanel;

/**
 * Adds "selectedness" to a pose 3D gizmo. It's not included in the base class because
 * there's a few different ways to do it.
 */
public class RDXSelectablePose3DGizmo
{
   private final RDXPose3DGizmo poseGizmo;
   private final ImBoolean selected;

   public RDXSelectablePose3DGizmo()
   {
      this.selected = new ImBoolean(false);
      poseGizmo = new RDXPose3DGizmo();
   }

   public RDXSelectablePose3DGizmo(ReferenceFrame parentReferenceFrame)
   {
      this.selected = new ImBoolean(false);
      poseGizmo = new RDXPose3DGizmo(parentReferenceFrame);
   }

   public RDXSelectablePose3DGizmo(ReferenceFrame gizmoFrame, RigidBodyTransform gizmoTransformToParentFrameToModify)
   {
      this(gizmoFrame, gizmoTransformToParentFrameToModify, new ImBoolean(false));
   }

   public RDXSelectablePose3DGizmo(ReferenceFrame gizmoFrame, RigidBodyTransform gizmoTransformToParentFrameToModify, ImBoolean selected)
   {
      this.selected = selected;
      poseGizmo = new RDXPose3DGizmo(gizmoFrame, gizmoTransformToParentFrameToModify);
   }

   public void create(RDX3DPanel panel3D)
   {
      poseGizmo.create(panel3D);
   }

   public void createAndSetupDefault(RDX3DPanel panel3D)
   {
      create(panel3D);
      panel3D.addImGui3DViewPickCalculator(this, this::calculate3DViewPick);
      panel3D.addImGui3DViewInputProcessor(this, this::process3DViewInput);
      panel3D.getScene().addRenderableProvider(this, this::getVirtualRenderables, RDXSceneLevel.VIRTUAL);
   }

   public void removeRenderables(RDX3DPanel panel3D)
   {
      panel3D.getScene().removeRenderable(this);
      panel3D.removeImGui3DViewPickCalculator(this);
      panel3D.removeImGui3DViewInputProcessor(this);
   }

   public void calculate3DViewPick(ImGui3DViewInput input)
   {
      if (input.isWindowHovered() && selected.get())
      {
         poseGizmo.calculate3DViewPick(input);
      }
   }

   public void process3DViewInput(ImGui3DViewInput input)
   {
      process3DViewInput(input, selected.get());
   }

   public void process3DViewInput(ImGui3DViewInput input, boolean isPickSelected)
   {
      if (input.isWindowHovered())
      {
         // Process input
         boolean leftMouseReleasedWithoutDrag = input.mouseReleasedWithoutDrag(ImGuiMouseButton.Left);
         boolean isClickedOn = isPickSelected && leftMouseReleasedWithoutDrag;
         boolean somethingElseIsClickedOn = !isPickSelected && leftMouseReleasedWithoutDrag;
         boolean deselectionKeyPressed = ImGui.isKeyReleased(ImGuiTools.getDeleteKey()) || ImGui.isKeyReleased(ImGuiTools.getEscapeKey());

         // Determine selectedness
         if (isClickedOn)
         {
            selected.set(true);
         }
         if (somethingElseIsClickedOn || deselectionKeyPressed)
         {
            selected.set(false);
         }
      }

      // Act
      if (selected.get())
      {
         poseGizmo.process3DViewInput(input);
      }
      else
      {
         poseGizmo.update();
      }
   }

   public void getVirtualRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (selected.get())
      {
         poseGizmo.getRenderables(renderables, pool);
      }
   }

   public RDXPose3DGizmo getPoseGizmo()
   {
      return poseGizmo;
   }

   public boolean isSelected()
   {
      return selected.get();
   }

   public ImBoolean getSelected()
   {
      return selected;
   }

   public void setSelected(boolean selected)
   {
      this.selected.set(selected);
   }
}