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
public class RDXSelectablePose3DGizmo extends RDXPose3DGizmo
{
   private final ImBoolean selected;

   /** See {@link RDXPose3DGizmo#RDXPose3DGizmo()} */
   public RDXSelectablePose3DGizmo()
   {
      super();
      this.selected = new ImBoolean(false);
   }

   /** See {@link RDXPose3DGizmo#RDXPose3DGizmo(ReferenceFrame)} */
   public RDXSelectablePose3DGizmo(ReferenceFrame parentReferenceFrame)
   {
      super(parentReferenceFrame);
      this.selected = new ImBoolean(false);
   }

   /** See {@link RDXPose3DGizmo#RDXPose3DGizmo(ReferenceFrame, RigidBodyTransform)} */
   public RDXSelectablePose3DGizmo(ReferenceFrame gizmoFrame, RigidBodyTransform gizmoTransformToParentFrameToModify)
   {
      this(gizmoFrame, gizmoTransformToParentFrameToModify, new ImBoolean(false));
   }

   /** See {@link RDXPose3DGizmo#RDXPose3DGizmo(ReferenceFrame, RigidBodyTransform)} */
   public RDXSelectablePose3DGizmo(ReferenceFrame gizmoFrame, RigidBodyTransform gizmoTransformToParentFrameToModify, ImBoolean selected)
   {
      super(gizmoFrame, gizmoTransformToParentFrameToModify);
      this.selected = selected;
   }

   /** See {@link RDXPose3DGizmo#RDXPose3DGizmo(RigidBodyTransform, ReferenceFrame)} */
   public RDXSelectablePose3DGizmo(RigidBodyTransform gizmoTransformToParentFrameToModify, ReferenceFrame parentReferenceFrame)
   {
      this(gizmoTransformToParentFrameToModify, parentReferenceFrame, new ImBoolean(false));
   }

   /** See {@link RDXPose3DGizmo#RDXPose3DGizmo(RigidBodyTransform, ReferenceFrame)} */
   public RDXSelectablePose3DGizmo(RigidBodyTransform gizmoTransformToParentFrameToModify, ReferenceFrame parentReferenceFrame, ImBoolean selected)
   {
      super(gizmoTransformToParentFrameToModify, parentReferenceFrame);
      this.selected = selected;
   }

   public void createAndSetupDefault(RDX3DPanel panel3D)
   {
      super.create(panel3D);
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
         super.calculate3DViewPick(input);
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
         boolean isCtrlPressed = ImGui.getIO().getKeyCtrl();
         boolean deselectionKeyPressed = ImGui.isKeyReleased(ImGuiTools.getDeleteKey()) || ImGui.isKeyReleased(ImGuiTools.getEscapeKey());

         // Determine selectedness
         if (isClickedOn)
         {
            selected.set(true);
         }
         // Allow mutli-selection by holding Ctrl
         if ((somethingElseIsClickedOn && !isCtrlPressed) || deselectionKeyPressed)
         {
            selected.set(false);
         }
      }

      // Act
      if (selected.get())
      {
         super.process3DViewInput(input);
      }
      else
      {
         super.update();
      }
   }

   public void getVirtualRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (selected.get())
      {
         super.getRenderables(renderables, pool);
      }
   }

   // TODO: (AM) this does not need a getter, only here for backward compatibility reasons
   public RDXPose3DGizmo getPoseGizmo()
   {
      return this;
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