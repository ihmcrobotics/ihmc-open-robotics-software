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
 * Adds "selectedness" to a path control ring gizmo. It's not included in the base class because
 * there's a few different ways to do it.
 */
public class RDXSelectablePathControlRingGizmo
{
   private final RDXPathControlRingGizmo pathControlRingGizmo;
   private final ImBoolean selected = new ImBoolean(false);

   public RDXSelectablePathControlRingGizmo()
   {
      pathControlRingGizmo = new RDXPathControlRingGizmo();
   }

   public RDXSelectablePathControlRingGizmo(ReferenceFrame parentReferenceFrame)
   {
      pathControlRingGizmo = new RDXPathControlRingGizmo(parentReferenceFrame);
   }

   public RDXSelectablePathControlRingGizmo(ReferenceFrame gizmoFrame, RigidBodyTransform gizmoTransformToParentFrameToModify)
   {
      pathControlRingGizmo = new RDXPathControlRingGizmo(gizmoFrame, gizmoTransformToParentFrameToModify);
   }

   public void create(RDX3DPanel panel3D)
   {
      pathControlRingGizmo.create(panel3D);
   }

   public void createAndSetupDefault(RDX3DPanel panel3D)
   {
      create(panel3D);
      panel3D.addImGui3DViewPickCalculator(this::calculate3DViewPick);
      panel3D.addImGui3DViewInputProcessor(this::process3DViewInput);
      panel3D.getScene().addRenderableProvider(this::getVirtualRenderables, RDXSceneLevel.VIRTUAL);
   }

   public void calculate3DViewPick(ImGui3DViewInput input)
   {
      if (input.isWindowHovered() && selected.get())
      {
         pathControlRingGizmo.calculate3DViewPick(input);
      }
   }

   public void process3DViewInput(ImGui3DViewInput input)
   {
      process3DViewInput(input, selected.get());
   }

   public void process3DViewInput(ImGui3DViewInput input, boolean isHovered)
   {
      if (input.isWindowHovered())
      {
         // Process input
         boolean leftMouseReleasedWithoutDrag = input.mouseReleasedWithoutDrag(ImGuiMouseButton.Left);
         boolean isClickedOn = isHovered && leftMouseReleasedWithoutDrag;
         boolean somethingElseIsClickedOn = !isHovered && leftMouseReleasedWithoutDrag;
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

         pathControlRingGizmo.setShowArrows(selected.get());
      }

      // Act
      if (selected.get())
      {
         pathControlRingGizmo.process3DViewInput(input);
      }
      else
      {
         pathControlRingGizmo.update();
      }
   }

   public void getVirtualRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (selected.get())
      {
         pathControlRingGizmo.getRenderables(renderables, pool);
      }
   }

   public RDXPathControlRingGizmo getPathControlRingGizmo()
   {
      return pathControlRingGizmo;
   }

   public boolean isSelected()
   {
      return selected.get();
   }

   public ImBoolean getSelected()
   {
      return selected;
   }
}
