package us.ihmc.rdx.ui.gizmo;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.flag.ImGuiMouseButton;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.RDX3DPanel;

/**
 * Adds "selectedness" to a path control ring gizmo.
 * It's not included in the base class because there's a few different ways to do it.
 *
 * This control ring is selected by the user hovering over the ring
 * and clicking on it to select it.
 *
 * The ring will start to show when the user hovers over it.
 */
public class RDXSelectablePathControlRingGizmo
{
   private final RDXPathControlRingGizmo pathControlRingGizmo;
   private boolean selectable = true;
   private boolean modified = false;
   private boolean focused = false;

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
      if (input.isWindowHovered())
      {
         pathControlRingGizmo.calculate3DViewPick(input);
      }
   }

   public void process3DViewInput(ImGui3DViewInput input)
   {
      pathControlRingGizmo.calculateHovered(input);

      if (input.isWindowHovered())
      {
         boolean leftMouseReleasedWithoutDrag = input.mouseReleasedWithoutDrag(ImGuiMouseButton.Left);
         boolean isClickedOn = isRingHovered() && leftMouseReleasedWithoutDrag;
         boolean somethingElseIsClickedOn = !isRingHovered() && leftMouseReleasedWithoutDrag;

         // Determine selectedness
         if (selectable && isClickedOn)
         {
            focused = true;
         }
         if (somethingElseIsClickedOn)
         {
            focused = false;
         }
      }

      pathControlRingGizmo.setShowArrows(focused);
      pathControlRingGizmo.setHighlightingEnabled(modified);

      // Act
      if (focused)
      {
         pathControlRingGizmo.process3DViewInputModification(input);
      }
      else if (selectable)
      {
         pathControlRingGizmo.update();
      }
   }

   public void getVirtualRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if ((selectable && isRingHovered()) || modified || focused)
      {
         pathControlRingGizmo.getRenderables(renderables, pool);
      }
   }

   public RDXPathControlRingGizmo getPathControlRingGizmo()
   {
      return pathControlRingGizmo;
   }

   public boolean getFocused()
   {
      return focused;
   }

   public void setFocused(boolean focused)
   {
      this.focused = focused;
   }

   /**
    * Prevent the ring from appearing when hovering over it if it is not
    * already selected.
    */
   public void setSelectable(boolean selectable)
   {
      this.selectable = selectable;
   }

   private boolean isRingHovered()
   {
      return pathControlRingGizmo.getRingHovered();
   }

   public boolean getModified()
   {
      return modified;
   }

   public void setModified(boolean modified)
   {
      this.modified = modified;
   }
}
