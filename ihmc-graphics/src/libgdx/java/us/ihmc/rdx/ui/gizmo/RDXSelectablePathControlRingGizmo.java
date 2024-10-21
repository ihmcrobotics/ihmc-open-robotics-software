package us.ihmc.rdx.ui.gizmo;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.flag.ImGuiMouseButton;
import imgui.ImGui;
import imgui.type.ImBoolean;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.vr.RDXVRContext;
import us.ihmc.robotics.robotSide.RobotSide;

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
   private final ImBoolean selected;

   public RDXSelectablePathControlRingGizmo()
   {
      this.selected = new ImBoolean(false);
      pathControlRingGizmo = new RDXPathControlRingGizmo();
   }

   public RDXSelectablePathControlRingGizmo(ReferenceFrame parentReferenceFrame)
   {
      this.selected = new ImBoolean(false);
      pathControlRingGizmo = new RDXPathControlRingGizmo(parentReferenceFrame);
   }

   public RDXSelectablePathControlRingGizmo(ReferenceFrame gizmoFrame, RigidBodyTransform gizmoTransformToParentFrameToModify)
   {
      this(gizmoFrame, gizmoTransformToParentFrameToModify, new ImBoolean(false));
   }

   public RDXSelectablePathControlRingGizmo(ReferenceFrame gizmoFrame, RigidBodyTransform gizmoTransformToParentFrameToModify, ImBoolean selected)
   {
      this.selected = selected;
      pathControlRingGizmo = new RDXPathControlRingGizmo(gizmoFrame, gizmoTransformToParentFrameToModify);
   }

   public void create(RDX3DPanel panel3D)
   {
      pathControlRingGizmo.create(panel3D);
   }

   public void createAndSetupDefault(RDXBaseUI baseUI)
   {
      create(baseUI.getPrimary3DPanel());
      baseUI.getVRManager().getContext().addVRPickCalculator(this::calculateVRPick);
      baseUI.getVRManager().getContext().addVRPickCalculator(this::processVRInput);
      baseUI.getPrimary3DPanel().addImGui3DViewPickCalculator(this::calculate3DViewPick);
      baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(this::process3DViewInput);
      baseUI.getPrimary3DPanel().getScene().addRenderableProvider(this::getVirtualRenderables, RDXSceneLevel.VIRTUAL);
   }

   public void calculateVRPick(RDXVRContext vrContext)
   {
      pathControlRingGizmo.calculateVRPick(vrContext);
   }

   public void processVRInput(RDXVRContext vrContext)
   {
      pathControlRingGizmo.processVRInput(vrContext);
      if (selectable && (pathControlRingGizmo.getIsRingBeingDraggedVR().get(RobotSide.LEFT) ||
                         pathControlRingGizmo.getIsRingBeingDraggedVR().get(RobotSide.RIGHT)))
      {
         selected.set(true);
      }

      pathControlRingGizmo.setShowArrows(selected.get());
      pathControlRingGizmo.setHighlightingEnabled(modified);
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
         // To select, click on the ring. The arrows are not showing up at this time.
         boolean isClickedOn = pathControlRingGizmo.getRingHovered() && leftMouseReleasedWithoutDrag;
         // To deselect, click on anything other than the ring and the arrows
         boolean somethingElseIsClickedOn = !pathControlRingGizmo.getAnyPartHovered() && leftMouseReleasedWithoutDrag;
         boolean isCtrlPressed = ImGui.getIO().getKeyCtrl();

         // Determine selectedness
         if (selectable && isClickedOn)
         {
            selected.set(true);
         }
         // Allow mutli-selection by holding Ctrl
         if (somethingElseIsClickedOn && !isCtrlPressed)
         {
            selected.set(false);
         }
      }

      pathControlRingGizmo.setShowArrows(selected.get());
      pathControlRingGizmo.setHighlightingEnabled(modified);

      // Act
      if (selected.get())
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
      if ((selectable && (pathControlRingGizmo.getRingHovered()) ||
           pathControlRingGizmo.getIsGizmoHoveredVR().get(RobotSide.RIGHT) ||
           pathControlRingGizmo.getIsGizmoHoveredVR().get(RobotSide.LEFT)) || modified || selected.get())
      {
         pathControlRingGizmo.getRenderables(renderables, pool);
      }
   }

   public RDXPathControlRingGizmo getPathControlRingGizmo()
   {
      return pathControlRingGizmo;
   }

   public boolean getSelected()
   {
      return selected.get();
   }

   public ImBoolean getImSelected()
   {
      return selected;
   }

   public void setSelected(boolean selected)
   {
      this.selected.set(selected);
   }

   /**
    * Prevent the ring from appearing when hovering over it if it is not
    * already selected.
    */
   public void setSelectable(boolean selectable)
   {
      this.selectable = selectable;
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
