package us.ihmc.gdx.ui.affordances;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.flag.ImGuiMouseButton;
import imgui.internal.ImGui;
import imgui.type.ImBoolean;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.gdx.imgui.ImGuiTools;
import us.ihmc.gdx.input.ImGui3DViewInput;
import us.ihmc.gdx.sceneManager.GDXSceneLevel;
import us.ihmc.gdx.ui.GDX3DPanel;
import us.ihmc.gdx.ui.gizmo.GDXPose3DGizmo;

/**
 * Adds "selectedness" to a pose 3D gizmo. It's not included in the base class because
 * there's a few different ways to do it.
 */
public class GDXSelectablePose3DGizmo
{
   private final GDXPose3DGizmo poseGizmo;
   private final ImBoolean selected = new ImBoolean(false);

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

   public void create(GDX3DPanel panel3D)
   {
      poseGizmo.create(panel3D);
   }

   public void createAndSetupDefault(GDX3DPanel panel3D)
   {
      create(panel3D);
      panel3D.addImGui3DViewPickCalculator(poseGizmo::calculate3DViewPick);
      panel3D.addImGui3DViewInputProcessor(poseGizmo::process3DViewInput);
      panel3D.getScene().addRenderableProvider(this::getVirtualRenderables, GDXSceneLevel.VIRTUAL);
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
         poseGizmo.updateTransforms();
      }
   }

   public void getVirtualRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (selected.get())
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
      return selected.get();
   }

   public ImBoolean getSelected()
   {
      return selected;
   }
}
