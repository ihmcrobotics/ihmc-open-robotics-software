package us.ihmc.gdx.ui.affordances;

import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.flag.ImGuiMouseButton;
import imgui.internal.ImGui;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.gdx.FocusBasedGDXCamera;
import us.ihmc.gdx.input.ImGui3DViewInput;
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.gdx.ui.gizmo.GDXPose3DGizmo;

public class GDXInteractableObject
{
   private GDXRobotCollisionLink collisionLink;
   private ReferenceFrame objectFrame;
   private ModelInstance highlightModelInstance;
   private boolean selected = false;
   private boolean modified = false;
   private final GDXPose3DGizmo poseGizmo = new GDXPose3DGizmo();
   private boolean mouseIntersects;
   private Runnable onSpacePressed;

   public void create(GDXRobotCollisionLink collisionLink, ReferenceFrame objectFrame, String modelFileName, FocusBasedGDXCamera camera3D)
   {
      this.collisionLink = collisionLink;
      this.objectFrame = objectFrame;
      highlightModelInstance = GDXInteractableTools.createHighlightEffectModel(modelFileName);
      poseGizmo.create(camera3D);
   }

   public void process3DViewInput(ImGui3DViewInput input)
   {
      mouseIntersects = collisionLink.getIntersects();
      boolean leftMouseReleasedWithoutDrag = input.mouseReleasedWithoutDrag(ImGuiMouseButton.Left);

      if (!modified && mouseIntersects)
      {
         GDXTools.toGDX(objectFrame.getTransformToWorldFrame(), highlightModelInstance.transform);

         if (leftMouseReleasedWithoutDrag)
         {
            selected = true;
            modified = true;
            collisionLink.overrideTransform(true);
            poseGizmo.getTransform().set(objectFrame.getTransformToWorldFrame());
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
         GDXTools.setTransparency(highlightModelInstance, 0.7f);
      }
      else
      {
         GDXTools.setTransparency(highlightModelInstance, 0.5f);
      }

      if (modified)
      {
         collisionLink.overrideTransform(true).set(poseGizmo.getTransform());
         GDXTools.toGDX(poseGizmo.getTransform(), highlightModelInstance.transform);
      }

      if (selected)
      {
         poseGizmo.process3DViewInput(input);

         if (ImGui.isKeyReleased(input.getSpaceKey()))
         {
            onSpacePressed.run();
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
         highlightModelInstance.getRenderables(renderables, pool);
      }
      if (selected)
      {
         poseGizmo.getRenderables(renderables, pool);
      }
   }

   public void destroy()
   {
      highlightModelInstance.model.dispose();
   }

   public Pose3DReadOnly getPose()
   {
      return poseGizmo.getPose();
   }

   public void setOnSpacePressed(Runnable onSpacePressed)
   {
      this.onSpacePressed = onSpacePressed;
   }
}
