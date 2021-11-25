package us.ihmc.gdx.ui.affordances;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.flag.ImGuiMouseButton;
import imgui.internal.ImGui;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.gdx.FocusBasedGDXCamera;
import us.ihmc.gdx.input.ImGui3DViewInput;
import us.ihmc.gdx.ui.gizmo.GDXPose3DGizmo;
import us.ihmc.gdx.ui.graphics.GDXReferenceFrameGraphic;

public class GDXInteractableObject
{
   private GDXRobotCollisionLink collisionLink;
   private ReferenceFrame graphicFrame;
   private ReferenceFrame collisionFrame;
   private ReferenceFrame controlFrame;
   private boolean hasMultipleFrames;
   private RigidBodyTransform controlToGraphicTransform;
   private RigidBodyTransform controlToCollisionTransform;
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();
   private final FramePose3D tempFramePose = new FramePose3D();
   private GDXInteractableHighlightModel highlightModel;
   private boolean selected = false;
   private boolean modified = false;
   private final GDXPose3DGizmo poseGizmo = new GDXPose3DGizmo();
   private boolean mouseIntersects;
   private Runnable onSpacePressed;
   private GDXReferenceFrameGraphic graphicReferenceFrameGraphic;
   private GDXReferenceFrameGraphic controlReferenceFrameGraphic;

   public void create(GDXRobotCollisionLink collisionLink, ReferenceFrame controlFrame, String graphicFileName, FocusBasedGDXCamera camera3D)
   {
      create(collisionLink, controlFrame, controlFrame, controlFrame, graphicFileName, camera3D);
   }

   public void create(GDXRobotCollisionLink collisionLink,
                      ReferenceFrame graphicFrame,
                      ReferenceFrame collisionFrame,
                      ReferenceFrame controlFrame,
                      String modelFileName,
                      FocusBasedGDXCamera camera3D)
   {
      this.collisionLink = collisionLink;
      this.graphicFrame = graphicFrame;
      this.collisionFrame = collisionFrame;
      this.controlFrame = controlFrame;
      hasMultipleFrames = !(graphicFrame == collisionFrame && collisionFrame == controlFrame);
      highlightModel = new GDXInteractableHighlightModel(modelFileName);
      poseGizmo.create(camera3D);
      graphicReferenceFrameGraphic = new GDXReferenceFrameGraphic();
      controlReferenceFrameGraphic = new GDXReferenceFrameGraphic();
   }

   public void process3DViewInput(ImGui3DViewInput input)
   {
      mouseIntersects = collisionLink.getIntersects();
      boolean leftMouseReleasedWithoutDrag = input.mouseReleasedWithoutDrag(ImGuiMouseButton.Left);

      if (!modified && mouseIntersects)
      {
         if (hasMultipleFrames && controlToGraphicTransform == null) // we just need to do this once
         {
            controlToGraphicTransform = new RigidBodyTransform();
            tempFramePose.setToZero(graphicFrame);
            tempFramePose.changeFrame(controlFrame);
            tempFramePose.get(controlToGraphicTransform);
            controlToCollisionTransform = new RigidBodyTransform();
            tempFramePose.setToZero(collisionFrame);
            tempFramePose.changeFrame(controlFrame);
            tempFramePose.get(controlToCollisionTransform);
         }

         if (hasMultipleFrames)
         {
            highlightModel.setPose(controlFrame.getTransformToWorldFrame(), controlToGraphicTransform);
         }
         else
         {
            highlightModel.setPose(controlFrame.getTransformToWorldFrame());
         }

         if (leftMouseReleasedWithoutDrag)
         {
            selected = true;
            modified = true;
            collisionLink.setOverrideTransform(true);
            poseGizmo.getTransform().set(controlFrame.getTransformToWorldFrame());
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
         highlightModel.setTransparency(0.7);
      }
      else
      {
         highlightModel.setTransparency(0.5);
      }

      if (modified)
      {
         if (hasMultipleFrames)
         {
            tempTransform.set(controlToCollisionTransform);
            poseGizmo.getTransform().transform(tempTransform);
            collisionLink.setOverrideTransform(true).set(tempTransform);
            highlightModel.setPose(poseGizmo.getTransform(), controlToGraphicTransform);
         }
         else
         {
            collisionLink.setOverrideTransform(true).set(poseGizmo.getTransform());
            highlightModel.setPose(poseGizmo.getTransform());
         }
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
         collisionLink.setOverrideTransform(false);
      }

      if (selected && ImGui.isKeyReleased(input.getEscapeKey()))
      {
         selected = false;
      }
   }

   public void getVirtualRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
//      if (hasMultipleFrames)
//      {
//         graphicReferenceFrameGraphic.setToReferenceFrame(graphicFrame);
//         graphicReferenceFrameGraphic.getRenderables(renderables, pool);
//      }
//      controlReferenceFrameGraphic.setToReferenceFrame(controlFrame);
//      controlReferenceFrameGraphic.getRenderables(renderables, pool);

      if (modified || mouseIntersects)
      {
         highlightModel.getRenderables(renderables, pool);
      }
      if (selected)
      {
         poseGizmo.getRenderables(renderables, pool);
      }
   }

   public void destroy()
   {
      highlightModel.dispose();
      if (hasMultipleFrames)
      {
         graphicReferenceFrameGraphic.dispose();
      }
      controlReferenceFrameGraphic.dispose();
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
