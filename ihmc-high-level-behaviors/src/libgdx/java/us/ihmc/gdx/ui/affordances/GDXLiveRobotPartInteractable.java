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
import us.ihmc.gdx.GDXFocusBasedCamera;
import us.ihmc.gdx.imgui.ImGuiTools;
import us.ihmc.gdx.input.ImGui3DViewInput;
import us.ihmc.gdx.ui.graphics.GDXReferenceFrameGraphic;

public class GDXLiveRobotPartInteractable
{
   private static final boolean SHOW_DEBUG_FRAMES = false;
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
   private boolean modified = false;
   private final GDXSelectablePose3DGizmo selectablePose3DGizmo = new GDXSelectablePose3DGizmo();
   private Runnable onSpacePressed;
   private GDXReferenceFrameGraphic graphicReferenceFrameGraphic;
   private GDXReferenceFrameGraphic controlReferenceFrameGraphic;
   private boolean pickSelected;

   public void create(GDXRobotCollisionLink collisionLink, ReferenceFrame controlFrame, String graphicFileName, GDXFocusBasedCamera camera3D)
   {
      create(collisionLink, controlFrame, controlFrame, controlFrame, graphicFileName, camera3D);
   }

   public void create(GDXRobotCollisionLink collisionLink,
                      ReferenceFrame graphicFrame,
                      ReferenceFrame collisionFrame,
                      ReferenceFrame controlFrame,
                      String modelFileName,
                      GDXFocusBasedCamera camera3D)
   {
      this.collisionLink = collisionLink;
      this.graphicFrame = graphicFrame;
      this.collisionFrame = collisionFrame;
      this.controlFrame = controlFrame;
      hasMultipleFrames = !(graphicFrame == collisionFrame && collisionFrame == controlFrame);
      highlightModel = new GDXInteractableHighlightModel(modelFileName);
      selectablePose3DGizmo.create(camera3D);
      graphicReferenceFrameGraphic = new GDXReferenceFrameGraphic(0.2);
      controlReferenceFrameGraphic = new GDXReferenceFrameGraphic(0.2);
   }

   public void calculate3DViewPick(ImGui3DViewInput input)
   {
      selectablePose3DGizmo.calculate3DViewPick(input);
   }

   public void process3DViewInput(ImGui3DViewInput input)
   {
      pickSelected = collisionLink.getPickSelected();
      boolean isClickedOn = pickSelected && input.mouseReleasedWithoutDrag(ImGuiMouseButton.Left);
      boolean isDeletedThisFrame = modified && selectablePose3DGizmo.isSelected() && ImGui.isKeyReleased(ImGuiTools.getDeleteKey());
      boolean unmodifiedButHovered = !modified && pickSelected;
      boolean becomesModified = unmodifiedButHovered && isClickedOn;
      boolean executeMotionKeyPressed = ImGui.isKeyReleased(ImGuiTools.getSpaceKey());
      boolean modifiedButNotSelectedHovered = modified && !selectablePose3DGizmo.isSelected() && pickSelected;

      if (isDeletedThisFrame)
      {
         modified = false;
         collisionLink.setOverrideTransform(false);
      }

      selectablePose3DGizmo.process3DViewInput(input, pickSelected);

      if (unmodifiedButHovered)
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
      }

      if (becomesModified)
      {
         modified = true;
         collisionLink.setOverrideTransform(true);
         selectablePose3DGizmo.getPoseGizmo().getTransformToParent().set(controlFrame.getTransformToWorldFrame());
      }

      if (modifiedButNotSelectedHovered)
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
            selectablePose3DGizmo.getPoseGizmo().getTransformToParent().transform(tempTransform);
            collisionLink.setOverrideTransform(true).set(tempTransform);
            highlightModel.setPose(selectablePose3DGizmo.getPoseGizmo().getTransformToParent(), controlToGraphicTransform);
         }
         else
         {
            collisionLink.setOverrideTransform(true).set(selectablePose3DGizmo.getPoseGizmo().getTransformToParent());
            highlightModel.setPose(selectablePose3DGizmo.getPoseGizmo().getTransformToParent());
         }
      }

      if (selectablePose3DGizmo.isSelected() && executeMotionKeyPressed)
      {
         onSpacePressed.run();
      }
   }

   public void getVirtualRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (SHOW_DEBUG_FRAMES)
      {
         if (hasMultipleFrames)
         {
            graphicReferenceFrameGraphic.setToReferenceFrame(graphicFrame);
            graphicReferenceFrameGraphic.getRenderables(renderables, pool);
         }
         controlReferenceFrameGraphic.setToReferenceFrame(controlFrame);
         controlReferenceFrameGraphic.getRenderables(renderables, pool);
      }

      if (modified || pickSelected)
      {
         highlightModel.getRenderables(renderables, pool);
      }
      selectablePose3DGizmo.getVirtualRenderables(renderables, pool);
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
      return selectablePose3DGizmo.getPoseGizmo().getPose();
   }

   public void setOnSpacePressed(Runnable onSpacePressed)
   {
      this.onSpacePressed = onSpacePressed;
   }
}
