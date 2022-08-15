package us.ihmc.gdx.ui.affordances;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.flag.ImGuiMouseButton;
import imgui.internal.ImGui;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.gdx.imgui.ImGuiTools;
import us.ihmc.gdx.input.ImGui3DViewInput;
import us.ihmc.gdx.ui.GDX3DPanel;
import us.ihmc.gdx.ui.graphics.GDXReferenceFrameGraphic;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

public class GDXLiveRobotPartInteractable
{
   private static final boolean SHOW_DEBUG_FRAMES = false;
   private GDXRobotCollisionLink collisionLink;
   private ReferenceFrame graphicFrame;
   private ReferenceFrame collisionFrame;
   private ReferenceFrame linkEstimateFrame;
   private boolean hasMultipleFrames;
   private RigidBodyTransform estimateToGraphicTransform;
   private RigidBodyTransform estimateToCollisionTransform;
   private final FramePose3D tempFramePose = new FramePose3D();
   private GDXInteractableHighlightModel highlightModel;
   private boolean modified = false;
   private final GDXSelectablePose3DGizmo selectablePose3DGizmo = new GDXSelectablePose3DGizmo();
   private Runnable onSpacePressed;
   private GDXReferenceFrameGraphic graphicReferenceFrameGraphic;
   private GDXReferenceFrameGraphic controlReferenceFrameGraphic;
   private boolean pickSelected;

   private final List<Consumer<FramePose3DReadOnly>> poseHasUpdatedCallbacks = new ArrayList<>();

   public void create(GDXRobotCollisionLink collisionLink, ReferenceFrame linkEstimateFrame, String graphicFileName, GDX3DPanel panel3D)
   {
      create(collisionLink, linkEstimateFrame, linkEstimateFrame, linkEstimateFrame, graphicFileName, panel3D);
   }

   public void create(GDXRobotCollisionLink collisionLink,
                      ReferenceFrame graphicFrame,
                      ReferenceFrame collisionFrame,
                      ReferenceFrame linkEstimateFrame,
                      String modelFileName,
                      GDX3DPanel panel3D)
   {
      this.collisionLink = collisionLink;
      this.graphicFrame = graphicFrame;
      this.collisionFrame = collisionFrame;
      this.linkEstimateFrame = linkEstimateFrame;
      hasMultipleFrames = !(graphicFrame == collisionFrame && collisionFrame == linkEstimateFrame);
      highlightModel = new GDXInteractableHighlightModel(modelFileName);
      selectablePose3DGizmo.create(panel3D);
      graphicReferenceFrameGraphic = new GDXReferenceFrameGraphic(0.2);
      controlReferenceFrameGraphic = new GDXReferenceFrameGraphic(0.2);
   }

   public void addPoseHasUpdatedCallback(Consumer<FramePose3DReadOnly> poseHasUpdatedCallback)
   {
      this.poseHasUpdatedCallbacks.add(poseHasUpdatedCallback);
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
         if (hasMultipleFrames && estimateToGraphicTransform == null) // we just need to do this once
         {
            estimateToGraphicTransform = new RigidBodyTransform();
            tempFramePose.setToZero(graphicFrame);
            tempFramePose.changeFrame(linkEstimateFrame);
            tempFramePose.get(estimateToGraphicTransform);
            estimateToCollisionTransform = new RigidBodyTransform();
            tempFramePose.setToZero(collisionFrame);
            tempFramePose.changeFrame(linkEstimateFrame);
            tempFramePose.get(estimateToCollisionTransform);
         }

         if (hasMultipleFrames)
         {
            highlightModel.setPose(linkEstimateFrame.getTransformToWorldFrame(), estimateToGraphicTransform);
         }
         else
         {
            highlightModel.setPose(linkEstimateFrame.getTransformToWorldFrame());
         }
      }

      if (becomesModified)
      {
         modified = true;
         collisionLink.setOverrideTransform(true);
         selectablePose3DGizmo.getPoseGizmo().getTransformToParent().set(linkEstimateFrame.getTransformToWorldFrame());
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
            selectablePose3DGizmo.getPoseGizmo().getTransformToParent().transform(estimateToCollisionTransform);
            collisionLink.setOverrideTransform(true).set(estimateToCollisionTransform);
            highlightModel.setPose(selectablePose3DGizmo.getPoseGizmo().getTransformToParent(), estimateToGraphicTransform);
         }
         else
         {
            collisionLink.setOverrideTransform(true).set(selectablePose3DGizmo.getPoseGizmo().getTransformToParent());
            highlightModel.setPose(selectablePose3DGizmo.getPoseGizmo().getTransformToParent());
         }

         processPoseHasUpdatedCallbacks();
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
         controlReferenceFrameGraphic.setToReferenceFrame(linkEstimateFrame);
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

   private void processPoseHasUpdatedCallbacks()
   {
      for (Consumer<FramePose3DReadOnly> callback : poseHasUpdatedCallbacks)
         callback.accept(selectablePose3DGizmo.getPoseGizmo().getPose());
   }
}
