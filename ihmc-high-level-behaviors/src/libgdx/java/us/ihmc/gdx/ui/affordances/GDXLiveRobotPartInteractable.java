package us.ihmc.gdx.ui.affordances;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.flag.ImGuiMouseButton;
import imgui.ImGui;
import us.ihmc.commons.thread.Notification;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.gdx.imgui.ImGuiTools;
import us.ihmc.gdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.gdx.input.ImGui3DViewInput;
import us.ihmc.gdx.ui.GDX3DPanel;
import us.ihmc.gdx.ui.graphics.GDXReferenceFrameGraphic;
import us.ihmc.gdx.vr.GDXVRContext;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.ArrayList;

public class GDXLiveRobotPartInteractable
{
   private static final boolean SHOW_DEBUG_FRAMES = false;
   private final ArrayList<GDXRobotCollisionLink> collisionLinks = new ArrayList<>();
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
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
   private boolean isMouseHovering;
   private final Notification contextMenuNotification = new Notification();
   private boolean isVRHovering;

   public void create(GDXRobotCollisionLink collisionLink, ReferenceFrame controlFrame, String graphicFileName, GDX3DPanel panel3D)
   {
      create(collisionLink, controlFrame, controlFrame, controlFrame, graphicFileName, panel3D);
   }

   public void create(GDXRobotCollisionLink collisionLink,
                      ReferenceFrame graphicFrame,
                      ReferenceFrame collisionFrame,
                      ReferenceFrame controlFrame,
                      String modelFileName,
                      GDX3DPanel panel3D)
   {
      collisionLinks.add(collisionLink);
      this.graphicFrame = graphicFrame;
      this.collisionFrame = collisionFrame;
      this.controlFrame = controlFrame;
      hasMultipleFrames = !(graphicFrame == collisionFrame && collisionFrame == controlFrame);
      highlightModel = new GDXInteractableHighlightModel(modelFileName);
      selectablePose3DGizmo.create(panel3D);
      graphicReferenceFrameGraphic = new GDXReferenceFrameGraphic(0.2);
      controlReferenceFrameGraphic = new GDXReferenceFrameGraphic(0.2);
   }

   public void calculateVRPick(GDXVRContext vrContext)
   {

   }

   public void processVRInput(GDXVRContext vrContext)
   {
      isVRHovering = false;
      for (RobotSide side : RobotSide.values)
      {
         vrContext.getController(side).runIfConnected(controller ->
         {
//            controller.getSelectionPose()
         });
      }
   }

   public void calculate3DViewPick(ImGui3DViewInput input)
   {
      selectablePose3DGizmo.calculate3DViewPick(input);
   }

   public void process3DViewInput(ImGui3DViewInput input)
   {
      isMouseHovering = false;
      for (GDXRobotCollisionLink collisionLink : collisionLinks)
      {
         isMouseHovering |= collisionLink.getPickSelected();
      }

      if (isMouseHovering && ImGui.getMouseClickedCount(ImGuiMouseButton.Right) == 1)
      {
         contextMenuNotification.set();
      }

      boolean isClickedOn = isMouseHovering && input.mouseReleasedWithoutDrag(ImGuiMouseButton.Left);
      boolean isDeletedThisFrame = modified && selectablePose3DGizmo.isSelected() && ImGui.isKeyReleased(ImGuiTools.getDeleteKey());
      boolean unmodifiedButHovered = !modified && isMouseHovering;
      boolean becomesModified = unmodifiedButHovered && isClickedOn;
      boolean executeMotionKeyPressed = ImGui.isKeyReleased(ImGuiTools.getSpaceKey());
      boolean modifiedButNotSelectedHovered = modified && !selectablePose3DGizmo.isSelected() && isMouseHovering;

      if (isDeletedThisFrame)
      {
         delete();
      }

      selectablePose3DGizmo.process3DViewInput(input, isMouseHovering);

      if (unmodifiedButHovered)
      {
         ensureMutlipleFramesAreSetup();

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
         onBecomesModified();
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
            ensureMutlipleFramesAreSetup();

            tempTransform.set(controlToCollisionTransform);
            selectablePose3DGizmo.getPoseGizmo().getTransformToParent().transform(tempTransform);
            for (GDXRobotCollisionLink collisionLink : collisionLinks)
            {
               collisionLink.setOverrideTransform(true).set(tempTransform);
            }
            highlightModel.setPose(selectablePose3DGizmo.getPoseGizmo().getTransformToParent(), controlToGraphicTransform);
         }
         else
         {
            for (GDXRobotCollisionLink collisionLink : collisionLinks)
            {
               collisionLink.setOverrideTransform(true).set(selectablePose3DGizmo.getPoseGizmo().getTransformToParent());
            }
            highlightModel.setPose(selectablePose3DGizmo.getPoseGizmo().getTransformToParent());
         }
      }

      if (selectablePose3DGizmo.isSelected() && executeMotionKeyPressed)
      {
         onSpacePressed.run();
      }
   }

   private void ensureMutlipleFramesAreSetup()
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
   }

   private void onBecomesModified()
   {
      modified = true;
      for (GDXRobotCollisionLink collisionLink : collisionLinks)
      {
         collisionLink.setOverrideTransform(true);
      }
      selectablePose3DGizmo.getPoseGizmo().getTransformToParent().set(controlFrame.getTransformToWorldFrame());
   }

   public boolean renderImGuiWidgets()
   {
      boolean becomesModified = false;
      if (ImGui.radioButton(labels.get("Deleted"), isDeleted()))
      {
         delete();
      }
      ImGui.sameLine();
      if (ImGui.radioButton(labels.get("Modified"), !selectablePose3DGizmo.getSelected().get() && modified))
      {
         selectablePose3DGizmo.getSelected().set(false);
         if (!modified)
         {
            becomesModified = true;
            onBecomesModified();
         }
      }
      ImGui.sameLine();
      if (ImGui.radioButton(labels.get("Selected"), selectablePose3DGizmo.getSelected().get()))
      {
         selectablePose3DGizmo.getSelected().set(true);
         if (!modified)
         {
            becomesModified = true;
            onBecomesModified();
         }
      }
      return becomesModified;
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

      if (modified || isMouseHovering)
      {
         highlightModel.getRenderables(renderables, pool);
      }
      selectablePose3DGizmo.getVirtualRenderables(renderables, pool);
   }

   public void delete()
   {
      modified = false;
      selectablePose3DGizmo.getSelected().set(false);
      for (GDXRobotCollisionLink collisionLink : collisionLinks)
      {
         collisionLink.setOverrideTransform(false);
      }
   }

   public boolean isDeleted()
   {
      return !selectablePose3DGizmo.getSelected().get() && !modified;
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

   public FramePose3DReadOnly getPose()
   {
      return selectablePose3DGizmo.getPoseGizmo().getPose();
   }

   public ReferenceFrame getControlReferenceFrame()
   {
      return selectablePose3DGizmo.getPoseGizmo().getGizmoFrame();
   }

   public void setOnSpacePressed(Runnable onSpacePressed)
   {
      this.onSpacePressed = onSpacePressed;
   }

   public void addAdditionalCollisionLink(GDXRobotCollisionLink collisionLink)
   {
      collisionLinks.add(collisionLink);
   }

   public Notification getContextMenuNotification()
   {
      return contextMenuNotification;
   }
}
