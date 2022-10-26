package us.ihmc.rdx.ui.affordances;

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
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.graphics.RDXReferenceFrameGraphic;
import us.ihmc.rdx.vr.RDXVRContext;
import us.ihmc.robotics.referenceFrames.ModifiableReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.ArrayList;

/**
 * An interactable robot link is a link on the robot that the user can
 * select and move around for various purposes. Modified means it is not updating
 * from the robot's pose -- the user has modified it. Selected means the
 * gizmo is showing, the arrow keys will move it, and usually, the spacebar
 * will send some action based on it.
 */
public class RDXInteractableRobotLink
{
   private static final boolean SHOW_DEBUG_FRAMES = false;
   private final ArrayList<RDXRobotCollidable> robotCollidables = new ArrayList<>();
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private ReferenceFrame graphicFrame;
   private ReferenceFrame collisionFrame;
   private ReferenceFrame controlFrame;
   private boolean hasMultipleFrames;
   private RigidBodyTransform controlToGraphicTransform;
   private RigidBodyTransform controlToCollisionTransform;
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();
   private final FramePose3D tempFramePose = new FramePose3D();
   private RDXInteractableHighlightModel highlightModel;
   private boolean modified = false;
   private final RDXSelectablePose3DGizmo selectablePose3DGizmo = new RDXSelectablePose3DGizmo();
   private Runnable onSpacePressed;
   private RDXReferenceFrameGraphic graphicReferenceFrameGraphic;
   private RDXReferenceFrameGraphic controlReferenceFrameGraphic;
   private boolean isMouseHovering;
   private final Notification contextMenuNotification = new Notification();
   private boolean isVRHovering;
   private boolean isVRDragging = false;
   private final SideDependentList<ModifiableReferenceFrame> dragReferenceFrame = new SideDependentList<>();

   public void create(RDXRobotCollidable robotCollidable, ReferenceFrame controlFrame, String graphicFileName, RDX3DPanel panel3D)
   {
      create(robotCollidable, controlFrame, controlFrame, controlFrame, graphicFileName, panel3D);
   }

   public void create(RDXRobotCollidable robotCollidable,
                      ReferenceFrame graphicFrame,
                      ReferenceFrame collisionFrame,
                      ReferenceFrame controlFrame,
                      String modelFileName,
                      RDX3DPanel panel3D)
   {
      robotCollidables.add(robotCollidable);
      this.graphicFrame = graphicFrame;
      this.collisionFrame = collisionFrame;
      this.controlFrame = controlFrame;
      hasMultipleFrames = !(graphicFrame == collisionFrame && collisionFrame == controlFrame);
      highlightModel = new RDXInteractableHighlightModel(modelFileName);
      selectablePose3DGizmo.create(panel3D);
      graphicReferenceFrameGraphic = new RDXReferenceFrameGraphic(0.2);
      controlReferenceFrameGraphic = new RDXReferenceFrameGraphic(0.2);
   }

   public void update()
   {
      ensureMutlipleFramesAreSetup();

      if (modified && !selectablePose3DGizmo.isSelected() && (isMouseHovering || isVRHovering))
      {
         highlightModel.setTransparency(0.7);
      }
      else
      {
         highlightModel.setTransparency(0.5);
      }
   }

   public void processVRInput(RDXVRContext vrContext)
   {
      isVRHovering = false;

      for (RobotSide side : RobotSide.values)
      {
         vrContext.getController(side).runIfConnected(controller ->
         {
            boolean isHovering = false;
            for (RDXRobotCollidable robotCollidable : robotCollidables)
            {
               isHovering |= robotCollidable.getVRPickSelected(side);
            }
            isVRHovering |= isHovering;

            boolean triggerDown = controller.getClickTriggerActionData().bState();
            boolean triggerNewlyDown = triggerDown && controller.getClickTriggerActionData().bChanged();
            boolean triggerNewlyUp = !triggerDown && controller.getClickTriggerActionData().bChanged();

            if (dragReferenceFrame.get(side) == null)
            {
               dragReferenceFrame.put(side, new ModifiableReferenceFrame(controller.getPickPoseFrame()));
            }

            if (!modified && isHovering)
            {
               updateUnmodifiedButHovered();
            }

            if (isHovering && triggerNewlyDown)
            {
               if (!modified)
               {
                  onBecomesModified();
               }

               selectablePose3DGizmo.getPoseGizmo().getGizmoFrame().getTransformToDesiredFrame(dragReferenceFrame.get(side).getTransformToParent(),
                                                                                               controller.getPickPoseFrame());
               dragReferenceFrame.get(side).getReferenceFrame().update();
               isVRDragging = true;
            }

            if (isVRDragging)
            {
               dragReferenceFrame.get(side).getReferenceFrame().getTransformToDesiredFrame(selectablePose3DGizmo.getPoseGizmo().getTransformToParent(),
                                                                                           ReferenceFrame.getWorldFrame());
               selectablePose3DGizmo.getPoseGizmo().updateTransforms();
               updateModified();
            }

            if (triggerNewlyUp)
               isVRDragging = false;
         });
      }
   }

   public void calculate3DViewPick(ImGui3DViewInput input)
   {
      selectablePose3DGizmo.calculate3DViewPick(input);
   }

   public boolean process3DViewInput(ImGui3DViewInput input)
   {
      isMouseHovering = false;
      for (RDXRobotCollidable robotCollidable : robotCollidables)
      {
         isMouseHovering |= robotCollidable.getMousePickSelected();
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

      if (isDeletedThisFrame)
      {
         delete();
      }

      selectablePose3DGizmo.process3DViewInput(input, isMouseHovering);

      if (unmodifiedButHovered)
      {
         updateUnmodifiedButHovered();
      }

      if (becomesModified)
      {
         onBecomesModified();
      }

      if (modified)
      {
         updateModified();
      }

      if (selectablePose3DGizmo.isSelected() && executeMotionKeyPressed)
      {
         onSpacePressed.run();
      }
      return becomesModified;
   }

   private void updateModified()
   {
      if (hasMultipleFrames)
      {
         tempTransform.set(controlToCollisionTransform);
         selectablePose3DGizmo.getPoseGizmo().getTransformToParent().transform(tempTransform);
         for (RDXRobotCollidable robotCollidable : robotCollidables)
         {
            robotCollidable.setDetachedTransform(true).set(tempTransform);
         }
         highlightModel.setPose(selectablePose3DGizmo.getPoseGizmo().getTransformToParent(), controlToGraphicTransform);
      }
      else
      {
         for (RDXRobotCollidable robotCollidable : robotCollidables)
         {
            robotCollidable.setDetachedTransform(true).set(selectablePose3DGizmo.getPoseGizmo().getTransformToParent());
         }
         highlightModel.setPose(selectablePose3DGizmo.getPoseGizmo().getTransformToParent());
      }
   }

   private void updateUnmodifiedButHovered()
   {
      if (hasMultipleFrames)
      {
         highlightModel.setPose(controlFrame.getTransformToWorldFrame(), controlToGraphicTransform);
      }
      else
      {
         highlightModel.setPose(controlFrame.getTransformToWorldFrame());
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
      for (RDXRobotCollidable robotCollidable : robotCollidables)
      {
         robotCollidable.setDetachedTransform(true);
      }
      selectablePose3DGizmo.getPoseGizmo().getTransformToParent().set(controlFrame.getTransformToWorldFrame());
      selectablePose3DGizmo.getPoseGizmo().updateTransforms();
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

      boolean anyRobotCollidableHovered = false;
      for (RDXRobotCollidable robotCollidable : robotCollidables)
         anyRobotCollidableHovered |= robotCollidable.getAnyPickSelected();

      if (modified || anyRobotCollidableHovered)
      {
         highlightModel.getRenderables(renderables, pool);
      }
      selectablePose3DGizmo.getVirtualRenderables(renderables, pool);
   }

   public void delete()
   {
      modified = false;
      selectablePose3DGizmo.getSelected().set(false);
      for (RDXRobotCollidable robotCollidable : robotCollidables)
      {
         robotCollidable.setDetachedTransform(false);
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

   public void addAdditionalRobotCollidable(RDXRobotCollidable robotCollidable)
   {
      robotCollidables.add(robotCollidable);
   }

   public Notification getContextMenuNotification()
   {
      return contextMenuNotification;
   }
}
