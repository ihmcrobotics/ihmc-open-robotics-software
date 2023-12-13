package us.ihmc.rdx.ui.affordances;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.flag.ImGuiMouseButton;
import imgui.ImGui;
import org.lwjgl.openvr.InputDigitalActionData;
import us.ihmc.commons.thread.Notification;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.gizmo.RDXSelectablePose3DGizmo;
import us.ihmc.rdx.vr.RDXVRContext;
import us.ihmc.rdx.vr.RDXVRDragData;
import us.ihmc.robotics.referenceFrames.ReferenceFrameMissingTools;
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
   private final ArrayList<RDXRobotCollidable> robotCollidables = new ArrayList<>();
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   /** The frame tracks the real robot's live pose. */
   private ReferenceFrame syncedControlFrame;
   private ReferenceFrame graphicFrame;
   /** Link frame is used to specify the collidables for user selection. */
   private ReferenceFrame linkFrame;
   private RDXInteractableHighlightModel highlightModel;
   private boolean modified = false;
   private RDXSelectablePose3DGizmo selectablePose3DGizmo;
   private Runnable onSpacePressed;
   private boolean isMouseHovering;
   private final Notification contextMenuNotification = new Notification();
   private final SideDependentList<Boolean> isVRHovering = new SideDependentList<>(false, false);
   private final SideDependentList<Boolean> isVRPointing = new SideDependentList<>(false, false);
   private final Notification becomesModified = new Notification();

   /** For when the graphic, the link, and control frame are all the same. */
   public void create(RDXRobotCollidable robotCollidable, ReferenceFrame syncedControlFrame, String graphicFileName, RDX3DPanel panel3D)
   {
      create(robotCollidable, syncedControlFrame, new RigidBodyTransform(), new RigidBodyTransform(), graphicFileName, panel3D);
   }

   /** Used for the hands especially, which have 3 frames each. */
   public void create(RDXRobotCollidable robotCollidable,
                      ReferenceFrame syncedControlFrame,
                      RigidBodyTransformReadOnly graphicToControlFrameTransform,
                      RigidBodyTransformReadOnly linkToControlFrameTransform,
                      String modelFileName,
                      RDX3DPanel panel3D)
   {
      this.syncedControlFrame = syncedControlFrame;
      selectablePose3DGizmo = new RDXSelectablePose3DGizmo();
      robotCollidables.add(robotCollidable);
      graphicFrame = ReferenceFrameMissingTools.constructFrameWithUnchangingTransformToParent(selectablePose3DGizmo.getPoseGizmo().getGizmoFrame(),
                                                                                              graphicToControlFrameTransform);
      linkFrame = ReferenceFrameMissingTools.constructFrameWithUnchangingTransformToParent(selectablePose3DGizmo.getPoseGizmo().getGizmoFrame(),
                                                                                           linkToControlFrameTransform);
      highlightModel = new RDXInteractableHighlightModel(modelFileName);
      selectablePose3DGizmo.create(panel3D);

      RDXBaseUI.getInstance().getKeyBindings().register("Execute / pause motion", "Space");
      RDXBaseUI.getInstance().getKeyBindings().register("Delete selected gizmo", "Delete");
   }

   public void update()
   {
      if (!modified) // Ensure the gizmo is at the hand when not modified
      {
         selectablePose3DGizmo.getPoseGizmo().getTransformToParent().set(syncedControlFrame.getTransformToRoot());
      }
      selectablePose3DGizmo.getPoseGizmo().update();
      for (RDXRobotCollidable robotCollidable : robotCollidables)
      {
         if (modified)
            robotCollidable.setDetached(linkFrame);
         else
            robotCollidable.setAttachedToSyncedLink();
      }

      highlightModel.setPose(graphicFrame);

      if (modified && !selectablePose3DGizmo.isSelected() && (isMouseHovering
                                                           || isVRHovering.get(RobotSide.LEFT)
                                                           || isVRHovering.get(RobotSide.RIGHT)
                                                           || isVRPointing.get(RobotSide.LEFT)
                                                           || isVRPointing.get(RobotSide.RIGHT)))
      {
         highlightModel.setTransparency(0.7);
      }
      else
      {
         highlightModel.setTransparency(0.5);
      }

      for (RobotSide side : RobotSide.values)
      {
         isVRHovering.put(side, false);
         isVRPointing.put(side, false);
      }
      isMouseHovering = false;
   }

   public void calculateVRPick(RDXVRContext vrContext)
   {
      for (RDXRobotCollidable robotCollidable : robotCollidables)
      {
         robotCollidable.calculateVRPick(vrContext);
      }
   }

   public void processVRInput(RDXVRContext vrContext)
   {
      for (RDXRobotCollidable robotCollidable : robotCollidables)
      {
         robotCollidable.processVRInput(vrContext);
      }

      for (RobotSide side : RobotSide.values)
      {
         vrContext.getController(side).runIfConnected(controller ->
         {
            for (RDXRobotCollidable robotCollidable : robotCollidables)
            {
               if (robotCollidable.getVRHovering(side))
                  isVRHovering.put(side, true);
               if (robotCollidable.getVRPointing(side))
                  isVRPointing.put(side, true);
            }

            RDXVRDragData gripDragData = controller.getGripDragData();
            InputDigitalActionData aButton = controller.getAButtonActionData();
            InputDigitalActionData bButton = controller.getBButtonActionData();

            // We want the delete and execute buttons to work when dragging, but
            // sometimes it's not hovering while dragging due to lag, so we also
            // enter this block if it's just being dragged.
            if (isVRHovering.get(side) || gripDragData.isBeingDragged(this))
            {
               if (gripDragData.getDragJustStarted())
               {
                  modified = true;
                  gripDragData.setObjectBeingDragged(this);
                  gripDragData.setInteractableFrameOnDragStart(selectablePose3DGizmo.getPoseGizmo().getGizmoFrame());
               }

               if (modified)
               {
                  controller.setBButtonText("Delete");
                  controller.setAButtonText("Execute");
                  if (aButton.bState() && aButton.bChanged())
                  {
                     onSpacePressed.run();
                  }
                  if (bButton.bState() && bButton.bChanged())
                  {
                     delete();
                  }
               }
            }

            if (gripDragData.isBeingDragged(this))
            {
               gripDragData.getDragFrame().getTransformToDesiredFrame(selectablePose3DGizmo.getPoseGizmo().getTransformToParent(),
                                                                      selectablePose3DGizmo.getPoseGizmo().getGizmoFrame().getParent());
            }
         });
      }
   }

   public void calculate3DViewPick(ImGui3DViewInput input)
   {
      for (RDXRobotCollidable robotCollidable : robotCollidables)
      {
         robotCollidable.calculatePick(input);
      }

      selectablePose3DGizmo.calculate3DViewPick(input);
   }

   public boolean process3DViewInput(ImGui3DViewInput input)
   {
      for (RDXRobotCollidable robotCollidable : robotCollidables)
      {
         robotCollidable.process3DViewInput(input);
         isMouseHovering |= robotCollidable.getMouseHovering();
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

      if (becomesModified)
      {
         modified = true;
      }

      if (selectablePose3DGizmo.isSelected() && executeMotionKeyPressed)
      {
         onSpacePressed.run();
      }
      return becomesModified;
   }

   public void renderImGuiWidgets()
   {
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
            becomesModified.set();
            modified = true;
         }
      }
      ImGui.sameLine();
      if (ImGui.radioButton(labels.get("Selected"), selectablePose3DGizmo.getSelected().get()))
      {
         selectablePose3DGizmo.getSelected().set(true);
         if (!modified)
         {
            becomesModified.set();
            modified = true;
         }
      }
   }

   public void getVirtualRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      boolean anyRobotCollidableHovered = false;
      for (RDXRobotCollidable robotCollidable : robotCollidables)
         anyRobotCollidableHovered |= robotCollidable.getIsHoveredByAnything();

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
   }

   public boolean isDeleted()
   {
      return !selectablePose3DGizmo.getSelected().get() && !modified;
   }

   public void destroy()
   {
      highlightModel.dispose();
   }

   public FramePose3DReadOnly getPose()
   {
      return selectablePose3DGizmo.getPoseGizmo().getPose();
   }

   public ReferenceFrame getControlReferenceFrame()
   {
      return selectablePose3DGizmo.getPoseGizmo().getGizmoFrame();
   }

   public boolean isVRPointing(RobotSide side)
   {
      return isVRPointing.get(side);
   }

   public boolean isVRHovering(RobotSide side)
   {
      return isVRHovering.get(side);
   }

   public Notification getBecomesModified()
   {
      return becomesModified;
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

   public Notification getGizmoModifiedByUser()
   {
      return selectablePose3DGizmo.getPoseGizmo().getGizmoModifiedByUser();
   }

   public RDXSelectablePose3DGizmo getSelectablePose3DGizmo()
   {
      return selectablePose3DGizmo;
   }

   public ReferenceFrame getSyncedControlFrame()
   {
      return syncedControlFrame;
   }

   public void selectInteractable()
   {
      isMouseHovering = true;
   }
}
