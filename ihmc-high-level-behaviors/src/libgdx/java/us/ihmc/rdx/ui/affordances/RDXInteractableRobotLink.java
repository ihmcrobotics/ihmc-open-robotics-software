package us.ihmc.rdx.ui.affordances;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.flag.ImGuiMouseButton;
import imgui.ImGui;
import org.lwjgl.openvr.InputAnalogActionData;
import org.lwjgl.openvr.InputDigitalActionData;
import us.ihmc.commons.thread.Notification;
import us.ihmc.euclid.referenceFrame.FrameBox3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.tools.RDXModelInstance;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.gizmo.RDXSelectablePose3DGizmo;
import us.ihmc.rdx.vr.RDXVRContext;
import us.ihmc.rdx.vr.RDXVRDragData;
import us.ihmc.rdx.vr.RDXVRJoystickSelection;
import us.ihmc.robotics.referenceFrames.ModifiableReferenceFrame;
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
   private boolean isVRHovering;
   private Runnable openCommands;
   private Runnable closeCommands;
   private RDXModelInstance wordsBoxMesh;
   private final FrameBox3D selectionCollisionBox = new FrameBox3D();
   private final SideDependentList<Point3D> boxOffset = new SideDependentList<>();
   private ModifiableReferenceFrame wordsBoxReferenceFrame;
   private final FramePose3D wordsBoxFramePose = new FramePose3D();
   private RDXVRJoystickSelection pastJoystickSelection;

   /** For when the graphic, the link, and control frame are all the same. */
   public void create(RDXRobotCollidable robotCollidable, ReferenceFrame syncedControlFrame, String graphicFileName, RDX3DPanel panel3D)
   {
      create(robotCollidable, syncedControlFrame, new RigidBodyTransform(), new RigidBodyTransform(), graphicFileName, panel3D);
   }

   /** Used for the hands especially, which have 3 frames each. */
   public void create(RDXRobotCollidable robotCollidable,
                      ReferenceFrame syncedControlFrame,
                      RigidBodyTransform graphicToControlFrameTransform,
                      RigidBodyTransform linkToControlFrameTransform,
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

      selectionCollisionBox.getSize().set(0.0125, 0.075, 0.025);
      FramePoint3DBasics[] vertices = selectionCollisionBox.getVertices();
      wordsBoxMesh = new RDXModelInstance(RDXModelBuilder.buildModel(boxMeshBuilder -> boxMeshBuilder.addMultiLineBox(vertices,
                                                                                                                      0.0005,
                                                                                                                      new Color(Color.WHITE))));
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
            if (controller.getJoystickSelection() == null)
            {
               controller.setJoystickSelection(RDXVRJoystickSelection.NONE);
            }
            boolean isHovering = false;
            wordsBoxReferenceFrame = new ModifiableReferenceFrame(controller.getJoystickReferenceFrame().getReferenceFrame());
            for (RDXRobotCollidable robotCollidable : robotCollidables)
            {
               isHovering |= robotCollidable.getVRHovering(side);
            }
            isVRHovering |= isHovering;

            RDXVRDragData gripDragData = controller.getGripDragData();
            InputAnalogActionData joystick = controller.getJoystickActionData();
            InputDigitalActionData joystickButton = controller.getJoystickPressActionData();

            if (isHovering)
            {
               selectionCollisionBox.getPose().set(controller.getJoystickFramePose());
                  controller.setTopJoystickText("Open Hand");
                  controller.setBottomJoystickText("Close Hand");
                  controller.setRightJoystickText("Delete Interactable");
                  controller.setLeftJoystickText("Execute");
               if (joystick.x() < 0 && Math.abs(joystick.x()) > Math.abs(joystick.y()) || controller.getJoystickSelection() == RDXVRJoystickSelection.EXECUTE)
               {
                  controller.setAllJoystickTextNull();
                  controller.setLeftJoystickText("Execute");
                  controller.setJoystickSelection(RDXVRJoystickSelection.EXECUTE);
               }
               if (joystick.x() > 0 && Math.abs(joystick.x()) > Math.abs(joystick.y())
                   || controller.getJoystickSelection() == RDXVRJoystickSelection.DELETE_INTERACTABLE)
               {
                  controller.setAllJoystickTextNull();
                  controller.setRightJoystickText("Delete Interactable");
                  controller.setJoystickSelection(RDXVRJoystickSelection.DELETE_INTERACTABLE);
               }
               if (joystick.y() > 0 && Math.abs(joystick.y()) > Math.abs(joystick.x()) || controller.getJoystickSelection() == RDXVRJoystickSelection.OPEN_HAND)
               {
                  controller.setAllJoystickTextNull();
                  controller.setTopJoystickText("Open Hand");
                  controller.setJoystickSelection(RDXVRJoystickSelection.OPEN_HAND);
               }
               if (joystick.y() < 0 && Math.abs(joystick.y()) > Math.abs(joystick.x())
                   || controller.getJoystickSelection() == RDXVRJoystickSelection.CLOSE_HAND)
               {
                  controller.setAllJoystickTextNull();
                  controller.setBottomJoystickText("Close Hand");
                  controller.setJoystickSelection(RDXVRJoystickSelection.CLOSE_HAND);
               }
               if (joystick.x() == 0 && joystick.y() == 0)
               {
                  controller.setJoystickSelection(RDXVRJoystickSelection.NONE);
               }
               if (joystickButton.bChanged() && joystickButton.bState())
               {
                  switch (controller.getJoystickSelection())
                  {
                     case EXECUTE:
                        onSpacePressed.run();
                        controller.setJoystickSelection(RDXVRJoystickSelection.NONE);
                        break;
                     case DELETE_INTERACTABLE:
                        delete();
                        controller.setJoystickSelection(RDXVRJoystickSelection.NONE);
                        break;
                     case OPEN_HAND:
                        if (openCommands != null)
                        {
                           openCommands.run();
                           break;
                        }
                     case CLOSE_HAND:
                        if (closeCommands != null)
                        {
                           closeCommands.run();
                           break;
                        }
                  }
               }
            }
            else
            {
               controller.setJoystickSelection(null);
               controller.setAllJoystickTextNull();
            }

            if (isHovering || gripDragData.getObjectBeingDragged() == this)
            {
               if (gripDragData.getDragJustStarted())
               {
                  modified = true;
                  gripDragData.setObjectBeingDragged(this);
                  gripDragData.setInteractableFrameOnDragStart(selectablePose3DGizmo.getPoseGizmo().getGizmoFrame());
               }
            }

            if (gripDragData.isDragging() && gripDragData.getObjectBeingDragged() == this)
            {
               gripDragData.getDragFrame().getTransformToDesiredFrame(selectablePose3DGizmo.getPoseGizmo().getTransformToParent(),
                                                                      selectablePose3DGizmo.getPoseGizmo().getGizmoFrame().getParent());
            }
            if (controller.getOffset() != null)
            {
               boxOffset.put(side, controller.getOffset());
               pastJoystickSelection = controller.getJoystickSelection();
               updateHoverBoxFramePose(side);
            }
            else if (controller.getOffset() == null && isHovering)
            {
               if (pastJoystickSelection != null && pastJoystickSelection != RDXVRJoystickSelection.NONE)
               {
                  controller.setJoystickSelection(pastJoystickSelection);
                  boxOffset.put(side, controller.getOffset());
                  updateHoverBoxFramePose(side);
               }
            }
            else if (controller.getJoystickSelection() == null)
            {
               boxOffset.put(side, null);
            }
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
            modified = true;
         }
      }
      ImGui.sameLine();
      if (ImGui.radioButton(labels.get("Selected"), selectablePose3DGizmo.getSelected().get()))
      {
         selectablePose3DGizmo.getSelected().set(true);
         if (!modified)
         {
            becomesModified = true;
            modified = true;
         }
      }
      return becomesModified;
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

      if (wordsBoxMesh != null && (boxOffset.get(RobotSide.LEFT) != null || boxOffset.get(RobotSide.RIGHT) != null))
         wordsBoxMesh.getRenderables(renderables, pool);
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

   public void setOnSpacePressed(Runnable onSpacePressed)
   {
      this.onSpacePressed = onSpacePressed;
   }

   public void setOpenCommands(Runnable openCommands)
   {
      this.openCommands = openCommands;
   }

   public void setCloseCommands(Runnable closeCommands)
   {
      this.closeCommands = closeCommands;
   }

   private void updateHoverBoxFramePose(RobotSide side)
   {
      selectionCollisionBox.getPose().getTranslation().add(boxOffset.get(side));
      wordsBoxReferenceFrame.getReferenceFrame().getTransformToParent().getTranslation().set(boxOffset.get(side));
      wordsBoxReferenceFrame.getReferenceFrame().update();
      wordsBoxFramePose.setToZero(wordsBoxReferenceFrame.getReferenceFrame());
      wordsBoxFramePose.getTranslation().add(boxOffset.get(side));
      wordsBoxFramePose.getTranslation().subY(0.03);
      wordsBoxFramePose.getRotation().setToYawOrientation(-0.2);
      wordsBoxFramePose.changeFrame(ReferenceFrame.getWorldFrame());
      wordsBoxMesh.setPoseInWorldFrame(wordsBoxFramePose);
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
}