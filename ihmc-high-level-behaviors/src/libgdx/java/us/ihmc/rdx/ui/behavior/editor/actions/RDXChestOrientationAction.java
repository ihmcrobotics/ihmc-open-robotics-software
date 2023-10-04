package us.ihmc.rdx.ui.behavior.editor.actions;

import behavior_msgs.msg.dds.BodyPartPoseStatusMessage;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.flag.ImGuiMouseButton;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.behaviors.sequence.BehaviorActionSequence;
import us.ihmc.behaviors.sequence.actions.ChestOrientationActionDefinition;
import us.ihmc.behaviors.sequence.actions.ChestOrientationActionState;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemBasics;
import us.ihmc.rdx.imgui.*;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.input.ImGui3DViewPickResult;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.RDX3DPanelTooltip;
import us.ihmc.rdx.ui.affordances.RDXInteractableHighlightModel;
import us.ihmc.rdx.ui.affordances.RDXInteractableTools;
import us.ihmc.rdx.ui.behavior.editor.RDXBehaviorAction;
import us.ihmc.rdx.ui.behavior.editor.RDXBehaviorActionSequenceEditor;
import us.ihmc.rdx.ui.gizmo.RDXSelectablePose3DGizmo;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.MultiBodySystemMissingTools;
import us.ihmc.robotics.interaction.MouseCollidable;
import us.ihmc.robotics.physics.Collidable;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.robotics.referenceFrames.ModifiableReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.tools.thread.Throttler;

import java.util.ArrayList;
import java.util.List;

public class RDXChestOrientationAction extends RDXBehaviorAction
{
   private final ChestOrientationActionState state;
   private final ChestOrientationActionDefinition definition;
   private final ReferenceFrameLibrary referenceFrameLibrary;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImDoubleWrapper yawWidget = new ImDoubleWrapper(() -> definition.getRotation().getYaw(), definition::setYaw,
                                                                 imDouble -> ImGuiTools.volatileInputDouble(labels.get("Yaw"), imDouble));
   private final ImDoubleWrapper pitchWidget = new ImDoubleWrapper(() -> definition.getRotation().getPitch(), definition::setPitch,
                                                                   imDouble -> ImGuiTools.volatileInputDouble(labels.get("Pitch"), imDouble));
   private final ImDoubleWrapper rollWidget = new ImDoubleWrapper(() -> definition.getRotation().getRoll(), definition::setRoll,
                                                                  imDouble -> ImGuiTools.volatileInputDouble(labels.get("Roll"), imDouble));
   private final ImDoubleWrapper trajectoryDurationWidget = new ImDoubleWrapper(definition::getTrajectoryDuration,
                                                                                definition::setTrajectoryDuration,
                                                                                imDouble -> ImGuiTools.volatileInputDouble(labels.get("Trajectory duration"),
                                                                                                                           imDouble));
   /** Gizmo is control frame */
   private final RDXSelectablePose3DGizmo poseGizmo = new RDXSelectablePose3DGizmo(definition.getConditionalReferenceFrame().get(),
                                                                                   definition.getTransformToParent());
   private final ImBooleanWrapper selectedWrapper = new ImBooleanWrapper(() -> poseGizmo.getSelected().get(),
                                                                         value -> poseGizmo.getSelected().set(value),
                                                                         imBoolean -> ImGui.checkbox(labels.get("Selected"), imBoolean));
   private final ImBooleanWrapper executeWithNextActionWrapper = new ImBooleanWrapper(definition::getExecuteWithNextAction,
                                                                                      definition::setExecuteWithNextAction,
                                                                                      imBoolean -> ImGui.checkbox(labels.get("Execute with next action"), imBoolean));
   private final ImBooleanWrapper holdPoseInWorldLaterWrapper = new ImBooleanWrapper(definition::getHoldPoseInWorldLater,
                                                                                     definition::setHoldPoseInWorldLater,
                                                                                     imBoolean -> ImGui.checkbox(labels.get("Hold pose in world later"), imBoolean));
   private final ModifiableReferenceFrame graphicFrame = new ModifiableReferenceFrame(definition.getConditionalReferenceFrame().get());
   private final ModifiableReferenceFrame collisionShapeFrame = new ModifiableReferenceFrame(definition.getConditionalReferenceFrame().get());
   private boolean isMouseHovering = false;
   private final ImGui3DViewPickResult pickResult = new ImGui3DViewPickResult();
   private final ArrayList<MouseCollidable> mouseCollidables = new ArrayList<>();
   private final RDXInteractableHighlightModel highlightModel;
   private final ImGuiReferenceFrameLibraryCombo parentFrameComboBox;
   private final RDX3DPanelTooltip tooltip;
   private final ROS2PublishSubscribeAPI ros2;
   private final BodyPartPoseStatusMessage chestPoseStatus = new BodyPartPoseStatusMessage();
   private final Throttler throttler = new Throttler().setFrequency(10.0);
   private boolean wasConcurrent = false;

   public RDXChestOrientationAction(RDXBehaviorActionSequenceEditor editor,
                                    RDX3DPanel panel3D,
                                    DRCRobotModel robotModel,
                                    FullHumanoidRobotModel syncedFullRobotModel,
                                    RobotCollisionModel selectionCollisionModel,
                                    ReferenceFrameLibrary referenceFrameLibrary,
                                    ROS2PublishSubscribeAPI ros2)
   {
      super(editor);

      this.ros2 = ros2;
      this.referenceFrameLibrary = referenceFrameLibrary;

      state = new ChestOrientationActionState(referenceFrameLibrary);
      definition = state.getDefinition();

      String chestBodyName = syncedFullRobotModel.getChest().getName();
      String modelFileName = RDXInteractableTools.getModelFileName(robotModel.getRobotDefinition().getRigidBodyDefinition(chestBodyName));
      highlightModel = new RDXInteractableHighlightModel(modelFileName);

      MultiBodySystemBasics chestOnlySystem = MultiBodySystemMissingTools.createSingleBodySystem(syncedFullRobotModel.getChest());
      List<Collidable> chestCollidables = selectionCollisionModel.getRobotCollidables(chestOnlySystem);

      for (Collidable chestCollidable : chestCollidables)
      {
         mouseCollidables.add(new MouseCollidable(chestCollidable));
      }

      parentFrameComboBox = new ImGuiReferenceFrameLibraryCombo("Parent frame",
                                                                referenceFrameLibrary,
                                                                definition::getParentFrameName,
                                                                definition::setParentFrameName);
      poseGizmo.create(panel3D);

      tooltip = new RDX3DPanelTooltip(panel3D);
      panel3D.addImGuiOverlayAddition(this::render3DPanelImGuiOverlays);
   }

   @Override
   public void updateAfterLoading()
   {
      parentFrameComboBox.setSelectedReferenceFrame(definition.getConditionalReferenceFrame());
   }

   public void setIncludingFrame(ReferenceFrame parentFrame, RigidBodyTransform transformToParent)
   {
      definition.getConditionalReferenceFrame().setParentFrameName(parentFrame.getName());
      definition.setTransformToParent(transformToParent);
      update();
   }

   public void setToReferenceFrame(ReferenceFrame referenceFrame)
   {
      definition.getConditionalReferenceFrame().setParentFrameName(ReferenceFrame.getWorldFrame().getName());
      definition.setTransformToParent(referenceFrame.getTransformToWorldFrame());
      update();
   }

   @Override
   public void update()
   {
      definition.update(referenceFrameLibrary);

      if (poseGizmo.getPoseGizmo().getGizmoFrame() != definition.getConditionalReferenceFrame().get())
      {
         poseGizmo.getPoseGizmo().setGizmoFrame(definition.getConditionalReferenceFrame().get());
         graphicFrame.changeParentFrame(definition.getConditionalReferenceFrame().get());
         collisionShapeFrame.changeParentFrame(definition.getConditionalReferenceFrame().get());
      }

      poseGizmo.getPoseGizmo().update();
      highlightModel.setPose(graphicFrame.getReferenceFrame());

      if (poseGizmo.isSelected() || isMouseHovering)
      {
         highlightModel.setTransparency(0.7);
      }
      else
      {
         highlightModel.setTransparency(0.5);
      }

      chestPoseStatus.getParentFrame().resetQuick();
      chestPoseStatus.getParentFrame().add(definition.get().getParent().getName());
      MessageTools.toMessage(definition.getTransformToParent(), chestPoseStatus.getTransformToParent());
      // if the action is part of a group of concurrent actions that is currently executing or about to be executed
      // send an update of the pose of the chest. Arms IK will be computed wrt this chest pose
      if (concurrentActionIsNextForExecution)
      {
         wasConcurrent = true;
         chestPoseStatus.setCurrentAndConcurrent(true);
         if (throttler.run())
         {
            if (getActionIndex() >= 0)
               ros2.publish(BehaviorActionSequence.CHEST_POSE_STATUS, chestPoseStatus);
         }
      }
      else if (wasConcurrent)
      {
         chestPoseStatus.setCurrentAndConcurrent(false);
         ros2.publish(BehaviorActionSequence.CHEST_POSE_STATUS, chestPoseStatus);
         wasConcurrent = false;
      }
   }

   @Override
   public void updateBeforeRemoving()
   {
      chestPoseStatus.setCurrentAndConcurrent(false);
      ros2.publish(BehaviorActionSequence.CHEST_POSE_STATUS, chestPoseStatus);
   }

   @Override
   protected void renderImGuiWidgetsInternal()
   {
      ImGui.sameLine();
      executeWithNextActionWrapper.renderImGuiWidget();
      holdPoseInWorldLaterWrapper.renderImGuiWidget();
      if (parentFrameComboBox.render())
      {
         definition.getConditionalReferenceFrame().setParentFrameName(parentFrameComboBox.getSelectedReferenceFrame().getParent().getName());
      }
      ImGui.pushItemWidth(80.0f);
      yawWidget.renderImGuiWidget();
      ImGui.sameLine();
      pitchWidget.renderImGuiWidget();
      ImGui.sameLine();
      rollWidget.renderImGuiWidget();
      trajectoryDurationWidget.renderImGuiWidget();
      ImGui.popItemWidth();
   }

   public void render3DPanelImGuiOverlays()
   {
      if (isMouseHovering)
      {
         tooltip.render("%s Action\nIndex: %d\nDescription: %s".formatted(getActionTypeTitle(),
                                                                          getActionIndex(),
                                                                          definition.getDescription()));
      }
   }

   @Override
   public void calculate3DViewPick(ImGui3DViewInput input)
   {
      poseGizmo.calculate3DViewPick(input);

      pickResult.reset();
      for (MouseCollidable mouseCollidable : mouseCollidables)
      {
         double collision = mouseCollidable.collide(input.getPickRayInWorld(), collisionShapeFrame.getReferenceFrame());
         if (!Double.isNaN(collision))
            pickResult.addPickCollision(collision);
      }
      if (pickResult.getPickCollisionWasAddedSinceReset())
         input.addPickResult(pickResult);
   }

   @Override
   public void process3DViewInput(ImGui3DViewInput input)
   {
      isMouseHovering = input.getClosestPick() == pickResult;

      boolean isClickedOn = isMouseHovering && input.mouseReleasedWithoutDrag(ImGuiMouseButton.Left);
      if (isClickedOn)
      {
         selectedWrapper.set(true);
      }

      poseGizmo.process3DViewInput(input, isMouseHovering);

      tooltip.setInput(input);
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      highlightModel.getRenderables(renderables, pool);
      poseGizmo.getVirtualRenderables(renderables, pool);
   }

   @Override
   public ImBooleanWrapper getSelected()
   {
      return selectedWrapper;
   }

   public ReferenceFrame getReferenceFrame()
   {
      return poseGizmo.getPoseGizmo().getGizmoFrame();
   }

   @Override
   public String getActionTypeTitle()
   {
      return "Chest Orientation";
   }

   @Override
   public ChestOrientationActionState getState()
   {
      return state;
   }

   @Override
   public ChestOrientationActionDefinition getDefinition()
   {
      return definition;
   }
}
