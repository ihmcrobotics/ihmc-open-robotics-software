package us.ihmc.rdx.ui.behavior.editor.actions;

import behavior_msgs.msg.dds.BodyPartPoseStatusMessage;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.flag.ImGuiMouseButton;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.behaviors.sequence.BehaviorActionSequence;
import us.ihmc.behaviors.sequence.actions.PelvisHeightPitchActionDefinition;
import us.ihmc.behaviors.sequence.actions.PelvisHeightPitchActionState;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.euclid.referenceFrame.FramePose3D;
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

public class RDXPelvisHeightPitchAction extends RDXBehaviorAction
{
   private final PelvisHeightPitchActionState state;
   private final PelvisHeightPitchActionDefinition definition;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImDoubleWrapper heightWidget = new ImDoubleWrapper(definition::getHeight,
                                                                    definition::setHeight,
                                                                    imDouble -> ImGuiTools.volatileInputDouble(labels.get("Height"), imDouble));
   private final ImDoubleWrapper pitchWidget = new ImDoubleWrapper(definition::getPitch,
                                                                   definition::setPitch,
                                                                   imDouble -> ImGuiTools.volatileInputDouble(labels.get("Pitch"), imDouble));
   private final ImDoubleWrapper trajectoryDurationWidget = new ImDoubleWrapper(definition::getTrajectoryDuration,
                                                                                definition::setTrajectoryDuration,
                                                                                imDouble -> ImGuiTools.volatileInputDouble(labels.get("Trajectory duration"),
                                                                                                                           imDouble));
   /** Gizmo is control frame */
   private final RDXSelectablePose3DGizmo poseGizmo = new RDXSelectablePose3DGizmo(definition.getConditionalReferenceFrame().get(),
                                                                                   definition.getPelvisToParentTransform());
   private final ImBooleanWrapper executeWithNextActionWrapper = new ImBooleanWrapper(definition::getExecuteWithNextAction,
                                                                                      definition::setExecuteWithNextAction,
                                                                                      imBoolean -> ImGui.checkbox(labels.get("Execute with next action"),
                                                                                                                  imBoolean));
   private final ModifiableReferenceFrame graphicFrame = new ModifiableReferenceFrame(definition.getConditionalReferenceFrame().get());
   private final ModifiableReferenceFrame collisionShapeFrame = new ModifiableReferenceFrame(definition.getConditionalReferenceFrame().get());
   private boolean isMouseHovering = false;
   private final ImGui3DViewPickResult pickResult = new ImGui3DViewPickResult();
   private final ArrayList<MouseCollidable> mouseCollidables = new ArrayList<>();
   private final RDXInteractableHighlightModel highlightModel;
   private final ImGuiReferenceFrameLibraryCombo parentFrameComboBox;
   private final RDX3DPanelTooltip tooltip;
   private final FullHumanoidRobotModel syncedFullRobotModel;
   private final ROS2PublishSubscribeAPI ros2;
   private final BodyPartPoseStatusMessage pelvisPoseStatus = new BodyPartPoseStatusMessage();
   private final Throttler throttler = new Throttler().setFrequency(10.0);
   private boolean wasConcurrent = false;

   public RDXPelvisHeightPitchAction(RDXBehaviorActionSequenceEditor editor,
                                     RDX3DPanel panel3D,
                                     DRCRobotModel robotModel,
                                     FullHumanoidRobotModel syncedFullRobotModel,
                                     RobotCollisionModel selectionCollisionModel,
                                     ReferenceFrameLibrary referenceFrameLibrary,
                                     ROS2PublishSubscribeAPI ros2)
   {
      super(editor);

      this.ros2 = ros2;
      this.syncedFullRobotModel = syncedFullRobotModel;

      state = new PelvisHeightPitchActionState(referenceFrameLibrary);
      definition = state.getDefinition();

      String pelvisBodyName = syncedFullRobotModel.getPelvis().getName();
      String modelFileName = RDXInteractableTools.getModelFileName(robotModel.getRobotDefinition().getRigidBodyDefinition(pelvisBodyName));
      highlightModel = new RDXInteractableHighlightModel(modelFileName);

      MultiBodySystemBasics pelvisOnlySystem = MultiBodySystemMissingTools.createSingleBodySystem(syncedFullRobotModel.getPelvis());
      List<Collidable> pelvisCollidables = selectionCollisionModel.getRobotCollidables(pelvisOnlySystem);

      for (Collidable pelvisCollidable : pelvisCollidables)
      {
         mouseCollidables.add(new MouseCollidable(pelvisCollidable));
      }

      parentFrameComboBox = new ImGuiReferenceFrameLibraryCombo("Parent frame", referenceFrameLibrary);
      poseGizmo.create(panel3D);

      tooltip = new RDX3DPanelTooltip(panel3D);
      panel3D.addImGuiOverlayAddition(this::render3DPanelImGuiOverlays);
   }

   @Override
   public void update()
   {
      state.update();

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

      pelvisPoseStatus.getParentFrame().resetQuick();
      pelvisPoseStatus.getParentFrame().add(definition.get().getParent().getName());

      // compute transform variation from previous pose
      FramePose3D currentRobotPelvisPose = new FramePose3D(syncedFullRobotModel.getPelvis().getParentJoint().getFrameAfterJoint());
      if (definition.get().getParent() != currentRobotPelvisPose.getReferenceFrame())
         currentRobotPelvisPose.changeFrame(definition.get().getParent());
      RigidBodyTransform transformVariation = new RigidBodyTransform();
      transformVariation.setAndInvert(currentRobotPelvisPose);
      definition.getPelvisToParentTransform().transform(transformVariation);
      MessageTools.toMessage(transformVariation, pelvisPoseStatus.getTransformToParent());


      // if the action is part of a group of concurrent actions that is currently executing or about to be executed
      // send an update of the pose of the pelvis. Arms IK will be computed wrt this change of this pelvis pose
      if (concurrentActionIsNextForExecution)
      {
         wasConcurrent = true;
         pelvisPoseStatus.setCurrentAndConcurrent(true);
         if (throttler.run())
         {
            if (getActionIndex() >= 0)
               ros2.publish(BehaviorActionSequence.PELVIS_POSE_VARIATION_STATUS, pelvisPoseStatus);
         }
      }
      else if (wasConcurrent)
      {
         pelvisPoseStatus.setCurrentAndConcurrent(false);
         ros2.publish(BehaviorActionSequence.PELVIS_POSE_VARIATION_STATUS, pelvisPoseStatus);
         wasConcurrent = false;
      }
   }

   @Override
   public void updateBeforeRemoving()
   {
      pelvisPoseStatus.setCurrentAndConcurrent(false);
      ros2.publish(BehaviorActionSequence.PELVIS_POSE_VARIATION_STATUS, pelvisPoseStatus);
   }

   @Override
   protected void renderImGuiWidgetsInternal()
   {
      ImGui.sameLine();
      executeWithNextActionWrapper.renderImGuiWidget();
      parentFrameComboBox.render();
      ImGui.pushItemWidth(80.0f);
      heightWidget.renderImGuiWidget();
      pitchWidget.renderImGuiWidget();
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
         getSelected().set(true);
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

   public ReferenceFrame getReferenceFrame()
   {
      return poseGizmo.getPoseGizmo().getGizmoFrame();
   }

   @Override
   public String getActionTypeTitle()
   {
      return "Pelvis Height and Pitch";
   }

   @Override
   public PelvisHeightPitchActionState getState()
   {
      return state;
   }

   @Override
   public PelvisHeightPitchActionDefinition getDefinition()
   {
      return definition;
   }
}
