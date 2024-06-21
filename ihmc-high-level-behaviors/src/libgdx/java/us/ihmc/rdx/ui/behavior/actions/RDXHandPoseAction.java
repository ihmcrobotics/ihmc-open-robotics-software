package us.ihmc.rdx.ui.behavior.actions;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.flag.ImGuiMouseButton;
import imgui.type.ImInt;
import org.apache.commons.lang3.mutable.MutableObject;
import us.ihmc.avatar.arm.PresetArmConfiguration;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeTools;
import us.ihmc.behaviors.sequence.ActionSequenceState;
import us.ihmc.behaviors.sequence.actions.HandPoseActionDefinition;
import us.ihmc.behaviors.sequence.actions.HandPoseActionState;
import us.ihmc.communication.crdt.CRDTDetachableReferenceFrame;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.crdt.CRDTUnidirectionalRigidBodyTransform;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.rdx.imgui.ImBooleanWrapper;
import us.ihmc.rdx.imgui.ImDoubleWrapper;
import us.ihmc.rdx.imgui.ImGuiInputDouble;
import us.ihmc.rdx.imgui.ImGuiLabelledWidgetAligner;
import us.ihmc.rdx.imgui.ImGuiReferenceFrameLibraryCombo;
import us.ihmc.rdx.imgui.ImGuiSliderDouble;
import us.ihmc.rdx.imgui.ImGuiSliderDoubleWrapper;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.input.ImGui3DViewPickResult;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.RDX3DPanelTooltip;
import us.ihmc.rdx.ui.affordances.RDXInteractableHighlightModel;
import us.ihmc.rdx.ui.affordances.RDXInteractableTools;
import us.ihmc.rdx.ui.behavior.sequence.RDXActionNode;
import us.ihmc.rdx.ui.gizmo.RDXSelectablePose3DGizmo;
import us.ihmc.rdx.ui.graphics.RDXArmMultiBodyGraphic;
import us.ihmc.rdx.ui.graphics.RDXTrajectoryGraphic;
import us.ihmc.rdx.ui.teleoperation.RDXIKSolverColors;
import us.ihmc.rdx.ui.widgets.ImGuiHandWidget;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.EuclidCoreMissingTools;
import us.ihmc.robotics.MultiBodySystemMissingTools;
import us.ihmc.robotics.interaction.MouseCollidable;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.physics.Collidable;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.robotics.referenceFrames.MutableReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.io.WorkspaceResourceDirectory;
import us.ihmc.wholeBodyController.HandTransformTools;

import java.util.ArrayList;
import java.util.List;

public class RDXHandPoseAction extends RDXActionNode<HandPoseActionState, HandPoseActionDefinition>
{
   private final HandPoseActionState state;
   private final HandPoseActionDefinition definition;
   private final ROS2SyncedRobotModel syncedRobot;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImGuiHandWidget handIconWidget = new ImGuiHandWidget();
   /** Gizmo is control frame */
   private final RDXSelectablePose3DGizmo poseGizmo;
   private final SideDependentList<RigidBodyTransformReadOnly> handGraphicToControlFrameTransforms = new SideDependentList<>();
   private final MutableReferenceFrame graphicFrame = new MutableReferenceFrame();
   private final MutableReferenceFrame collisionShapeFrame = new MutableReferenceFrame();
   private boolean isMouseHovering = false;
   private final ImGui3DViewPickResult pickResult = new ImGui3DViewPickResult();
   private final ArrayList<MouseCollidable> mouseCollidables = new ArrayList<>();
   private final SideDependentList<RDXInteractableHighlightModel> highlightModels = new SideDependentList<>();
   private final ImGuiReferenceFrameLibraryCombo parentFrameComboBox;
   private final ImDoubleWrapper trajectoryDurationWidget;
   private final String[] configurations = new String[PresetArmConfiguration.values().length + 1];
   private final ImInt currentConfiguration = new ImInt(PresetArmConfiguration.HOME.ordinal() + 1);
   private final ImDoubleWrapper[] jointAngleWidgets = new ImDoubleWrapper[HandPoseActionDefinition.MAX_NUMBER_OF_JOINTS];
   private final ImGuiSliderDoubleWrapper linearPositionWeightWidget;
   private final ImGuiSliderDoubleWrapper angularPositionWeightWidget;
   private final ImGuiSliderDoubleWrapper jointspaceWeightWidget;
   private final ImBooleanWrapper holdPoseInWorldLaterWrapper;
   private final ImBooleanWrapper jointSpaceControlWrapper;
   private final ImBooleanWrapper usePredefinedJointAnglesWrapper;
   private final ImDoubleWrapper positionErrorToleranceInput;
   private final ImDoubleWrapper orientationErrorToleranceDegreesInput;
   private final SideDependentList<RDXArmMultiBodyGraphic> armMultiBodyGraphics = new SideDependentList<>();
   private final RDX3DPanelTooltip tooltip;
   private final RDXTrajectoryGraphic trajectoryGraphic = new RDXTrajectoryGraphic();

   public RDXHandPoseAction(long id,
                            CRDTInfo crdtInfo,
                            WorkspaceResourceDirectory saveFileDirectory,
                            RDX3DPanel panel3D,
                            DRCRobotModel robotModel,
                            ROS2SyncedRobotModel syncedRobot,
                            RobotCollisionModel selectionCollisionModel,
                            ReferenceFrameLibrary referenceFrameLibrary)
   {
      super(new HandPoseActionState(id, crdtInfo, saveFileDirectory, referenceFrameLibrary));

      state = getState();
      definition = getDefinition();

      this.syncedRobot = syncedRobot;

      definition.setName("Hand pose");

      poseGizmo = new RDXSelectablePose3DGizmo(ReferenceFrame.getWorldFrame(), definition.getPalmTransformToParent().getValue());
      poseGizmo.create(panel3D);

      trajectoryDurationWidget = new ImDoubleWrapper(definition::getTrajectoryDuration,
                                                     definition::setTrajectoryDuration,
                                                     imDouble -> ImGui.inputDouble(labels.get("Trajectory duration"), imDouble));
      int configurationIndex = 0;
      configurations[configurationIndex++] = HandPoseActionDefinition.CUSTOM_ANGLES_NAME;
      for (PresetArmConfiguration preset : PresetArmConfiguration.values())
      {
         configurations[configurationIndex++] = preset.name();
      }

      for (int i = 0; i < HandPoseActionDefinition.MAX_NUMBER_OF_JOINTS; i++)
      {
         int jointIndex = i;

         OneDoFJointBasics armJoint = syncedRobot.getFullRobotModel().getArmJoint(definition.getSide(), robotModel.getJointMap().getArmJointNames()[i]);
         double jointLimitLower = armJoint.getJointLimitLower();
         double jointLimitUpper = armJoint.getJointLimitUpper();

         final MutableObject<ImGuiInputDouble> fancyInput = new MutableObject<>();
         final MutableObject<ImGuiSliderDouble> fancySlider = new MutableObject<>();
         jointAngleWidgets[i] = new ImDoubleWrapper(() -> getDefinition().getJointAngles().getValue()[jointIndex],
                                                    jointAngle -> getDefinition().getJointAngles().getValue()[jointIndex] = jointAngle,
         imDouble ->
         {
            if (fancyInput.getValue() == null)
            {
               fancyInput.setValue(new ImGuiInputDouble("j" + jointIndex, "%.3f", imDouble));
               fancyInput.getValue().setWidgetWidth(119.0f);

               fancySlider.setValue(new ImGuiSliderDouble("", "", imDouble));
            }

            fancyInput.getValue().render(0.01, 0.1);

            ImGui.sameLine();
            fancySlider.getValue().setWidgetText("%.1f%s".formatted(Math.toDegrees(imDouble.get()), EuclidCoreMissingTools.DEGREE_SYMBOL));
            fancySlider.getValue().render(jointLimitLower, jointLimitUpper);
         });
      }
      holdPoseInWorldLaterWrapper = new ImBooleanWrapper(definition::getHoldPoseInWorldLater,
                                                         definition::setHoldPoseInWorldLater,
                                                         imBoolean -> ImGui.checkbox(labels.get("Hold pose in world later"), imBoolean));
      usePredefinedJointAnglesWrapper = new ImBooleanWrapper(definition::getUsePredefinedJointAngles,
                                                             definition::setUsePredefinedJointAngles,
                                                             imBoolean ->
                                                             {
                                                                if (ImGui.checkbox(labels.get("Use Predefined Joint Angles"), imBoolean))
                                                                {
                                                                   definition.setPreset(null); // Preserve joint angles from before
                                                                }
                                                             });
      jointSpaceControlWrapper = new ImBooleanWrapper(definition::getJointspaceOnly,
                                                      definition::setJointspaceOnly,
                                                      imBoolean -> {
                                                         if (ImGui.radioButton(labels.get("Hybrid"), !imBoolean.get()))
                                                            imBoolean.set(false);
                                                         ImGui.sameLine();
                                                         if (ImGui.radioButton(labels.get("Jointspace Only"), imBoolean.get()))
                                                            imBoolean.set(true);
                                                      });
      ImGuiLabelledWidgetAligner widgetAligner = new ImGuiLabelledWidgetAligner();
      linearPositionWeightWidget = new ImGuiSliderDoubleWrapper("Linear Position Weight", "%.2f", 0.0, 100.0,
                                                                definition::getLinearPositionWeight,
                                                                definition::setLinearPositionWeight);
      linearPositionWeightWidget.addButton("Use Default Weights", () -> definition.setLinearPositionWeight(-1.0));
      linearPositionWeightWidget.addWidgetAligner(widgetAligner);
      angularPositionWeightWidget = new ImGuiSliderDoubleWrapper("Angular Position Weight", "%.2f", 0.0, 100.0,
                                                                 definition::getAngularPositionWeight,
                                                                 definition::setAngularPositionWeight);
      angularPositionWeightWidget.addButton("Use Default Weights", () -> definition.setAngularPositionWeight(-1.0));
      angularPositionWeightWidget.addWidgetAligner(widgetAligner);
      jointspaceWeightWidget = new ImGuiSliderDoubleWrapper("Jointspace Weight", "%.2f", 0.0, 70.0,
                                                            definition::getJointspaceWeight,
                                                            definition::setJointspaceWeight);
      jointspaceWeightWidget.addButton("Use Default Weights", () -> definition.setJointspaceWeight(-1.0));
      jointspaceWeightWidget.addWidgetAligner(widgetAligner);
      positionErrorToleranceInput = new ImDoubleWrapper(definition::getPositionErrorTolerance,
                                                        definition::setPositionErrorTolerance,
                                                        imDouble -> ImGui.inputDouble(labels.get("Position Error Tolerance"), imDouble));
      orientationErrorToleranceDegreesInput = new ImDoubleWrapper(
            () -> Math.toDegrees(definition.getOrientationErrorTolerance()),
            orientationErrorToleranceDegrees -> definition.setOrientationErrorTolerance(Math.toRadians(orientationErrorToleranceDegrees)),
            imDouble -> ImGui.inputDouble(labels.get("Orientation Error Tolerance (%s)".formatted(EuclidCoreMissingTools.DEGREE_SYMBOL)), imDouble));

      FullHumanoidRobotModel syncedFullRobotModel = syncedRobot.getFullRobotModel();
      for (RobotSide side : RobotSide.values)
      {
         handGraphicToControlFrameTransforms.put(side, HandTransformTools.getHandGraphicToControlFrameTransform(syncedFullRobotModel,
                                                                                                                robotModel.getUIParameters(),
                                                                                                                side));
         String handBodyName = syncedFullRobotModel.getHand(side).getName();
         String modelFileName = RDXInteractableTools.getModelFileName(robotModel.getRobotDefinition().getRigidBodyDefinition(handBodyName));
         highlightModels.put(side, new RDXInteractableHighlightModel(modelFileName));

         MultiBodySystemBasics handOnlySystem = MultiBodySystemMissingTools.createSingleBodySystem(syncedFullRobotModel.getHand(side));
         List<Collidable> handCollidables = selectionCollisionModel.getRobotCollidables(handOnlySystem);

         RigidBodyTransformReadOnly linkToControlFrameTransform = HandTransformTools.getHandLinkToControlFrameTransform(syncedFullRobotModel, side);
         collisionShapeFrame.update(transformToParent -> transformToParent.set(linkToControlFrameTransform));

         for (Collidable handCollidable : handCollidables)
         {
            mouseCollidables.add(new MouseCollidable(handCollidable));
         }

         armMultiBodyGraphics.put(side, new RDXArmMultiBodyGraphic(robotModel, syncedFullRobotModel, side));
      }

      parentFrameComboBox = new ImGuiReferenceFrameLibraryCombo("Parent frame",
                                                                referenceFrameLibrary,
                                                                definition::getPalmParentFrameName,
                                                                getState().getPalmFrame()::changeFrame);

      tooltip = new RDX3DPanelTooltip(panel3D);
      panel3D.addImGuiOverlayAddition(this::render3DPanelImGuiOverlays);
   }

   @Override
   public void update()
   {
      super.update();

      // IK solution visualization via ghost arms
      if (state.getIsNextForExecution())
         visualizeIK();

      if (definition.getUsePredefinedJointAngles())
      {
         poseGizmo.setSelected(false);

         PresetArmConfiguration preset = getDefinition().getPreset();
         currentConfiguration.set(preset == null ? 0 : preset.ordinal() + 1);

         // Copy the preset values into the custom data fields so they can be tweaked
         // relatively when switching to custom angles.
         if (preset != null)
         {
            // TODO: Would be great if there was a #getPresetArmConfiguration that accepts an array to pack
            double[] jointAngles = syncedRobot.getRobotModel().getPresetArmConfiguration(getDefinition().getSide(), preset);
            for (int i = 0; i < jointAngles.length; i++)
            {
               getDefinition().getJointAngles().getValue()[i] = jointAngles[i];
            }
         }

         armMultiBodyGraphics.get(definition.getSide()).getHandControlFrame().getTransformToDesiredFrame(definition.getPalmTransformToParent().getValue(),
                                                                                                         ReferenceFrame.getWorldFrame());
      }
      else if (state.getPalmFrame().isChildOfWorld())
      {
         for (int i = 0; i < state.getJointAngles().getLength(); i++)
         {
            definition.getJointAngles().getValue()[i] = state.getJointAngles().getValueReadOnly(i);
         }

         if (poseGizmo.getPoseGizmo().getGizmoFrame() != state.getPalmFrame().getReferenceFrame())
         {
            poseGizmo.getPoseGizmo().setGizmoFrame(state.getPalmFrame().getReferenceFrame());
            graphicFrame.setParentFrame(state.getPalmFrame().getReferenceFrame());
            collisionShapeFrame.setParentFrame(state.getPalmFrame().getReferenceFrame());
         }

         graphicFrame.update(transformToParent -> transformToParent.set(handGraphicToControlFrameTransforms.get(definition.getSide())));

         poseGizmo.getPoseGizmo().update();
         highlightModels.get(definition.getSide()).setPose(graphicFrame.getReferenceFrame());

         if (poseGizmo.isSelected() || isMouseHovering)
         {
            highlightModels.get(definition.getSide()).setTransparency(0.7);
         }
         else
         {
            highlightModels.get(definition.getSide()).setTransparency(0.5);
         }

         ActionSequenceState actionSequence = BehaviorTreeTools.findActionSequenceAncestor(state);
         if (actionSequence != null)
         {
            HandPoseActionState previousHandAction = actionSequence.findNextPreviousAction(HandPoseActionState.class,
                                                                                           getState().getActionIndex(),
                                                                                           definition.getSide());

            boolean previousHandActionExists = previousHandAction != null;
            boolean weAreAfterIt = previousHandActionExists && actionSequence.getExecutionNextIndex() > previousHandAction.getActionIndex();

            boolean previousIsExecuting = previousHandActionExists && previousHandAction.getIsExecuting();
            boolean showFromPreviousHand = previousHandActionExists;
            boolean showFromCurrentHand = !showFromPreviousHand && state.getIsNextForExecution() && ! previousIsExecuting;

            ReferenceFrame fromFrame = null;
            if (showFromPreviousHand)
            {
               fromFrame = previousHandAction.getPalmFrame().getReferenceFrame();
            }
            if (showFromCurrentHand)
            {
               fromFrame = syncedRobot.getReferenceFrames().getHandFrame(definition.getSide());
            }

            if (fromFrame != null)
            {
               double lineWidth = 0.01;
               trajectoryGraphic.update(lineWidth,
                                        fromFrame.getTransformToRoot(),
                                        getState().getPalmFrame().getReferenceFrame().getTransformToRoot());
            }
            else
            {
               trajectoryGraphic.clear();
            }
         }
      }
   }

   private void visualizeIK()
   {
      RDXArmMultiBodyGraphic armMultiBodyGraphic = armMultiBodyGraphics.get(definition.getSide());
      armMultiBodyGraphic.getFloatingJoint().getJointPose().set(state.getGoalChestToWorldTransform().getValueReadOnly());
      for (int i = 0; i < armMultiBodyGraphic.getJoints().length; i++)
      {
         armMultiBodyGraphic.getJoints()[i].setQ(state.getJointAngles().getValueReadOnly(i));
      }
      armMultiBodyGraphic.updateAfterModifyingConfiguration();
      armMultiBodyGraphic.setColor(RDXIKSolverColors.getColor(state.getSolutionQuality()));
   }

   @Override
   public void renderTreeViewIconArea()
   {
      super.renderTreeViewIconArea();

      boolean gizmoWasSelected = poseGizmo.getSelected().get();
      if (handIconWidget.render(definition.getSide(), ImGui.getFrameHeight(), gizmoWasSelected))
      {
         poseGizmo.setSelected(!gizmoWasSelected);
      }

      ImGui.sameLine();
   }

   @Override
   protected void renderImGuiWidgetsInternal()
   {
      trajectoryDurationWidget.renderImGuiWidget();
      usePredefinedJointAnglesWrapper.renderImGuiWidget();

      if (definition.getUsePredefinedJointAngles())
      {
         ImGui.pushItemWidth(200.0f);
         if (ImGui.combo(labels.get("Configuration"), currentConfiguration, configurations))
            getDefinition().setPreset(currentConfiguration.get() == 0 ? null : PresetArmConfiguration.values()[currentConfiguration.get() - 1]);
         ImGui.popItemWidth();

         if (getDefinition().getPreset() == null)
         {
            ArmJointName[] armJointNames = syncedRobot.getRobotModel().getJointMap().getArmJointNames(getDefinition().getSide());
            ImGui.pushItemWidth(80.0f);
            for (int i = 0; i < armJointNames.length; i++)
            {
               jointAngleWidgets[i].renderImGuiWidget();
            }
            ImGui.popItemWidth();
            if (ImGui.button(labels.get("Set Configuration to Synced Arm")))
            {
               for (int i = 0; i < getDefinition().getJointAngles().getLength(); i++)
               {
                  OneDoFJointBasics syncedJoint = syncedRobot.getFullRobotModel().getArmJoint(getDefinition().getSide(), armJointNames[i]);
                  if (syncedJoint != null)
                     getDefinition().getJointAngles().getValue()[i] = syncedJoint.getQ();
                  else
                     getDefinition().getJointAngles().getValue()[i] = 0.0;
               }
            }
         }
      }
      else
      {
         ImGui.checkbox(labels.get("Adjust Goal Pose"), poseGizmo.getSelected());
         jointSpaceControlWrapper.renderImGuiWidget();
         if (!definition.getJointspaceOnly())
         {
            ImGui.sameLine();
            holdPoseInWorldLaterWrapper.renderImGuiWidget();
         }
         parentFrameComboBox.render();
         if (definition.getJointspaceOnly())
            ImGui.beginDisabled();
         linearPositionWeightWidget.renderImGuiWidget();
         angularPositionWeightWidget.renderImGuiWidget();
         if (definition.getJointspaceOnly())
            ImGui.endDisabled();
         jointspaceWeightWidget.renderImGuiWidget();
         positionErrorToleranceInput.renderImGuiWidget();
         orientationErrorToleranceDegreesInput.renderImGuiWidget();
         ImGui.text("IK Solution Quality: %.2f".formatted(state.getSolutionQuality()));
         ImGui.sameLine();
         if (ImGui.button(labels.get("Set Pose to Synced Hand")))
         {
            CRDTDetachableReferenceFrame actionPalmFrame = getState().getPalmFrame();
            CRDTUnidirectionalRigidBodyTransform palmTransformToParent = definition.getPalmTransformToParent();
            MovingReferenceFrame syncedPalmFrame = syncedRobot.getReferenceFrames().getHandFrame(definition.getSide());
            FramePose3D syncedPalmPose = new FramePose3D();
            syncedPalmPose.setToZero(syncedPalmFrame);
            syncedPalmPose.changeFrame(actionPalmFrame.getReferenceFrame().getParent());
            palmTransformToParent.getValue().set(syncedPalmPose);
            actionPalmFrame.update();
         }
      }
   }

   @Override
   public void deselectGizmos()
   {
      poseGizmo.setSelected(false);
   }

   public void render3DPanelImGuiOverlays()
   {
      if (isMouseHovering)
      {
         tooltip.render("%s Action\nIndex: %d\nName: %s".formatted(getActionTypeTitle(), state.getActionIndex(), definition.getName()));
      }
   }

   @Override
   public void calculate3DViewPick(ImGui3DViewInput input)
   {
      if (state.getPalmFrame().isChildOfWorld())
      {
         poseGizmo.calculate3DViewPick(input);

         if (!definition.getUsePredefinedJointAngles())
         {
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
      }
   }

   @Override
   public void process3DViewInput(ImGui3DViewInput input)
   {
      if (state.getPalmFrame().isChildOfWorld())
      {
         isMouseHovering = input.getClosestPick() == pickResult;

         boolean isClickedOn = isMouseHovering && input.mouseReleasedWithoutDrag(ImGuiMouseButton.Left);
         if (isClickedOn)
         {
            poseGizmo.setSelected(true);
         }

         poseGizmo.process3DViewInput(input, isMouseHovering);

         tooltip.setInput(input);
      }
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (state.getPalmFrame().isChildOfWorld())
      {
         if (!definition.getUsePredefinedJointAngles() && (getSelected() || poseGizmo.isSelected() || handIconWidget.getIsHovered()))
            highlightModels.get(definition.getSide()).getRenderables(renderables, pool);
         poseGizmo.getVirtualRenderables(renderables, pool);

         if (state.getIsNextForExecution())
            armMultiBodyGraphics.get(definition.getSide()).getRootBody().getVisualRenderables(renderables, pool);

         trajectoryGraphic.getRenderables(renderables, pool);
      }
   }

   @Override
   public String getActionTypeTitle()
   {
      return definition.getSide().getPascalCaseName() + " Hand Pose";
   }

   public ReferenceFrame getReferenceFrame()
   {
      return poseGizmo.getPoseGizmo().getGizmoFrame();
   }
}
