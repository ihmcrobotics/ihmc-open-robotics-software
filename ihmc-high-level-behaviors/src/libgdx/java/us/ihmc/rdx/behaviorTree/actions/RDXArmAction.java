package us.ihmc.rdx.behaviorTree.actions;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.flag.ImGuiMouseButton;
import imgui.type.ImInt;
import org.apache.commons.lang3.mutable.MutableObject;
import us.ihmc.avatar.arm.PresetArmConfiguration;
import us.ihmc.avatar.inverseKinematics.ArmIKSolver;
import us.ihmc.behaviors.behaviorTree.action.actions.AbilityHandActionState;
import us.ihmc.behaviors.behaviorTree.action.actions.ArmActionDefinition;
import us.ihmc.behaviors.behaviorTree.action.actions.ArmActionState;
import us.ihmc.behaviors.behaviorTree.action.actions.ArmActionTaskspaceTrajectoryMode;
import us.ihmc.commons.thread.Throttler;
import us.ihmc.communication.crdt.CRDTBidirectionalRigidBodyTransform;
import us.ihmc.communication.crdt.CRDTDetachableReferenceFrame;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.handsros2.abilityHand.AbilityHandControlMode;
import us.ihmc.handsros2.abilityHand.AbilityHandGrip;
import us.ihmc.handsros2.abilityHand.AbilityHandModel.AbilityHandJointName;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.RevoluteJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.rdx.behaviorTree.RDXBehaviorTreeRootNode;
import us.ihmc.rdx.behaviorTree.RDXCRDTTools;
import us.ihmc.rdx.imgui.*;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.input.ImGui3DViewPickResult;
import us.ihmc.rdx.simulation.scs2.RDXRigidBody;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.RDX3DPanelTooltip;
import us.ihmc.rdx.ui.affordances.RDXInteractableHighlightModel;
import us.ihmc.rdx.ui.affordances.RDXInteractableTools;
import us.ihmc.rdx.ui.gizmo.RDXSelectablePose3DGizmo;
import us.ihmc.rdx.ui.graphics.RDXArmMultiBodyGraphic;
import us.ihmc.rdx.ui.teleoperation.RDXIKSolverColors;
import us.ihmc.rdx.ui.widgets.ImGuiArmIconWidget;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.EuclidCoreMissingTools;
import us.ihmc.robotics.MultiBodySystemMissingTools;
import us.ihmc.robotics.interaction.MouseCollidable;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.referenceFrames.MutableReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.scs2.simulation.collision.Collidable;
import us.ihmc.wholeBodyController.HandTransformTools;

import java.util.ArrayList;
import java.util.List;

import static us.ihmc.behaviors.behaviorTree.action.actions.ArmActionDefinition.MAX_NUMBER_OF_JOINTS;

public class RDXArmAction extends RDXActionNode<ArmActionState, ArmActionDefinition>
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImGuiArmIconWidget armIconWidget = new ImGuiArmIconWidget();
   /** Gizmo is control frame */
   private final RDXSelectablePose3DGizmo poseGizmo;
   private final RDXScrewPrimitive screwPrimitive;
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
   private final ImDoubleWrapper[] jointAngleWidgets = new ImDoubleWrapper[MAX_NUMBER_OF_JOINTS];
   private final SideDependentList<ArmJointName[]> jointNames = new SideDependentList<>();
   private final SideDependentList<double[]> jointLowerLimits = new SideDependentList<>();
   private final SideDependentList<double[]> jointUpperLimits = new SideDependentList<>();
   private final ImGuiLabelledWidgetAligner widgetAligner = new ImGuiLabelledWidgetAligner();
   private final ImGuiSliderDoubleWrapper linearPositionWeightWidget;
   private final ImGuiSliderDoubleWrapper angularPositionWeightWidget;
   private final ImGuiSliderDoubleWrapper jointspaceWeightWidget;
   private final ImBooleanWrapper holdPoseInWorldLaterWrapper;
   private final ImBooleanWrapper jointSpaceControlWrapper;
   private final ImBooleanWrapper definedInJointspaceWrapper;
   private final ImDoubleWrapper positionErrorToleranceInput;
   private final ImDoubleWrapper orientationErrorToleranceDegreesInput;
   private final SideDependentList<RDXArmMultiBodyGraphic> armMultiBodyGraphics = new SideDependentList<>();
   private final SideDependentList<RDXRigidBody> abilityHands = new SideDependentList<>();
   private final double[] fingerPositions = new double[6];
   private boolean showAbilityHand = false;
   private final Throttler lowQualityRenderThrottler = new Throttler();
   private final RDX3DPanelTooltip tooltip;

   public RDXArmAction(long id, RDXBehaviorTreeRootNode rootNode)
   {
      super(new ArmActionState(id, rootNode.getState()), rootNode);

      poseGizmo = new RDXSelectablePose3DGizmo();
      poseGizmo.create(panel3D);
      screwPrimitive = new RDXScrewPrimitive(this);

      trajectoryDurationWidget = new ImDoubleWrapper(definition::getTrajectoryDuration,
                                                     definition::setTrajectoryDuration,
                                                     imDouble -> ImGui.inputDouble(labels.get("Trajectory duration"), imDouble));
      int configurationIndex = 0;
      configurations[configurationIndex++] = ArmActionDefinition.CUSTOM_ANGLES_NAME;
      for (PresetArmConfiguration preset : PresetArmConfiguration.values())
         configurations[configurationIndex++] = preset.name();

      for (RobotSide side : RobotSide.values)
      {
         ArmJointName[] armJointNames = robotModel.getJointMap().getArmJointNames(side);

         jointNames.put(side, armJointNames);
         jointLowerLimits.put(side, new double[armJointNames.length]);
         jointUpperLimits.put(side, new double[armJointNames.length]);

         int jointIndex = 0;
         for (ArmJointName armJointName : armJointNames)
         {
            OneDoFJointBasics armJoint = syncedRobot.getFullRobotModel().getArmJoint(side, armJointName);
            jointLowerLimits.get(side)[jointIndex] = armJoint.getJointLimitLower();
            jointUpperLimits.get(side)[jointIndex] = armJoint.getJointLimitUpper();
            ++jointIndex;
         }
      }

      for (int i = 0; i < MAX_NUMBER_OF_JOINTS; i++)
      {
         int jointIndex = i;
         final MutableObject<ImGuiInputDouble> fancyInput = new MutableObject<>();
         final MutableObject<ImGuiSliderDouble> fancySlider = new MutableObject<>();
         jointAngleWidgets[i] = new ImDoubleWrapper(() -> definition.getJointAngles().getValueReadOnly(jointIndex),
                                                    jointAngle -> definition.getJointAngles().setValue(jointIndex, jointAngle),
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
            fancySlider.getValue().setWidgetText("%s %.1f%s".formatted(jointNames.get(definition.getSide())[jointIndex].name(),
                                                                       Math.toDegrees(imDouble.get()),
                                                                       EuclidCoreMissingTools.DEGREE_SYMBOL));
            fancySlider.getValue().render(jointLowerLimits.get(definition.getSide())[jointIndex], jointUpperLimits.get(definition.getSide())[jointIndex]);
         });
      }
      holdPoseInWorldLaterWrapper = new ImBooleanWrapper(definition::getHoldPoseInWorldLater,
                                                         definition::setHoldPoseInWorldLater,
                                                         imBoolean -> ImGui.checkbox(labels.get("Hold pose in world later"), imBoolean));
      definedInJointspaceWrapper = new ImBooleanWrapper(definition::getDefinedInJointspace,
                                                        definition::setDefinedInJointspace,
                                                        imBoolean ->
      {
         ImGui.text("Definition:");
         ImGui.sameLine();
         if (ImGui.radioButton(labels.get("Jointspace"), imBoolean.get()) && !imBoolean.get())
         {
            imBoolean.set(true);
            definition.setPreset(null);
            for (int i = 0; i < state.getPreviewJointAngles().getLength(); i++)
               definition.getJointAngles().setValue(i, state.getPreviewJointAngles().getValueReadOnly(i));
         }
         ImGui.sameLine();
         if (ImGui.radioButton(labels.get("Taskspace"), !imBoolean.get()) && imBoolean.get())
         {
            imBoolean.set(false);
            if (definition.getTaskspaceTrajectoryMode() == ArmActionTaskspaceTrajectoryMode.SINGLE_POSE
                && state.getPalmFrame().isChildOfWorld())
            {
               CRDTDetachableReferenceFrame actionPalmFrame = state.getPalmFrame();
               CRDTBidirectionalRigidBodyTransform palmTransformToParent = definition.getPalmTransformToParent();
               ReferenceFrame previewPalmFrame = armMultiBodyGraphics.get(definition.getSide()).getHandControlFrame();
               FramePose3D previewPalmPose = new FramePose3D();
               previewPalmPose.setToZero(previewPalmFrame);
               previewPalmPose.changeFrame(actionPalmFrame.getReferenceFrame().getParent());
               palmTransformToParent.getValueAndModify().set(previewPalmPose);
               actionPalmFrame.update();
            }
         }
      });
      jointSpaceControlWrapper = new ImBooleanWrapper(definition::getJointspaceOnly,
                                                      definition::setJointspaceOnly,
                                                      imBoolean ->
      {
         ImGui.text("Control mode:");
         ImGui.sameLine();
         if (ImGui.radioButton(labels.get("Hybrid"), !imBoolean.get()))
            imBoolean.set(false);
         ImGui.sameLine();
         if (ImGui.radioButton(labels.get("Jointspace Only"), imBoolean.get()))
            imBoolean.set(true);
      });
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
      positionErrorToleranceInput = new ImDoubleWrapper(() ->
                                                        {
                                                           if (definition.getDefinedInJointspace())
                                                              return Math.toDegrees(definition.getPositionErrorTolerance());
                                                           else
                                                              return definition.getPositionErrorTolerance();
                                                        },
                                                        positionErrorTolerance ->
                                                        {
                                                           if (definition.getDefinedInJointspace())
                                                              definition.setPositionErrorTolerance(Math.toRadians(positionErrorTolerance));
                                                           else
                                                              definition.setPositionErrorTolerance(positionErrorTolerance);
                                                        },
                                                        imDouble ->
                                                        {
                                                           if (definition.getDefinedInJointspace())
                                                              ImGui.inputDouble(labels.get("Position Error Tolerance (%s)".formatted(EuclidCoreMissingTools.DEGREE_SYMBOL)), imDouble);
                                                           else
                                                              ImGui.inputDouble(labels.get("Position Error Tolerance"), imDouble);
                                                        });
      orientationErrorToleranceDegreesInput = new ImDoubleWrapper(
            () -> Math.toDegrees(definition.getOrientationErrorTolerance()),
            orientationErrorToleranceDegrees -> definition.setOrientationErrorTolerance(Math.toRadians(orientationErrorToleranceDegrees)),
            imDouble -> ImGui.inputDouble(labels.get("Orientation Error Tolerance (%s)".formatted(EuclidCoreMissingTools.DEGREE_SYMBOL)), imDouble));

      FullHumanoidRobotModel syncedFullRobotModel = syncedRobot.getFullRobotModel();
      for (RobotSide side : RobotSide.values)
      {
         handGraphicToControlFrameTransforms.put(side, HandTransformTools.getHandGraphicToControlFrameTransform(syncedFullRobotModel,
                                                                                                                robotModel.getHandGraphicToHandFrameTransform(side),
                                                                                                                side));
         String handBodyName = syncedFullRobotModel.getHand(side).getName();
         String modelFileName = RDXInteractableTools.getModelFileName(robotModel.getRobotDefinition().getRigidBodyDefinition(handBodyName));
         highlightModels.put(side, new RDXInteractableHighlightModel(modelFileName));

         MultiBodySystemBasics handOnlySystem = MultiBodySystemMissingTools.createSingleBodySystem(syncedFullRobotModel.getHand(side));
         List<Collidable> handCollidables = selectionCollisionModel.getRobotCollidables(handOnlySystem);

         RigidBodyTransformReadOnly linkToControlFrameTransform = HandTransformTools.getHandLinkToControlFrameTransform(syncedFullRobotModel, side);
         collisionShapeFrame.update(transformToParent -> transformToParent.set(linkToControlFrameTransform));

         for (Collidable handCollidable : handCollidables)
            mouseCollidables.add(new MouseCollidable(handCollidable));

         armMultiBodyGraphics.put(side, new RDXArmMultiBodyGraphic(robotModel, syncedFullRobotModel, side));

         if (robotModel.getRobotVersion().hasSakeGripperJoints(side))
         {
            // TODO: Sake gripper
         }
         else if (robotModel.getRobotVersion().hasHandWithFingers(side))
         {
            abilityHands.put(side, RDXInteractableTools.loadAbilityHand(robotModel.getRobotDefinition(), side));
            abilityHands.get(side).setOpacityRecursive(0.5f);
         }
      }

      parentFrameComboBox = new ImGuiReferenceFrameLibraryCombo("Parent frame",
                                                                scene::getAllFrameNames,
                                                                definition::getPalmParentFrameName,
                                                                state.getPalmFrame()::changeFrame);

      tooltip = new RDX3DPanelTooltip(panel3D);
      panel3D.addImGuiOverlayAddition(this::render3DPanelImGuiOverlays);
   }

   @Override
   public void update()
   {
      super.update();

      showAbilityHand = false;
      if (state.getIsNextForExecution())
         visualizeIK();

      if (definition.getDefinedInJointspace())
      {
         poseGizmo.setSelected(false);
         screwPrimitive.deselectGizmos();

         PresetArmConfiguration preset = definition.getPreset();
         currentConfiguration.set(preset == null ? 0 : preset.ordinal() + 1);

         if (preset != null)
         {
            double[] jointAngles = syncedRobot.getRobotModel().getPresetArmConfiguration(definition.getSide(), preset);
            for (int i = 0; i < jointAngles.length; i++)
               definition.getJointAngles().setValue(i, jointAngles[i]);
         }
      }
      else if (isSinglePoseTaskspace() && state.getPalmFrame().isChildOfWorld())
      {
         screwPrimitive.deselectGizmos();

         if (poseGizmo.getPoseGizmo().getGizmoFrame() != state.getPalmFrame().getReferenceFrame())
         {
            poseGizmo.getPoseGizmo().setGizmoFrame(state.getPalmFrame().getReferenceFrame());
            graphicFrame.setParentFrame(state.getPalmFrame().getReferenceFrame());
            collisionShapeFrame.setParentFrame(state.getPalmFrame().getReferenceFrame());
         }

         RDXCRDTTools.syncGizmoWithBidirectionalField(poseGizmo.getPoseGizmo(), definition.getPalmTransformToParent(), definition);

         graphicFrame.update(transformToParent -> transformToParent.set(handGraphicToControlFrameTransforms.get(definition.getSide())));
         highlightModels.get(definition.getSide()).setPose(graphicFrame.getReferenceFrame());

         if (poseGizmo.isSelected() || isMouseHovering)
            highlightModels.get(definition.getSide()).setTransparency(0.7);
         else
            highlightModels.get(definition.getSide()).setTransparency(0.5);
      }
      else if (isScrewTaskspace())
      {
         poseGizmo.setSelected(false);
         screwPrimitive.update();
      }
   }

   private void visualizeIK()
   {
      RDXArmMultiBodyGraphic armMultiBodyGraphic = armMultiBodyGraphics.get(definition.getSide());

      if (isScrewTaskspace())
      {
         screwPrimitive.visualizeIK(armMultiBodyGraphic);
      }
      else
      {
         double solutionQuality = state.getSolutionQuality();
         if (solutionQuality < ArmIKSolver.GOOD_QUALITY_MAX || lowQualityRenderThrottler.run(0.5))
         {
            armMultiBodyGraphic.getFloatingJoint().getJointPose().set(state.getGoalChestToWorldTransform().getValueReadOnly());
            for (int i = 0; i < armMultiBodyGraphic.getJoints().length; i++)
               armMultiBodyGraphic.getJoints()[i].setQ(state.getPreviewJointAngles().getValueReadOnly(i));
            armMultiBodyGraphic.updateAfterModifyingConfiguration();
         }
         armMultiBodyGraphic.setColor(RDXIKSolverColors.getColor(solutionQuality));
      }

      if (abilityHands.containsKey(definition.getSide()))
      {
         RDXRigidBody abilityHand = abilityHands.get(definition.getSide());
         RigidBodyBasics palm = abilityHand.getChildrenJoints().get(0).getSuccessor();
         for (int i = state.getLeafIndex() - 1; i >= 0; i--)
         {
            if (rootNode.getState().getOrderedLeaves().get(i) instanceof AbilityHandActionState abilityHandActionState
                && abilityHandActionState.getDefinition().getSide() == definition.getSide())
            {
               if (abilityHandActionState.getDefinition().getControlMode() == AbilityHandControlMode.GRIP)
               {
                  AbilityHandGrip grip = abilityHandActionState.getDefinition().getGrip();
                  for (int s = 0; s < grip.getNumberOfStages(); s++)
                     for (int j = 0; j < grip.getFingersInStage(s); j++)
                        fingerPositions[grip.getStageFingerIndex(s, j)] = grip.getStageFingerPosition(s, j);
                  showAbilityHand = true;
               }
               else if (abilityHandActionState.getDefinition().getControlMode() == AbilityHandControlMode.POSITION)
               {
                  for (int f = 0; f < 6; f++)
                     fingerPositions[f] = abilityHandActionState.getDefinition().getGoalPositions().getValueReadOnly(f);
                  showAbilityHand = true;
               }
               break;
            }
         }
         if (showAbilityHand)
         {
            abilityHand.getFloatingJoint().getJointPose().set(armMultiBodyGraphic.getHand().getParentJoint().getFrameAfterJoint().getTransformToWorldFrame());
            for (int i = 0; i < 6; i++)
            {
               RevoluteJoint joint;
               if (i == 4)
                  joint = (RevoluteJoint) palm.getChildrenJoints().get(4).getSuccessor().getChildrenJoints().get(0);
               else
                  joint = (RevoluteJoint) palm.getChildrenJoints().get(i == 5 ? 4 : i);
               joint.setQ(Math.toRadians(fingerPositions[i]));
               if (i < 4)
                  ((RevoluteJoint) palm.getChildrenJoints().get(i).getSuccessor().getChildrenJoints().get(0))
                        .setQ(AbilityHandJointName.getQ2Position(Math.toRadians(fingerPositions[i])));
            }
            abilityHand.update();
         }
      }
   }

   @Override
   public void renderTreeViewRow()
   {
      super.renderRowBeginning();
      super.renderEditableName();

      ImGui.sameLine();
      if (isScrewTaskspace())
         ImGui.textDisabled("Screw");
      else
      {
         boolean gizmoWasSelected = poseGizmo.getSelected().get();
         if (armIconWidget.render(definition.getSide(), gizmoWasSelected, definition.getDefinedInJointspace()))
            poseGizmo.setSelected(!gizmoWasSelected);
      }

      renderRowEnd();
   }

   @Override
   protected void renderImGuiWidgetsInternal()
   {
      definedInJointspaceWrapper.renderImGuiWidget();

      if (!definition.getDefinedInJointspace())
      {
         ImGui.text("Trajectory mode:");
         ImGui.sameLine();
         boolean singlePose = definition.getTaskspaceTrajectoryMode() == ArmActionTaskspaceTrajectoryMode.SINGLE_POSE;
         if (ImGui.radioButton(labels.get("Single point"), singlePose))
            definition.setTaskspaceTrajectoryMode(ArmActionTaskspaceTrajectoryMode.SINGLE_POSE);
         ImGui.sameLine();
         if (ImGui.radioButton(labels.get("Screw primitive"), !singlePose))
            definition.setTaskspaceTrajectoryMode(ArmActionTaskspaceTrajectoryMode.SCREW_PRIMITIVE);
      }

      if (definition.getDefinedInJointspace() || isSinglePoseTaskspace())
         trajectoryDurationWidget.renderImGuiWidget();

      if (definition.getDefinedInJointspace())
      {
         ImGui.pushItemWidth(200.0f);
         if (ImGui.combo(labels.get("Configuration"), currentConfiguration, configurations))
            definition.setPreset(currentConfiguration.get() == 0 ? null : PresetArmConfiguration.values()[currentConfiguration.get() - 1]);
         ImGui.popItemWidth();

         if (definition.getPreset() == null)
         {
            ArmJointName[] armJointNames = syncedRobot.getRobotModel().getJointMap().getArmJointNames(definition.getSide());
            ImGui.pushItemWidth(80.0f);
            for (int i = 0; i < armJointNames.length; i++)
               jointAngleWidgets[i].renderImGuiWidget();
            ImGui.popItemWidth();
            if (ImGui.button(labels.get("Set Configuration to Synced Arm")))
            {
               for (int i = 0; i < armJointNames.length; i++)
               {
                  OneDoFJointBasics syncedJoint = syncedRobot.getFullRobotModel().getArmJoint(definition.getSide(), armJointNames[i]);
                  if (syncedJoint != null)
                     definition.getJointAngles().setValue(i, syncedJoint.getQ());
                  else
                     definition.getJointAngles().setValue(i, 0.0);
               }
            }
         }
         jointspaceWeightWidget.renderImGuiWidget();
         ImGui.pushItemWidth(ImGui.getFontSize() * 10.0f);
         positionErrorToleranceInput.renderImGuiWidget();
         ImGui.popItemWidth();
      }
      else if (isSinglePoseTaskspace())
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
         ImGui.pushItemWidth(ImGui.getFontSize() * 10.0f);
         positionErrorToleranceInput.renderImGuiWidget();
         orientationErrorToleranceDegreesInput.renderImGuiWidget();
         ImGui.popItemWidth();
         ImGui.text("IK Solution Quality: %.2f".formatted(state.getSolutionQuality()));
         ImGui.sameLine();
         if (ImGui.button(labels.get("Set Pose to Synced Hand")))
         {
            CRDTDetachableReferenceFrame actionPalmFrame = state.getPalmFrame();
            CRDTBidirectionalRigidBodyTransform palmTransformToParent = definition.getPalmTransformToParent();
            MovingReferenceFrame syncedPalmFrame = syncedRobot.getReferenceFrames().getHandFrame(definition.getSide());
            FramePose3D syncedPalmPose = new FramePose3D();
            syncedPalmPose.setToZero(syncedPalmFrame);
            syncedPalmPose.changeFrame(actionPalmFrame.getReferenceFrame().getParent());
            palmTransformToParent.getValueAndModify().set(syncedPalmPose);
            actionPalmFrame.update();
         }
      }
      else if (isScrewTaskspace())
      {
         screwPrimitive.renderImGuiWidgets();
      }
   }

   @Override
   public void deselectGizmos()
   {
      poseGizmo.setSelected(false);
      screwPrimitive.deselectGizmos();
   }

   public void render3DPanelImGuiOverlays()
   {
      if (isMouseHovering)
         tooltip.render("%s Action\nIndex: %d\nName: %s".formatted(getLeafTypeTitle(), state.getLeafIndex(), definition.getName()));
   }

   @Override
   public void calculate3DViewPick(ImGui3DViewInput input)
   {
      if (!getSelected())
         return;

      if (isSinglePoseTaskspace() && state.getPalmFrame().isChildOfWorld())
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
      else if (isScrewTaskspace())
      {
         screwPrimitive.calculate3DViewPick(input);
      }
   }

   @Override
   public void process3DViewInput(ImGui3DViewInput input)
   {
      if (!getSelected())
         return;

      if (isSinglePoseTaskspace() && state.getPalmFrame().isChildOfWorld())
      {
         isMouseHovering = input.getClosestPick() == pickResult;

         boolean isClickedOn = isMouseHovering && input.mouseReleasedWithoutDrag(ImGuiMouseButton.Left);
         if (isClickedOn)
            poseGizmo.setSelected(true);

         poseGizmo.process3DViewInput(input, isMouseHovering);
         tooltip.setInput(input);
      }
      else if (isScrewTaskspace())
      {
         screwPrimitive.process3DViewInput(input);
      }
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (!(state.getIsNextForExecution() || getSelected() || state.getIsExecuting()))
         return;

      if (isSinglePoseTaskspace() && state.getPalmFrame().isChildOfWorld())
      {
         if (getSelected() || poseGizmo.isSelected() || armIconWidget.getIsHovered())
            highlightModels.get(definition.getSide()).getRenderables(renderables, pool);
         poseGizmo.getVirtualRenderables(renderables, pool);
      }
      else if (isScrewTaskspace())
      {
         screwPrimitive.getRenderables(renderables, pool);
      }

      if (state.getIsNextForExecution() || state.getIsExecuting())
      {
         armMultiBodyGraphics.get(definition.getSide()).getRootBody().getVisualRenderables(renderables, pool);
         if (showAbilityHand && abilityHands.containsKey(definition.getSide()))
            abilityHands.get(definition.getSide()).getVisualRenderables(renderables, pool);
      }
   }

   @Override
   public String getLeafTypeTitle()
   {
      if (isScrewTaskspace())
         return screwPrimitive.getLeafTypeTitle();
      return definition.getSide().getPascalCaseName() + " Arm";
   }

   private boolean isSinglePoseTaskspace()
   {
      return !definition.getDefinedInJointspace()
             && definition.getTaskspaceTrajectoryMode() == ArmActionTaskspaceTrajectoryMode.SINGLE_POSE;
   }

   private boolean isScrewTaskspace()
   {
      return !definition.getDefinedInJointspace()
             && definition.getTaskspaceTrajectoryMode() == ArmActionTaskspaceTrajectoryMode.SCREW_PRIMITIVE;
   }

   public ReferenceFrame getReferenceFrame()
   {
      return poseGizmo.getPoseGizmo().getGizmoFrame();
   }

   RDX3DPanel getPanel3D()
   {
      return panel3D;
   }

   ROS2SyncedRobotModel getSyncedRobot()
   {
      return syncedRobot;
   }

   ImGuiUniqueLabelMap getLabels()
   {
      return labels;
   }

   ImGuiLabelledWidgetAligner getWidgetAligner()
   {
      return widgetAligner;
   }

   ImGuiReferenceFrameLibraryCombo getParentFrameComboBox()
   {
      return parentFrameComboBox;
   }

   ImGuiSliderDoubleWrapper getLinearPositionWeightWidget()
   {
      return linearPositionWeightWidget;
   }

   ImGuiSliderDoubleWrapper getAngularPositionWeightWidget()
   {
      return angularPositionWeightWidget;
   }

   ImGuiSliderDoubleWrapper getJointspaceWeightWidget()
   {
      return jointspaceWeightWidget;
   }

   ImDoubleWrapper getPositionErrorToleranceInput()
   {
      return positionErrorToleranceInput;
   }

   ImDoubleWrapper getOrientationErrorToleranceDegreesInput()
   {
      return orientationErrorToleranceDegreesInput;
   }
}
