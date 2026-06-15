package us.ihmc.rdx.behaviorTree.actions;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.flag.ImGuiCol;
import imgui.flag.ImGuiMouseButton;
import imgui.type.ImInt;
import org.apache.commons.lang3.mutable.MutableObject;
import us.ihmc.avatar.arm.PresetArmConfiguration;
import us.ihmc.avatar.inverseKinematics.ArmIKSolver;
import us.ihmc.behaviors.behaviorTree.action.actions.AbilityHandActionState;
import us.ihmc.behaviors.behaviorTree.action.actions.ArmActionDefinition;
import us.ihmc.behaviors.behaviorTree.action.actions.ArmActionState;
import us.ihmc.behaviors.behaviorTree.action.actions.ArmActionTaskspaceTrajectoryMode;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.commons.thread.Throttler;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.communication.crdt.CRDTBidirectionalRigidBodyTransform;
import us.ihmc.communication.crdt.CRDTDetachableReferenceFrame;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.handsros2.abilityHand.AbilityHandControlMode;
import us.ihmc.handsros2.abilityHand.AbilityHandGrip;
import us.ihmc.handsros2.abilityHand.AbilityHandModel.AbilityHandJointName;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
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
import us.ihmc.rdx.mesh.RDXDashedLineMesh;
import us.ihmc.rdx.simulation.scs2.RDXRigidBody;
import us.ihmc.rdx.ui.RDX3DPanelTooltip;
import us.ihmc.rdx.ui.affordances.RDXInteractableHighlightModel;
import us.ihmc.rdx.ui.affordances.RDXInteractableTools;
import us.ihmc.rdx.ui.gizmo.RDXSelectablePose3DGizmo;
import us.ihmc.rdx.ui.graphics.RDXArmMultiBodyGraphic;
import us.ihmc.rdx.ui.graphics.RDXTrajectoryGraphic;
import us.ihmc.rdx.ui.teleoperation.RDXIKSolverColors;
import us.ihmc.rdx.ui.widgets.ImGuiArmIconWidget;
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
   private final RDXSelectablePose3DGizmo screwAxisGizmo;
   private final RDXDashedLineMesh screwAxisGraphic = new RDXDashedLineMesh(Color.WHITE, Axis3D.X, 0.04);
   private final RDXTrajectoryGraphic trajectoryGraphic = new RDXTrajectoryGraphic();
   private final RecyclingArrayList<FramePose3D> trajectoryPoses = new RecyclingArrayList<>(FramePose3D::new);
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
   private final ImGuiSliderDoubleWrapper linearPositionWeightWidget;
   private final ImGuiSliderDoubleWrapper angularPositionWeightWidget;
   private final ImGuiSliderDoubleWrapper jointspaceWeightWidget;
   private final ImBooleanWrapper holdPoseInWorldLaterWrapper;
   private final ImBooleanWrapper jointSpaceControlWrapper;
   private final ImBooleanWrapper definedInJointspaceWrapper;
   private final ImGuiSliderDoubleWrapper translationWidget;
   private final ImGuiSliderDoubleWrapper rotationWidget;
   private final ImGuiSliderDoubleWrapper maxLinearVelocityWidget;
   private final ImGuiSliderDoubleWrapper maxAngularVelocityWidget;
   private final ImGuiSliderDoubleWrapper previewTimeWidget;
   private final ImDoubleWrapper positionErrorToleranceInput;
   private final ImDoubleWrapper orientationErrorToleranceDegreesInput;
   private final SideDependentList<RDXArmMultiBodyGraphic> armMultiBodyGraphics = new SideDependentList<>();
   private final SideDependentList<RDXRigidBody> abilityHands = new SideDependentList<>();
   private final double[] fingerPositions = new double[6];
   private boolean showAbilityHand = false;
   private boolean playbackPreview = false;
   private final Stopwatch playbackStopwatch = new Stopwatch();
   private final Throttler lowQualityRenderThrottler = new Throttler();
   private final RDX3DPanelTooltip tooltip;

   public RDXArmAction(long id, RDXBehaviorTreeRootNode rootNode)
   {
      super(new ArmActionState(id, rootNode.getState()), rootNode);

      poseGizmo = new RDXSelectablePose3DGizmo();
      poseGizmo.create(panel3D);
      screwAxisGizmo = new RDXSelectablePose3DGizmo();
      screwAxisGizmo.create(panel3D);

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
      translationWidget = new ImGuiSliderDoubleWrapper("Translation", "%.2f", -0.4, 0.4, definition::getTranslation, definition::setTranslation);
      translationWidget.addWidgetAligner(widgetAligner);
      rotationWidget = new ImGuiSliderDoubleWrapper("Rotation", "%.2f", -2.0 * Math.PI, 2.0 * Math.PI,
                                                    definition::getRotation,
                                                    definition::setRotation);
      rotationWidget.addWidgetAligner(widgetAligner);
      maxLinearVelocityWidget = new ImGuiSliderDoubleWrapper("Max Linear Velocity", "%.2f", 0.05, 1.0,
                                                             definition::getMaxLinearVelocity,
                                                             definition::setMaxLinearVelocity);
      maxLinearVelocityWidget.addWidgetAligner(widgetAligner);
      maxAngularVelocityWidget = new ImGuiSliderDoubleWrapper("Max Angular Velocity", "%.2f", 0.1, Math.PI,
                                                              definition::getMaxAngularVelocity,
                                                              definition::setMaxAngularVelocity);
      maxAngularVelocityWidget.addWidgetAligner(widgetAligner);
      previewTimeWidget = new ImGuiSliderDoubleWrapper("Preview Time", "%.2f", 0.0, 1.0,
                                                       state.getPreviewRequestedTime()::getValue,
                                                       state.getPreviewRequestedTime()::setValue);
      previewTimeWidget.addButton("Play", this::togglePlayPausePreview);
      previewTimeWidget.addWidgetAligner(widgetAligner);
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
         screwAxisGizmo.setSelected(false);

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
      else if (isScrewTaskspace() && state.getScrewFrame().isChildOfWorld())
      {
         if (screwAxisGizmo.getPoseGizmo().getGizmoFrame() != state.getScrewFrame().getReferenceFrame())
            screwAxisGizmo.getPoseGizmo().setGizmoFrame(state.getScrewFrame().getReferenceFrame());

         if (!getSelected())
            screwAxisGizmo.setSelected(false);

         RDXCRDTTools.syncGizmoWithBidirectionalField(screwAxisGizmo.getPoseGizmo(), definition.getScrewAxisPoseInObjectFrame(), definition);

         double screwAxisLineWidth = 0.005;
         screwAxisGraphic.update(screwAxisGizmo.getPoseGizmo().getPose(), screwAxisLineWidth, 1.0);

         double trajectoryLineWidth = 0.01;
         trajectoryPoses.clear();
         for (int i = 0; i < state.getPreviewTrajectory().getSize(); i++)
            trajectoryPoses.add().set(state.getPreviewTrajectory().getValueReadOnly(i));
         trajectoryGraphic.update(trajectoryLineWidth, trajectoryPoses);

         if (playbackPreview)
         {
            double requestedTime = state.getPreviewRequestedTime().getValue();
            requestedTime += playbackStopwatch.lap() / state.getPreviewTrajectoryDuration().getValue();
            requestedTime %= 1.0;
            state.getPreviewRequestedTime().setValue(requestedTime);
         }
      }
   }

   private void visualizeIK()
   {
      RDXArmMultiBodyGraphic armMultiBodyGraphic = armMultiBodyGraphics.get(definition.getSide());

      if (isScrewTaskspace())
      {
         armMultiBodyGraphic.getFloatingJoint().getJointPose().set(syncedRobot.getFramePoseReadOnly(HumanoidReferenceFrames::getChestFrame));
         for (int i = 0; i < armMultiBodyGraphic.getJoints().length; i++)
            armMultiBodyGraphic.getJoints()[i].setQ(state.getScrewPreviewJointAngles().getValueReadOnly(i));
         armMultiBodyGraphic.updateAfterModifyingConfiguration();
         armMultiBodyGraphic.setColor(RDXIKSolverColors.getColor(state.getPreviewSolutionQuality().getValue()));
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
         ImGui.checkbox(labels.get("Adjust Screw Axis Pose"), screwAxisGizmo.getSelected());
         parentFrameComboBox.render();
         int size = state.getPreviewTrajectory().getSize();
         int limit = ArmActionState.TRAJECTORY_SIZE_LIMIT;
         if (size == limit)
            ImGui.pushStyleColor(ImGuiCol.Text, ImGuiTools.RED);
         ImGui.text("Trajectory points: %d/%d  Duration: %.1f s  Velocity %.2f m/s  %.2f %s/s"
                          .formatted(size,
                                     limit,
                                     state.getPreviewTrajectoryDuration().getValue(),
                                     state.getPreviewTrajectoryLinearVelocity().getValue(),
                                     state.getPreviewTrajectoryAngularVelocity().getValue(),
                                     EuclidCoreMissingTools.DEGREE_SYMBOL));
         if (size == limit)
            ImGui.popStyleColor();
         translationWidget.renderImGuiWidget();
         rotationWidget.renderImGuiWidget();
         maxLinearVelocityWidget.renderImGuiWidget();
         maxAngularVelocityWidget.renderImGuiWidget();
         linearPositionWeightWidget.renderImGuiWidget();
         angularPositionWeightWidget.renderImGuiWidget();
         jointspaceWeightWidget.renderImGuiWidget();
         ImGui.pushItemWidth(ImGui.getFontSize() * 10.0f);
         positionErrorToleranceInput.renderImGuiWidget();
         orientationErrorToleranceDegreesInput.renderImGuiWidget();
         ImGui.popItemWidth();
         previewTimeWidget.renderImGuiWidget();
      }
   }

   private void togglePlayPausePreview()
   {
      playbackPreview = !playbackPreview;
      previewTimeWidget.addButton(playbackPreview ? "Pause" : "Play", this::togglePlayPausePreview);

      if (playbackPreview)
         playbackStopwatch.reset();
   }

   @Override
   public void deselectGizmos()
   {
      poseGizmo.setSelected(false);
      screwAxisGizmo.setSelected(false);
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
      else if (isScrewTaskspace() && state.getScrewFrame().isChildOfWorld())
      {
         screwAxisGizmo.calculate3DViewPick(input);
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
      else if (isScrewTaskspace() && state.getScrewFrame().isChildOfWorld())
      {
         screwAxisGizmo.process3DViewInput(input);
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
      else if (isScrewTaskspace() && state.getScrewFrame().isChildOfWorld())
      {
         screwAxisGizmo.getVirtualRenderables(renderables, pool);
         screwAxisGraphic.getRenderables(renderables, pool);
         trajectoryGraphic.getRenderables(renderables, pool);
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
         return definition.getSide().getPascalCaseName() + " Screw Primitive";
      return definition.getSide().getPascalCaseName() + " Arm";
   }

   public ReferenceFrame getReferenceFrame()
   {
      return poseGizmo.getPoseGizmo().getGizmoFrame();
   }
}
