package us.ihmc.rdx.behaviorTree.actions;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.type.ImBoolean;
import us.ihmc.behaviors.behaviorTree.action.actions.FootstepPlanActionDefinition;
import us.ihmc.behaviors.behaviorTree.action.actions.FootstepPlanActionFootstepState;
import us.ihmc.behaviors.behaviorTree.action.actions.FootstepPlanActionState;
import us.ihmc.behaviors.tools.MinimalFootstep;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.commons.lists.RecyclingArrayListTools;
import us.ihmc.commons.thread.Notification;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.footstepPlanning.graphSearch.parameters.InitialStanceSide;
import us.ihmc.rdx.behaviorTree.RDXBehaviorTreeRootNode;
import us.ihmc.rdx.behaviorTree.RDXCRDTTools;
import us.ihmc.rdx.imgui.ImBooleanWrapper;
import us.ihmc.rdx.imgui.ImDoubleWrapper;
import us.ihmc.rdx.imgui.ImIntegerWrapper;
import us.ihmc.rdx.imgui.ImGuiReferenceFrameLibraryCombo;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.mesh.RDXMutableArrowModel;
import us.ihmc.rdx.ui.RDX3DPanelTooltip;
import us.ihmc.rdx.ui.RDXStoredPropertySetTuner;
import us.ihmc.rdx.ui.gizmo.RDXPose3DGizmo;
import us.ihmc.rdx.ui.gizmo.RDXSelectablePose3DGizmo;
import us.ihmc.rdx.ui.graphics.RDXFootstepPlanGraphic;
import us.ihmc.rdx.ui.widgets.ImGuiFootstepsWidget;
import us.ihmc.rdx.vr.RDXVRContext;
import us.ihmc.robotics.EuclidCoreMissingTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.ArrayList;

import static behavior_msgs.msg.dds.FootstepPlanActionDefinitionMessage.*;

public class RDXFootstepPlanAction extends RDXActionNode<FootstepPlanActionState, FootstepPlanActionDefinition>
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImGuiReferenceFrameLibraryCombo parentFrameComboBox;
   private final ImBoolean editManuallyPlacedSteps = new ImBoolean();
   private final ImBooleanWrapper manuallyPlaceStepsWrapper;
   private final ImDoubleWrapper swingDurationWidget;
   private final ImDoubleWrapper transferDurationWidget;
   private final ImBooleanWrapper walkWithGoalOrientationWidget;
   private final ImBooleanWrapper planWithBodyPathWidget;
   private final ImIntegerWrapper quickHipWidthWidget;
   private final ImIntegerWrapper quickStepLengthWidget;
   private final ImIntegerWrapper quickNextPelvisYawLimitWidget;
   private final ImIntegerWrapper quickInwardLimitWidget;
   private final ImIntegerWrapper quickOutwardLimitWidget;
   private final ImIntegerWrapper quickStepAngleLimitWidget;
   private final ImIntegerWrapper quickSwingTimeDistanceLowerWidget;
   private final ImIntegerWrapper quickSwingTimeDistanceUpperWidget;
   private final ImDoubleWrapper quickMinSwingTimeWidget;
   private final ImDoubleWrapper quickMaxSwingTimeWidget;
   private final RDXStoredPropertySetTuner plannerParametersWidgets;
   private int numberOfAllocatedFootsteps = 0;
   private final RecyclingArrayList<RDXFootstepPlanActionFootstep> manuallyPlacedFootsteps;
   private final TypedNotification<RobotSide> userAddedFootstep = new TypedNotification<>();
   private final Notification userRemovedFootstep = new Notification();
   private final FramePose3D goalArrowPose = new FramePose3D();
   private final Vector3D goalFocalPointVector = new Vector3D();
   private final RDXMutableArrowModel goalArrowGraphic = new RDXMutableArrowModel();
   private final RDXSelectablePose3DGizmo goalStancePointGizmo = new RDXSelectablePose3DGizmo();
   private final RDXSelectablePose3DGizmo goalFocalPointGizmo = new RDXSelectablePose3DGizmo();
   private final SideDependentList<RDXFootstepPlanActionGoalFootstep> goalFeet = new SideDependentList<>();
   private final ArrayList<RDXPose3DGizmo> waypointGizmos = new ArrayList<>();
   private final RDX3DPanelTooltip tooltip;
   private final ImGuiFootstepsWidget footstepsWidget = new ImGuiFootstepsWidget();
   private final RDXFootstepPlanGraphic previewFootstepPlan;

   public RDXFootstepPlanAction(long id, RDXBehaviorTreeRootNode rootNode)
   {
      super(new FootstepPlanActionState(id, rootNode.getState()), rootNode);

      manuallyPlacedFootsteps = new RecyclingArrayList<>(() ->
         new RDXFootstepPlanActionFootstep(baseUI,
                                           robotModel,
                                           this,
                                           RecyclingArrayListTools.getUnsafe(state.getManuallyPlacedFootsteps(), numberOfAllocatedFootsteps++)));

      parentFrameComboBox = new ImGuiReferenceFrameLibraryCombo("Parent frame",
                                                                scene::getAllFrameNames,
                                                                definition::getParentFrameName,
                                                                this::changeParentFrame);
      manuallyPlaceStepsWrapper = new ImBooleanWrapper(definition::getIsManuallyPlaced,
                                                       definition::setIsManuallyPlaced,
                                                       imBoolean ->
      {
         if (ImGui.checkbox(labels.get("Manually place steps"), imBoolean))
         {
            if (imBoolean.get()) // Copy planned footsteps into manually placed footsteps
            {
               while (!manuallyPlacedFootsteps.isEmpty())
               {
                  RecyclingArrayListTools.removeLast(manuallyPlacedFootsteps);
                  RecyclingArrayListTools.removeLast(state.getManuallyPlacedFootsteps());
                  RecyclingArrayListTools.removeLast(definition.getManuallyPlacedFootsteps().getValueAndModify());
               }

               for (int i = 0; i < state.getPreviewFootsteps().getSize(); i++)
               {
                  RobotSide side = state.getPreviewFootsteps().getSide(i);
                  RecyclingArrayListTools.addToAll(definition.getManuallyPlacedFootsteps().getValueAndModify(), state.getManuallyPlacedFootsteps());
                  RDXFootstepPlanActionFootstep addedFootstep = manuallyPlacedFootsteps.add();
                  addedFootstep.getDefinition().setSide(side);
                  addedFootstep.update();

                  Pose3DReadOnly poseInWorld = state.getPreviewFootsteps().getPoseReadOnly(i);
                  FramePose3D poseInParent = new FramePose3D(poseInWorld);
                  poseInParent.changeFrame(addedFootstep.getState().getSoleFrame().getReferenceFrame().getParent());
                  addedFootstep.getDefinition().getSoleToPlanFrameTransform().getValueAndModify().set(poseInParent);
               }
            }
         }
      });
      swingDurationWidget = new ImDoubleWrapper(definition::getSwingDuration,
                                                definition::setSwingDuration,
                                                imDouble -> ImGui.inputDouble(labels.get("Swing duration"), imDouble));
      transferDurationWidget = new ImDoubleWrapper(definition::getTransferDuration,
                                                   definition::setTransferDuration,
                                                   imDouble -> ImGui.inputDouble(labels.get("Transfer duration"), imDouble));
      walkWithGoalOrientationWidget = new ImBooleanWrapper(definition.getPlannerWalkWithGoalOrientation()::getValue,
                                                           definition.getPlannerWalkWithGoalOrientation()::setValue,
                                                           imBoolean -> ImGui.checkbox(labels.get("Walk with goal orientation"), imBoolean));
      planWithBodyPathWidget = new ImBooleanWrapper(definition.getPlannerPlanWithBodyPath()::getValue,
                                                    definition.getPlannerPlanWithBodyPath()::setValue,
                                                    imBoolean -> ImGui.checkbox(labels.get("Plan with body path"), imBoolean));
      quickHipWidthWidget = new ImIntegerWrapper(() -> (int) Math.round(definition.getQuickHipWidth().getValue() * 100.0),
                                                 value -> definition.getQuickHipWidth().setValue(value / 100.0),
                                                 imInt -> ImGui.inputInt(labels.get("Quick hip width (cm)"), imInt, 1, 5));
      quickStepLengthWidget = new ImIntegerWrapper(() -> (int) Math.round(definition.getQuickStepLength().getValue() * 100.0),
                                                   value -> definition.getQuickStepLength().setValue(value / 100.0),
                                                   imInt -> ImGui.inputInt(labels.get("Quick step length (cm)"), imInt, 1, 5));
      quickNextPelvisYawLimitWidget = new ImIntegerWrapper(() -> (int) Math.round(Math.toDegrees(definition.getQuickNextPelvisYawLimit().getValue())),
                                                           value -> definition.getQuickNextPelvisYawLimit().setValue(Math.toRadians(value)),
                                                           imInt -> ImGui.inputInt(labels.get("Quick next pelvis yaw limit (%s)".formatted(EuclidCoreMissingTools.DEGREE_SYMBOL)), imInt, 1, 5));
      quickInwardLimitWidget = new ImIntegerWrapper(() -> (int) Math.round(Math.toDegrees(definition.getQuickInwardLimit().getValue())),
                                                    value -> definition.getQuickInwardLimit().setValue(Math.toRadians(value)),
                                                    imInt -> ImGui.inputInt(labels.get("Quick inward limit (%s)".formatted(EuclidCoreMissingTools.DEGREE_SYMBOL)), imInt, 1, 5));
      quickOutwardLimitWidget = new ImIntegerWrapper(() -> (int) Math.round(Math.toDegrees(definition.getQuickOutwardLimit().getValue())),
                                                     value -> definition.getQuickOutwardLimit().setValue(Math.toRadians(value)),
                                                     imInt -> ImGui.inputInt(labels.get("Quick outward limit (%s)".formatted(EuclidCoreMissingTools.DEGREE_SYMBOL)), imInt, 1, 5));
      quickStepAngleLimitWidget = new ImIntegerWrapper(() -> (int) Math.round(Math.toDegrees(definition.getQuickStepAngleLimit().getValue())),
                                                       value -> definition.getQuickStepAngleLimit().setValue(Math.toRadians(value)),
                                                       imInt -> ImGui.inputInt(labels.get("Quick step angle limit (%s)".formatted(EuclidCoreMissingTools.DEGREE_SYMBOL)), imInt, 1, 5));
      quickSwingTimeDistanceLowerWidget = new ImIntegerWrapper(() -> (int) Math.round(definition.getQuickSwingTimeDistanceLower().getValue() * 100.0),
                                                               value -> definition.getQuickSwingTimeDistanceLower().setValue(value / 100.0),
                                                               imInt -> ImGui.inputInt(labels.get("Quick swing time distance lower (cm)"), imInt, 1, 5));
      quickSwingTimeDistanceUpperWidget = new ImIntegerWrapper(() -> (int) Math.round(definition.getQuickSwingTimeDistanceUpper().getValue() * 100.0),
                                                               value -> definition.getQuickSwingTimeDistanceUpper().setValue(value / 100.0),
                                                               imInt -> ImGui.inputInt(labels.get("Quick swing time distance upper (cm)"), imInt, 1, 5));
      quickMinSwingTimeWidget = new ImDoubleWrapper(definition.getQuickMinSwingTime()::getValue,
                                                    definition.getQuickMinSwingTime()::setValue,
                                                    imDouble -> ImGui.inputDouble(labels.get("Quick min swing time (s)"), imDouble, 0.05, 0.2, "%.3f", 0));
      quickMaxSwingTimeWidget = new ImDoubleWrapper(definition.getQuickMaxSwingTime()::getValue,
                                                    definition.getQuickMaxSwingTime()::setValue,
                                                    imDouble -> ImGui.inputDouble(labels.get("Quick max swing time (s)"), imDouble, 0.05, 0.2, "%.3f", 0));
      plannerParametersWidgets = new RDXStoredPropertySetTuner("Planner Parameters");
      plannerParametersWidgets.create(definition.getPlannerParametersUnsafe(), false);

      for (RobotSide side : RobotSide.values)
      {
         definition.getGoalFootstepToGoalY(side).setInitialValue(0.5 * side.negateIfRightSide(definition.getPlannerParametersReadOnly().getIdealFootstepWidth()));
         state.copyDefinitionToGoalFootstepToGoalTransform(side);
      }

      definition.getGoalFocalPoint().getValueToInitialize().set(0.1, 0.0, 0.0);

      goalStancePointGizmo.create(baseUI.getPrimary3DPanel());
      goalStancePointGizmo.getPoseGizmo().getCenterSphereToTorusRatio().set(0.5f);
      goalFocalPointGizmo.create(baseUI.getPrimary3DPanel());
      goalFocalPointGizmo.getPoseGizmo().getCenterSphereToTorusRatio().set(0.5f);

      for (RobotSide side : RobotSide.values)
         goalFeet.put(side, new RDXFootstepPlanActionGoalFootstep(baseUI, side, definition, state, robotModel));

      previewFootstepPlan = new RDXFootstepPlanGraphic(robotModel.getContactPointParameters().getControllerFootGroundContactPoints());

      tooltip = new RDX3DPanelTooltip(baseUI.getPrimary3DPanel());
      baseUI.getPrimary3DPanel().addImGuiOverlayAddition(this::render3DPanelImGuiOverlays);
   }

   @Override
   public void update()
   {
      super.update();

      RecyclingArrayListTools.synchronizeSize(manuallyPlacedFootsteps, state.getManuallyPlacedFootsteps());

      if (state.areFramesInWorld())
      {
         // Add a footstep to the action data only
         if (userAddedFootstep.poll())
         {
            RobotSide newSide = userAddedFootstep.read();
            RecyclingArrayListTools.addToAll(definition.getManuallyPlacedFootsteps().getValueAndModify(), state.getManuallyPlacedFootsteps());
            RDXFootstepPlanActionFootstep addedFootstep = manuallyPlacedFootsteps.add();
            addedFootstep.getDefinition().setSide(newSide);
            addedFootstep.getState().update();
            FramePose3D newFootstepPose = new FramePose3D();
            if (manuallyPlacedFootsteps.size() > 1)
            {
               RDXFootstepPlanActionFootstep previousFootstep = manuallyPlacedFootsteps.get(manuallyPlacedFootsteps.size() - 2);
               newFootstepPose.setToZero(previousFootstep.getState().getSoleFrame().getReferenceFrame());

               if (previousFootstep.getDefinition().getSide() != newSide)
               {
                  double minStepWidth = robotModel.getWalkingControllerParameters().getSteppingParameters().getInPlaceWidth();
                  newFootstepPose.getPosition().addY(newSide.negateIfRightSide(minStepWidth));
               }
            }
            else
            {
               newFootstepPose.setToZero(syncedRobot.getReferenceFrames().getSoleFrame(newSide));
            }

            double aLittleInFront = 0.15;
            newFootstepPose.getPosition().addX(aLittleInFront);

            newFootstepPose.changeFrame(addedFootstep.getState().getSoleFrame().getReferenceFrame().getParent());
            addedFootstep.getDefinition().getSoleToPlanFrameTransform().getValueAndModify().set(newFootstepPose);
         }

         if (userRemovedFootstep.poll())
         {
            RecyclingArrayListTools.removeLast(manuallyPlacedFootsteps);
            RecyclingArrayListTools.removeLast(state.getManuallyPlacedFootsteps());
            RecyclingArrayListTools.removeLast(definition.getManuallyPlacedFootsteps().getValueAndModify());
         }

         for (RDXFootstepPlanActionFootstep footstep : manuallyPlacedFootsteps)
         {
            footstep.update();
         }

         ReferenceFrame parentFrame = state.getParentFrame();
         goalStancePointGizmo.getPoseGizmo().setParentFrame(parentFrame);
         goalFocalPointGizmo.getPoseGizmo().setParentFrame(parentFrame);

         for (RobotSide side : RobotSide.values)
            goalFeet.get(side).setParentFrame();

         if (!definition.getIsManuallyPlaced() && definition.getPlannerType().getValue() == QUICK)
         {
            while (waypointGizmos.size() < definition.getWaypoints().getSize())
            {
               RDXPose3DGizmo gizmo = new RDXPose3DGizmo();
               gizmo.create(baseUI.getPrimary3DPanel());
               gizmo.setResizeAutomatically(false);
               gizmo.setCenterSphereToTorusRatio(0.8f);
               waypointGizmos.add(gizmo);
            }

            for (int i = 0; i < definition.getWaypoints().getSize(); i++)
            {
               RDXPose3DGizmo gizmo = waypointGizmos.get(i);
               gizmo.setParentFrame(parentFrame);

               if (gizmo.getGizmoModifiedByUser().poll())
                  definition.getWaypoints().getValueAndModify().get(i).set(gizmo.getTransformToParent());
               else if (definition.isModified())
                  gizmo.getTransformToParent().set(definition.getWaypoints().getValueReadOnly(i));

               gizmo.update();
            }
         }

         RDXCRDTTools.syncGizmoWithBidirectionalField(goalStancePointGizmo.getPoseGizmo(), definition.getGoalStancePoint(), definition);
         RDXCRDTTools.syncGizmoWithBidirectionalField(goalFocalPointGizmo.getPoseGizmo(), definition.getGoalFocalPoint(), definition);

         for (RobotSide side : RobotSide.values)
         {
            RDXPose3DGizmo gizmo = goalFeet.get(side).getGizmo().getPoseGizmo();
            if (gizmo.getGizmoModifiedByUser().poll())
            {
               state.copyGoalFootstepToGoalTransformToDefinition(side);
            }
            else if (definition.isModified())
            {
               state.copyDefinitionToGoalFootstepToGoalTransform(side);
            }

            gizmo.update();
         }

         state.getGoalFrame().getReferenceFrame().update();

         // Update arrow graphic geometry
         goalArrowPose.setToZero(state.getParentFrame());
         goalArrowPose.getTranslation().set(definition.getGoalStancePoint().getValueReadOnly());
         goalFocalPointVector.sub(definition.getGoalFocalPoint().getValueReadOnly(), definition.getGoalStancePoint().getValueReadOnly());
         EuclidGeometryTools.orientation3DFromZUpToVector3D(goalFocalPointVector, goalArrowPose.getOrientation());
         goalArrowPose.changeFrame(ReferenceFrame.getWorldFrame());
         goalArrowGraphic.update(goalFocalPointVector.norm(), Color.WHITE);
         goalArrowGraphic.accessModelIfExists(modelInstance ->
         {
            modelInstance.setPoseInWorldFrame(goalArrowPose);
            modelInstance.setOpacity(0.6f);
         });

         for (RobotSide side : RobotSide.values)
            goalFeet.get(side).updatePoses();

         if (state.getIsNextForExecution())
         {
            if (!definition.getIsManuallyPlaced() && state.getPreviewFootsteps().getSize() > 0)
            {
               ArrayList<MinimalFootstep> minimalFootsteps = new ArrayList<>();
               for (int i = 0; i < state.getPreviewFootsteps().getSize(); i++)
               {
                  minimalFootsteps.add(new MinimalFootstep(state.getPreviewFootsteps().getSide(i), new Pose3D(state.getPreviewFootsteps().getPoseReadOnly(i))));
               }
               previewFootstepPlan.generateMeshesAsync(minimalFootsteps);
            }
            else
            {
               previewFootstepPlan.clear();
            }
            previewFootstepPlan.update();
         }
      }
   }

   @Override
   public void renderTreeViewRow()
   {
      super.renderRowBeginning();
      super.renderEditableName();

      ImGui.sameLine();
      footstepsWidget.render(ImGui.getFrameHeight());

      renderRowEnd();
   }

   @Override
   public void calculateVRPick(RDXVRContext vrContext)
   {
      if (getSelected() && state.areFramesInWorld())
      {
         // TODO: VR support for Pose3DGizmo
      }
   }

   @Override
   public void processVRInput(RDXVRContext vrContext)
   {
      if (getSelected() && state.areFramesInWorld())
      {
         if (!definition.getIsManuallyPlaced())
         {
            // TODO: VR support for Pose3DGizmo
         }
      }
   }

   @Override
   public void calculate3DViewPick(ImGui3DViewInput input)
   {
      if (getSelected() && state.areFramesInWorld())
      {
         if (definition.getIsManuallyPlaced())
         {
            for (RDXFootstepPlanActionFootstep footstep : manuallyPlacedFootsteps)
            {
               footstep.calculate3DViewPick(input);
            }
         }
         else
         {
            goalStancePointGizmo.calculate3DViewPick(input);
            goalFocalPointGizmo.calculate3DViewPick(input);
            for (RobotSide side : RobotSide.values)
               goalFeet.get(side).calculate3DViewPick(input);
            if (definition.getPlannerType().getValue() == QUICK)
            {
               int waypointCount = Math.min(waypointGizmos.size(), definition.getWaypoints().getSize());
               for (int i = 0; i < waypointCount; i++)
                  waypointGizmos.get(i).calculate3DViewPick(input);
            }
         }
      }
   }

   @Override
   public void process3DViewInput(ImGui3DViewInput input)
   {
      if (getSelected() && state.areFramesInWorld())
      {
         if (definition.getIsManuallyPlaced())
         {
            for (RDXFootstepPlanActionFootstep footstep : manuallyPlacedFootsteps)
            {
               footstep.process3DViewInput(input);
            }
         }
         else
         {
            goalStancePointGizmo.process3DViewInput(input, false);
            goalFocalPointGizmo.process3DViewInput(input, false);
            tooltip.setInput(input);
            for (RobotSide side : RobotSide.values)
               goalFeet.get(side).process3DViewInput(input);
            if (definition.getPlannerType().getValue() == QUICK)
            {
               int waypointCount = Math.min(waypointGizmos.size(), definition.getWaypoints().getSize());
               for (int i = 0; i < waypointCount; i++)
                  waypointGizmos.get(i).process3DViewInput(input);
            }
         }
      }
   }

   @Override
   protected void renderImGuiWidgetsInternal()
   {
      parentFrameComboBox.render();

      ImGui.pushItemWidth(80.0f);
      swingDurationWidget.renderImGuiWidget();
      transferDurationWidget.renderImGuiWidget();
      ImGui.popItemWidth();
      ImGui.text("Execution mode:");
      ImGui.sameLine();
      if (ImGui.radioButton(labels.get("Override"), definition.getExecutionMode().getValue() == ExecutionMode.OVERRIDE))
         definition.getExecutionMode().setValue(ExecutionMode.OVERRIDE);
      ImGui.sameLine();
      if (ImGui.radioButton(labels.get("Queue"), definition.getExecutionMode().getValue() == ExecutionMode.QUEUE))
         definition.getExecutionMode().setValue(ExecutionMode.QUEUE);

      manuallyPlaceStepsWrapper.renderImGuiWidget();

      if (state.areFramesInWorld()) // Not allowing modification if not renderable
      {
         if (definition.getIsManuallyPlaced())
         {
            if (ImGui.checkbox(labels.get("Edit Manually Placed Steps"), editManuallyPlacedSteps))
               for (RDXFootstepPlanActionFootstep manuallyPlacedFootstep : manuallyPlacedFootsteps)
                  manuallyPlacedFootstep.updateGizmo();

            ImGui.sameLine();
            if (editManuallyPlacedSteps.get() && ImGui.button("Select All Footsteps"))
               for (RDXFootstepPlanActionFootstep manuallyPlacedFootstep : manuallyPlacedFootsteps)
                  manuallyPlacedFootstep.getInteractableFootstep().getSelectablePose3DGizmo().setSelected(true);

            ImGui.text("Number of footsteps: %d".formatted(manuallyPlacedFootsteps.size()));
            ImGui.text("Add:");
            for (RobotSide side : RobotSide.values)
            {
               ImGui.sameLine();
               if (ImGui.button(labels.get(side.getPascalCaseName())))
                  userAddedFootstep.set(side);
            }
            if (!state.getManuallyPlacedFootsteps().isEmpty())
            {
               ImGui.sameLine();
               ImGui.text("Remove:");
               ImGui.sameLine();
               if (ImGui.button(labels.get("Last")))
               {
                  userRemovedFootstep.set();
               }
            }
         }
         else
         {
            ImGui.text("Planning goal gizmo adjustment:");
            if (ImGui.checkbox(labels.get("Stance Point"), goalStancePointGizmo.getSelected()))
               goalStancePointGizmo.getPoseGizmo().getTransformToParent().getTranslation().set(definition.getGoalStancePoint().getValueReadOnly());
            ImGui.sameLine();
            if (ImGui.checkbox(labels.get("Focal Point"), goalFocalPointGizmo.getSelected()))
               goalFocalPointGizmo.getPoseGizmo().getTransformToParent().getTranslation().set(definition.getGoalFocalPoint().getValueReadOnly());
            for (RobotSide side : RobotSide.values)
            {
               if (ImGui.checkbox(labels.get(side.getPascalCaseName() + " Foot to Goal"), goalFeet.get(side).getGizmo().getSelected()))
                  state.copyDefinitionToGoalFootstepToGoalTransform(side);
               if (side == RobotSide.LEFT)
                  ImGui.sameLine();
            }
            ImGui.text("Initial stance side:");
            for (InitialStanceSide initialStanceSide : InitialStanceSide.values)
            {
               ImGui.sameLine();
               if (ImGui.radioButton(labels.get(initialStanceSide.name()), definition.getPlannerInitialStanceSide().getValue() == initialStanceSide))
                  definition.getPlannerInitialStanceSide().setValue(initialStanceSide);
            }

            if (ImGui.radioButton(labels.get("Quick"), definition.getPlannerType().getValue() == QUICK))
               definition.getPlannerType().setValue(QUICK);
            ImGui.sameLine();
            if (ImGui.radioButton(labels.get("Turn walk turn"), definition.getPlannerType().getValue() == TURN_WALK_TURN))
               definition.getPlannerType().setValue(TURN_WALK_TURN);
            ImGui.sameLine();
            if (ImGui.radioButton(labels.get("A*"), definition.getPlannerType().getValue() == A_STAR))
               definition.getPlannerType().setValue(A_STAR);

            ImGui.beginDisabled(definition.getPlannerType().getValue() == QUICK);
            walkWithGoalOrientationWidget.renderImGuiWidget();
            ImGui.endDisabled();

            planWithBodyPathWidget.renderImGuiWidget();

            if (definition.getPlannerType().getValue() == QUICK)
            {
               ImGui.text("Waypoints: %d".formatted(definition.getWaypoints().getSize()));
               ImGui.sameLine();
               if (ImGui.button(labels.get("Add", "Waypoint")))
               {
                  Pose3D waypoint = definition.getWaypoints().getValueAndModify().add();
                  if (definition.getWaypoints().getSize() > 1)
                  {
                     waypoint.set(definition.getWaypoints().getValueReadOnly(definition.getWaypoints().getSize() - 2));
                     waypoint.appendTranslation(0.1, 0.0, 0.0);
                  }
                  else
                  {
                     waypoint.getPosition().set(definition.getGoalStancePoint().getValueReadOnly());
                  }
               }
               if (definition.getWaypoints().getSize() > 0)
               {
                  ImGui.sameLine();
                  if (ImGui.button(labels.get("Remove", "Waypoint")))
                     RecyclingArrayListTools.removeLast(definition.getWaypoints().getValueAndModify());
               }
            }

            ImGui.text("Preview steps: %d".formatted(state.getPreviewFootsteps().getSize()));

            if (ImGui.collapsingHeader(labels.get("Planner Parameters")))
            {
               ImGui.pushItemWidth(ImGui.getFontSize() * 8.0f);
               if (definition.getPlannerType().getValue() == QUICK)
               {
                  quickHipWidthWidget.renderImGuiWidget();
                  quickStepLengthWidget.renderImGuiWidget();
                  quickNextPelvisYawLimitWidget.renderImGuiWidget();
                  quickInwardLimitWidget.renderImGuiWidget();
                  quickOutwardLimitWidget.renderImGuiWidget();
                  quickStepAngleLimitWidget.renderImGuiWidget();
                  quickSwingTimeDistanceLowerWidget.renderImGuiWidget();
                  quickSwingTimeDistanceUpperWidget.renderImGuiWidget();
                  quickMinSwingTimeWidget.renderImGuiWidget();
                  quickMaxSwingTimeWidget.renderImGuiWidget();
               }
               else
                  if (plannerParametersWidgets.renderImGuiWidgetsSimple())
                     definition.getAndModifyPlannerParameters();
               ImGui.popItemWidth();
            }
         }
      }
   }

   @Override
   public void deselectGizmos()
   {
      goalStancePointGizmo.setSelected(false);
      goalFocalPointGizmo.setSelected(false);
      for (RobotSide side : RobotSide.values)
         goalFeet.get(side).getGizmo().setSelected(false);
      editManuallyPlacedSteps.set(false);
   }

   public void render3DPanelImGuiOverlays()
   {
      if (!definition.getIsManuallyPlaced())
      {
//         if (hovered)
//         {
//            tooltip.render("%s Action\nIndex: %d\nName: %s".formatted(getActionTypeTitle(), state.getActionIndex(), definition.getName()));
//         }
      }
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if ((state.getIsNextForExecution() || getSelected() || state.getIsExecuting()) && state.areFramesInWorld())
      {
         if (definition.getIsManuallyPlaced())
            for (RDXFootstepPlanActionFootstep footstep : manuallyPlacedFootsteps)
               footstep.getVirtualRenderables(renderables, pool);
         else
         {
            if (!state.getIsExecuting())
            {
               goalArrowGraphic.getRenderables(renderables, pool);

               goalStancePointGizmo.getVirtualRenderables(renderables, pool);
               goalFocalPointGizmo.getVirtualRenderables(renderables, pool);
               for (RobotSide side : RobotSide.values)
                  goalFeet.get(side).getRenderables(renderables, pool, footstepsWidget.getIsHovered().get(side));
               if (definition.getPlannerType().getValue() == QUICK)
                  for (int i = 0; i < Math.min(waypointGizmos.size(), definition.getWaypoints().getSize()); i++)
                     waypointGizmos.get(i).getRenderables(renderables, pool);
            }

            if (state.getIsNextForExecution() || state.getIsExecuting())
               previewFootstepPlan.getRenderables(renderables, pool);
         }
      }
   }

   @Override
   public String getLeafTypeTitle()
   {
      return "Footstep Plan";
   }

   public void changeParentFrame(String newParentFrameName)
   {
      definition.setParentFrameName(newParentFrameName);
      // Timestamp modification to prevent the frame from glitching when changing frames
      state.getGoalFrame().changeFrame(newParentFrameName, state.getGoalToParentTransform().getValueAndModify());

      ReferenceFrame newParent = state.getGoalFrame().getReferenceFrame().getParent();
      FramePoint3D frameStancePoint;
      FramePoint3D frameFocalPoint;
      if (newParent.getRootFrame() == state.getParentFrame().getRootFrame())
      {
         // Keep the points in the same place w.r.t common ancestor frames
         frameStancePoint = new FramePoint3D(state.getParentFrame(), definition.getGoalStancePoint().getValueReadOnly());
         frameFocalPoint = new FramePoint3D(state.getParentFrame(), definition.getGoalFocalPoint().getValueReadOnly());
         frameStancePoint.changeFrame(newParent);
         frameFocalPoint.changeFrame(newParent);
      }
      else // The frame was detached so we keep the same transform to the new frame
      {
         frameStancePoint = new FramePoint3D(newParent, definition.getGoalStancePoint().getValueReadOnly());
         frameFocalPoint = new FramePoint3D(newParent, definition.getGoalFocalPoint().getValueReadOnly());
      }

      definition.getGoalStancePoint().getValueAndModify().set(frameStancePoint);
      definition.getGoalFocalPoint().getValueAndModify().set(frameFocalPoint);
      goalStancePointGizmo.getPoseGizmo().setParentFrame(newParent);
      goalFocalPointGizmo.getPoseGizmo().setParentFrame(newParent);
      goalStancePointGizmo.getPoseGizmo().getTransformToParent().getTranslation().set(definition.getGoalStancePoint().getValueReadOnly());
      goalFocalPointGizmo.getPoseGizmo().getTransformToParent().getTranslation().set(definition.getGoalFocalPoint().getValueReadOnly());
      goalStancePointGizmo.getPoseGizmo().update();
      goalFocalPointGizmo.getPoseGizmo().update();

      for (FootstepPlanActionFootstepState footstepState : state.getManuallyPlacedFootsteps())
      {
         footstepState.getSoleFrame().changeFrame(newParentFrameName);
      }
   }

   @Override
   public void destroy()
   {
      super.destroy();
      previewFootstepPlan.destroy(); // It is important to destroy this because its thread needs to be shutdown
   }

   public ImBoolean getEditManuallyPlacedSteps()
   {
      return editManuallyPlacedSteps;
   }
}
