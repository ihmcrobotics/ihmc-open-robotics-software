package us.ihmc.rdx.ui.behavior.actions;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.type.ImBoolean;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.sequence.actions.FootstepPlanActionDefinition;
import us.ihmc.behaviors.sequence.actions.FootstepPlanActionFootstepState;
import us.ihmc.behaviors.sequence.actions.FootstepPlanActionState;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.commons.thread.Notification;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.rdx.imgui.ImBooleanWrapper;
import us.ihmc.rdx.imgui.ImDoubleWrapper;
import us.ihmc.rdx.imgui.ImGuiReferenceFrameLibraryCombo;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.mesh.RDXMutableArrowModel;
import us.ihmc.rdx.ui.RDX3DPanelTooltip;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.behavior.sequence.RDXActionNode;
import us.ihmc.rdx.ui.gizmo.RDXPose3DGizmo;
import us.ihmc.rdx.ui.gizmo.RDXSelectablePose3DGizmo;
import us.ihmc.rdx.ui.graphics.RDXFootstepGraphic;
import us.ihmc.rdx.ui.widgets.ImGuiFootstepsWidget;
import us.ihmc.rdx.vr.RDXVRContext;
import us.ihmc.robotics.lists.RecyclingArrayListTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class RDXFootstepPlanAction extends RDXActionNode<FootstepPlanActionState, FootstepPlanActionDefinition>
{
   private final DRCRobotModel robotModel;
   private final ROS2SyncedRobotModel syncedRobot;
   private final FootstepPlanActionState state;
   private final FootstepPlanActionDefinition definition;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImGuiReferenceFrameLibraryCombo parentFrameComboBox;
   private final ImBoolean editManuallyPlacedSteps = new ImBoolean();
   private final ImBooleanWrapper manuallyPlaceStepsWrapper;
   private final ImDoubleWrapper swingDurationWidget;
   private final ImDoubleWrapper transferDurationWidget;
   private int numberOfAllocatedFootsteps = 0;
   private final RecyclingArrayList<RDXFootstepPlanActionFootstep> manuallyPlacedFootsteps;
   private final TypedNotification<RobotSide> userAddedFootstep = new TypedNotification<>();
   private final Notification userRemovedFootstep = new Notification();
   private final FramePose3D goalArrowPose = new FramePose3D();
   private final Vector3D goalFocalPointVector = new Vector3D();
   private final RDXMutableArrowModel goalArrowGraphic = new RDXMutableArrowModel();
   private final RDXSelectablePose3DGizmo goalStancePointGizmo = new RDXSelectablePose3DGizmo();
   private final RDXSelectablePose3DGizmo goalFocalPointGizmo = new RDXSelectablePose3DGizmo();
   private final SideDependentList<RDXFootstepGraphic> goalFeetGraphics = new SideDependentList<>();
   private final SideDependentList<ImBoolean> goalFeetPosesSelected = new SideDependentList<>();
   private final SideDependentList<RDXPose3DGizmo> goalFeetGizmos = new SideDependentList<>();
   private final RDX3DPanelTooltip tooltip;
   private final ImGuiFootstepsWidget footstepsWidget = new ImGuiFootstepsWidget();

   public RDXFootstepPlanAction(long id,
                                CRDTInfo crdtInfo,
                                WorkspaceResourceDirectory saveFileDirectory,
                                RDXBaseUI baseUI,
                                DRCRobotModel robotModel,
                                ROS2SyncedRobotModel syncedRobot,
                                ReferenceFrameLibrary referenceFrameLibrary,
                                FootstepPlannerParametersBasics footstepPlannerParameters)
   {
      super(new FootstepPlanActionState(id, crdtInfo, saveFileDirectory, referenceFrameLibrary));

      state = getState();
      definition = getDefinition();

      this.robotModel = robotModel;
      this.syncedRobot = syncedRobot;

      definition.setName("Footstep plan");

      parentFrameComboBox = new ImGuiReferenceFrameLibraryCombo("Parent frame",
                                                                referenceFrameLibrary,
                                                                definition::getParentFrameName,
                                                                this::changeParentFrame);
      manuallyPlaceStepsWrapper = new ImBooleanWrapper(definition::getIsManuallyPlaced,
                                                       definition::setIsManuallyPlaced,
                                                       imBoolean -> ImGui.checkbox(labels.get("Manually place steps"), imBoolean));
      swingDurationWidget = new ImDoubleWrapper(definition::getSwingDuration,
                                                definition::setSwingDuration,
                                                imDouble -> ImGui.inputDouble(labels.get("Swing duration"), imDouble));
      transferDurationWidget = new ImDoubleWrapper(definition::getTransferDuration,
                                                   definition::setTransferDuration,
                                                   imDouble -> ImGui.inputDouble(labels.get("Transfer duration"), imDouble));

      manuallyPlacedFootsteps = new RecyclingArrayList<>(() ->
         new RDXFootstepPlanActionFootstep(baseUI,
                                           robotModel,
                                           this,
                                           RecyclingArrayListTools.getUnsafe(state.getFootsteps(), numberOfAllocatedFootsteps++)));

      for (RobotSide side : RobotSide.values)
      {
         definition.getGoalFootstepToGoalY(side).setValue(0.5 * side.negateIfRightSide(footstepPlannerParameters.getIdealFootstepWidth()));
      }

      definition.getGoalFocalPoint().getValue().set(0.1, 0.0, 0.0);

      goalStancePointGizmo.create(baseUI.getPrimary3DPanel());
      goalFocalPointGizmo.create(baseUI.getPrimary3DPanel());

      for (RobotSide side : RobotSide.values)
      {
         goalFeetPosesSelected.put(side, new ImBoolean(false));

         RDXPose3DGizmo footGizmo = new RDXPose3DGizmo(ReferenceFrame.getWorldFrame(),
                                                       state.getGoalFootstepToGoalTransform(side));
         footGizmo.create(baseUI.getPrimary3DPanel());
         goalFeetGizmos.put(side, footGizmo);

         RDXFootstepGraphic goalFootGraphic = new RDXFootstepGraphic(robotModel.getContactPointParameters().getControllerFootGroundContactPoints(), side);
         goalFootGraphic.create();
         goalFeetGraphics.put(side, goalFootGraphic);
      }

      tooltip = new RDX3DPanelTooltip(baseUI.getPrimary3DPanel());
      baseUI.getPrimary3DPanel().addImGuiOverlayAddition(this::render3DPanelImGuiOverlays);
   }

   @Override
   public void update()
   {
      super.update();

      RecyclingArrayListTools.synchronizeSize(manuallyPlacedFootsteps, state.getFootsteps());

      if (state.areFramesInWorld())
      {
         // Add a footstep to the action data only
         if (userAddedFootstep.poll())
         {
            RobotSide newSide = userAddedFootstep.read();
            RecyclingArrayListTools.addToAll(definition.getFootsteps().getValue(), state.getFootsteps());
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
            addedFootstep.getDefinition().getSoleToPlanFrameTransform().getValue().set(newFootstepPose);
         }

         if (userRemovedFootstep.poll())
         {
            RecyclingArrayListTools.removeLast(manuallyPlacedFootsteps);
            RecyclingArrayListTools.removeLast(state.getFootsteps());
            RecyclingArrayListTools.removeLast(definition.getFootsteps().getValue());
         }

         for (RDXFootstepPlanActionFootstep footstep : manuallyPlacedFootsteps)
         {
            footstep.update();
         }

         ReferenceFrame parentFrame = state.getParentFrame();
         goalStancePointGizmo.getPoseGizmo().setParentFrame(parentFrame);
         goalFocalPointGizmo.getPoseGizmo().setParentFrame(parentFrame);

         for (RobotSide side : RobotSide.values)
            goalFeetGizmos.get(side).setParentFrame(state.getGoalFrame().getReferenceFrame());

         // In this section, we want to update the definition when the gizmos are moved
         // and/or the parent frame is changed. However, on loading and otherwise we want
         // to make sure the current state reflects the definition because the definition
         // can change. We do this by freezing the node so user changes can propagate.
         if (goalStancePointGizmo.getPoseGizmo().getGizmoModifiedByUser().poll()
             || goalFocalPointGizmo.getPoseGizmo().getGizmoModifiedByUser().poll())
         {
            definition.getGoalStancePoint().getValue().set(goalStancePointGizmo.getPoseGizmo().getTransformToParent().getTranslation());
            definition.getGoalFocalPoint().getValue().set(goalFocalPointGizmo.getPoseGizmo().getTransformToParent().getTranslation());
         }
         else
         {
            goalStancePointGizmo.getPoseGizmo().getTransformToParent().getTranslation().set(definition.getGoalStancePoint().getValueReadOnly());
            goalFocalPointGizmo.getPoseGizmo().getTransformToParent().getTranslation().set(definition.getGoalFocalPoint().getValueReadOnly());
         }

         for (RobotSide side : RobotSide.values)
            if (goalFeetGizmos.get(side).getGizmoModifiedByUser().poll())
               state.freeze();

         if (state.isFrozen())
         {
            for (RobotSide side : RobotSide.values)
               state.copyGoalFootstepToGoalTransformToDefinition(side);
         }
         else
         {
            for (RobotSide side : RobotSide.values)
               state.copyDefinitionToGoalFoostepToGoalTransform(side);
         }
         state.getGoalFrame().getReferenceFrame().update();

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
         {
            goalFeetGizmos.get(side).update();
            goalFeetGraphics.get(side).setPose(goalFeetGizmos.get(side).getPose());
         }
      }
   }

   @Override
   public void renderTreeViewIconArea()
   {
      super.renderTreeViewIconArea();

      footstepsWidget.render(ImGui.getFrameHeight());
      ImGui.sameLine();
   }

   @Override
   public void calculateVRPick(RDXVRContext vrContext)
   {
      if (state.areFramesInWorld())
      {
         if (!definition.getIsManuallyPlaced())
         {

         }
      }
   }

   @Override
   public void processVRInput(RDXVRContext vrContext)
   {
      if (state.areFramesInWorld())
      {
         if (!definition.getIsManuallyPlaced())
         {

         }
      }
   }

   @Override
   public void calculate3DViewPick(ImGui3DViewInput input)
   {
      if (state.areFramesInWorld())
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
            {
               if (goalFeetPosesSelected.get(side).get())
               {
                  goalFeetGizmos.get(side).calculate3DViewPick(input);
               }
            }
         }
      }
   }

   @Override
   public void process3DViewInput(ImGui3DViewInput input)
   {
      if (state.areFramesInWorld())
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
            goalStancePointGizmo.process3DViewInput(input);
            goalFocalPointGizmo.process3DViewInput(input);
            tooltip.setInput(input);
            for (RobotSide side : RobotSide.values)
            {
               if (goalFeetPosesSelected.get(side).get())
               {
                  goalFeetGizmos.get(side).process3DViewInput(input);
               }
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

      manuallyPlaceStepsWrapper.renderImGuiWidget();

      if (state.areFramesInWorld()) // Not allowing modification if not renderable
      {
         if (definition.getIsManuallyPlaced())
         {
            ImGui.checkbox(labels.get("Edit Manually Placed Steps"), editManuallyPlacedSteps);
            ImGui.text("Number of footsteps: %d".formatted(manuallyPlacedFootsteps.size()));
            ImGui.text("Add:");
            for (RobotSide side : RobotSide.values)
            {
               ImGui.sameLine();
               if (ImGui.button(labels.get(side.getPascalCaseName())))
                  userAddedFootstep.set(side);
            }
            if (!getState().getFootsteps().isEmpty())
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
            ImGui.checkbox(labels.get("Stance Point"), goalStancePointGizmo.getSelected());
            ImGui.sameLine();
            ImGui.checkbox(labels.get("Focal Point"), goalFocalPointGizmo.getSelected());
            for (RobotSide side : RobotSide.values)
            {
               ImGui.checkbox(labels.get(side.getPascalCaseName() + " Foot to Goal"), goalFeetPosesSelected.get(side));
               if (side == RobotSide.LEFT)
                  ImGui.sameLine();
            }
         }
      }
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
      if (state.areFramesInWorld())
      {
         if (definition.getIsManuallyPlaced())
         {
            for (RDXFootstepPlanActionFootstep footstep : manuallyPlacedFootsteps)
            {
               footstep.getVirtualRenderables(renderables, pool);
            }
         }
         else
         {
            goalArrowGraphic.getRenderables(renderables, pool);

            goalStancePointGizmo.getVirtualRenderables(renderables, pool);
            goalFocalPointGizmo.getVirtualRenderables(renderables, pool);
            for (RobotSide side : RobotSide.values)
            {
               if (goalFeetPosesSelected.get(side).get())
               {
                  goalFeetGizmos.get(side).getRenderables(renderables, pool);
               }
            }
            for (RobotSide side : RobotSide.values)
            {
               goalFeetGraphics.get(side).setHighlighted(footstepsWidget.getIsHovered().get(side));
               goalFeetGraphics.get(side).getRenderables(renderables, pool);
            }
         }
      }
   }

   @Override
   public String getActionTypeTitle()
   {
      return "Footstep Plan";
   }

   public void changeParentFrame(String newParentFrameName)
   {
      FramePoint3D frameStancePoint = new FramePoint3D(state.getParentFrame(), definition.getGoalStancePoint().getValueReadOnly());
      FramePoint3D frameFocalPoint = new FramePoint3D(state.getParentFrame(), definition.getGoalFocalPoint().getValueReadOnly());

      definition.setParentFrameName(newParentFrameName);
      // Freeze to prevent the frame from glitching when changing frames
      state.getGoalFrame().changeFrame(newParentFrameName, state.getGoalToParentTransform().getValueAndFreeze());

      // Keep the points in the same place w.r.t common ancestor frames
      ReferenceFrame newParent = state.getGoalFrame().getReferenceFrame().getParent();
      frameStancePoint.changeFrame(newParent);
      frameFocalPoint.changeFrame(newParent);
      definition.getGoalStancePoint().getValue().set(frameStancePoint);
      definition.getGoalFocalPoint().getValue().set(frameFocalPoint);
      goalStancePointGizmo.getPoseGizmo().setParentFrame(newParent);
      goalFocalPointGizmo.getPoseGizmo().setParentFrame(newParent);
      goalStancePointGizmo.getPoseGizmo().getTransformToParent().getTranslation().set(definition.getGoalStancePoint().getValueReadOnly());
      goalFocalPointGizmo.getPoseGizmo().getTransformToParent().getTranslation().set(definition.getGoalFocalPoint().getValueReadOnly());
      goalStancePointGizmo.getPoseGizmo().update();
      goalFocalPointGizmo.getPoseGizmo().update();

      for (FootstepPlanActionFootstepState footstepState : getState().getFootsteps())
      {
         footstepState.getSoleFrame().changeFrame(newParentFrameName);
      }
   }

   public ImBoolean getEditManuallyPlacedSteps()
   {
      return editManuallyPlacedSteps;
   }
}
