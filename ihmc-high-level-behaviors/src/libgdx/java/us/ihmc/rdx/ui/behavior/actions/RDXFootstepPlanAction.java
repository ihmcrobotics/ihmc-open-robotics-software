package us.ihmc.rdx.ui.behavior.actions;

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
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.rdx.imgui.ImBooleanWrapper;
import us.ihmc.rdx.imgui.ImDoubleWrapper;
import us.ihmc.rdx.imgui.ImGuiReferenceFrameLibraryCombo;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.ui.RDX3DPanelTooltip;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.behavior.sequence.RDXActionNode;
import us.ihmc.rdx.ui.gizmo.RDXPose3DGizmo;
import us.ihmc.rdx.ui.gizmo.RDXSelectablePathControlRingGizmo;
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
   private final ImBoolean showAdjustmentInteractables = new ImBoolean();
   private final ImBooleanWrapper manuallyPlaceStepsWrapper;
   private final ImDoubleWrapper swingDurationWidget;
   private final ImDoubleWrapper transferDurationWidget;
   private int numberOfAllocatedFootsteps = 0;
   private final RecyclingArrayList<RDXFootstepPlanActionFootstep> manuallyPlacedFootsteps;
   private final TypedNotification<RobotSide> userAddedFootstep = new TypedNotification<>();
   private final Notification userRemovedFootstep = new Notification();
   private final SideDependentList<RDXFootstepGraphic> goalFeetGraphics = new SideDependentList<>();
   private final RDXSelectablePathControlRingGizmo footstepPlannerGoalGizmo;
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

      footstepPlannerGoalGizmo = new RDXSelectablePathControlRingGizmo(ReferenceFrame.getWorldFrame(),
                                                                       state.getGoalToParentTransform(),
                                                                       showAdjustmentInteractables);
      footstepPlannerGoalGizmo.create(baseUI.getPrimary3DPanel());

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
      // Set the definition from what we adjusted by the gizmo
      // However, we check the goal frame is not null, which will be the case
      if (state.getGoalFrame().getReferenceFrame() != null)
      {
         definition.getGoalToParentX().setValue(state.getGoalToParentTransform().getTranslation().getX());
         definition.getGoalToParentY().setValue(state.getGoalToParentTransform().getTranslation().getY());
         definition.getGoalToParentYaw().setValue(state.getGoalToParentTransform().getRotation().getYaw());
      }

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

         if (footstepPlannerGoalGizmo.getPathControlRingGizmo().getGizmoFrame() != state.getGoalFrame().getReferenceFrame())
         {
            footstepPlannerGoalGizmo.getPathControlRingGizmo().setGizmoFrame(state.getGoalFrame().getReferenceFrame());
         }

         for (RobotSide side : RobotSide.values)
         {
            if (goalFeetGizmos.get(side).getGizmoFrame().getParent() != state.getGoalFrame().getReferenceFrame())
            {
               goalFeetGizmos.get(side).setParentFrame(state.getGoalFrame().getReferenceFrame());
            }
         }

         if (!showAdjustmentInteractables.get())
            goalFeetPosesSelected.forEach(imBoolean -> imBoolean.set(false));

         footstepPlannerGoalGizmo.getPathControlRingGizmo().update();

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
      if (state.getGoalFrame().isChildOfWorld())
      {
         if (!definition.getIsManuallyPlaced())
         {
            footstepPlannerGoalGizmo.calculateVRPick(vrContext);
         }
      }
   }

   @Override
   public void processVRInput(RDXVRContext vrContext)
   {
      if (state.getGoalFrame().isChildOfWorld())
      {
         if (!definition.getIsManuallyPlaced())
         {
            footstepPlannerGoalGizmo.processVRInput(vrContext);
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
            footstepPlannerGoalGizmo.calculate3DViewPick(input);
            if (showAdjustmentInteractables.get())
            {
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
            footstepPlannerGoalGizmo.process3DViewInput(input);
            tooltip.setInput(input);
            if (showAdjustmentInteractables.get())
            {
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
   }

   @Override
   protected void renderImGuiWidgetsInternal()
   {
      ImGui.checkbox(labels.get("Show Adjustment Interactables"), showAdjustmentInteractables);
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
            for (RobotSide side : RobotSide.values)
            {
               ImGui.checkbox(labels.get("Edit Goal " + side.getPascalCaseName()), goalFeetPosesSelected.get(side));
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
         if (footstepPlannerGoalGizmo.getPathControlRingGizmo().getRingHovered())
         {
            tooltip.render("%s Action\nIndex: %d\nName: %s".formatted(getActionTypeTitle(), state.getActionIndex(), definition.getName()));
         }
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
            footstepPlannerGoalGizmo.getVirtualRenderables(renderables, pool);
            if (showAdjustmentInteractables.get())
            {
               for (RobotSide side : RobotSide.values)
               {
                  if (goalFeetPosesSelected.get(side).get())
                  {
                     goalFeetGizmos.get(side).getRenderables(renderables, pool);
                  }
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
      definition.setParentFrameName(newParentFrameName);
      for (FootstepPlanActionFootstepState footstepState : getState().getFootsteps())
      {
         footstepState.getSoleFrame().changeFrame(newParentFrameName);
      }
   }

   public ImBoolean getShowAdjustmentInteractables()
   {
      return showAdjustmentInteractables;
   }
}
