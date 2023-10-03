package us.ihmc.rdx.ui.behavior.editor.actions;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.type.ImBoolean;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.behaviors.sequence.actions.WalkActionDefinition;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.rdx.imgui.ImBooleanWrapper;
import us.ihmc.rdx.imgui.ImDoubleWrapper;
import us.ihmc.rdx.imgui.ImGuiReferenceFrameLibraryCombo;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.RDX3DPanelTooltip;
import us.ihmc.rdx.ui.behavior.editor.RDXBehaviorAction;
import us.ihmc.rdx.ui.gizmo.RDXPose3DGizmo;
import us.ihmc.rdx.ui.gizmo.RDXSelectablePathControlRingGizmo;
import us.ihmc.rdx.ui.graphics.RDXFootstepGraphic;
import us.ihmc.rdx.ui.graphics.RDXFootstepPlanGraphic;
import us.ihmc.rdx.vr.RDXVRContext;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class RDXWalkAction extends RDXBehaviorAction
{
   private final ReferenceFrameLibrary referenceFrameLibrary;

   private final WalkActionDefinition actionDefinition = new WalkActionDefinition();
   private final RDXFootstepPlanGraphic footstepPlanGraphic;
   private final ImGuiReferenceFrameLibraryCombo referenceFrameLibraryCombo;
   private final SideDependentList<RDXFootstepGraphic> goalFeetGraphics = new SideDependentList<>();
   private final RDXSelectablePathControlRingGizmo footstepPlannerGoalGizmo = new RDXSelectablePathControlRingGizmo(actionDefinition.getConditionalReferenceFrame().get(),
                                                                                                                    actionDefinition.getTransformToParent());
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImBooleanWrapper selectedWrapper = new ImBooleanWrapper(footstepPlannerGoalGizmo::getSelected,
                                                                         footstepPlannerGoalGizmo::setSelected,
                                                                         imBoolean -> ImGui.checkbox(labels.get("Selected"), imBoolean));
   private final SideDependentList<ImBoolean> goalFeetPosesSelected = new SideDependentList<>();
   private final SideDependentList<RDXPose3DGizmo> goalFeetGizmos = new SideDependentList<>();
   private final ImDoubleWrapper swingDurationWidget = new ImDoubleWrapper(actionDefinition::getSwingDuration,
                                                                           actionDefinition::setSwingDuration,
                                                                           imDouble -> ImGui.inputDouble(labels.get("Swing duration"), imDouble));
   private final ImDoubleWrapper transferDurationWidget = new ImDoubleWrapper(actionDefinition::getTransferDuration,
                                                                              actionDefinition::setTransferDuration,
                                                                              imDouble -> ImGui.inputDouble(labels.get("Transfer duration"), imDouble));
   private final RDX3DPanelTooltip tooltip;

   public RDXWalkAction(RDX3DPanel panel3D,
                        DRCRobotModel robotModel,
                        ReferenceFrameLibrary referenceFrameLibrary)
   {
      this.referenceFrameLibrary = referenceFrameLibrary;
      footstepPlanGraphic = new RDXFootstepPlanGraphic(robotModel.getContactPointParameters().getControllerFootGroundContactPoints());
      referenceFrameLibraryCombo = new ImGuiReferenceFrameLibraryCombo(referenceFrameLibrary);

      footstepPlannerGoalGizmo.create(panel3D);
      FootstepPlannerParametersBasics footstepPlannerParameters = robotModel.getFootstepPlannerParameters();

      tooltip = new RDX3DPanelTooltip(panel3D);
      panel3D.addImGuiOverlayAddition(this::render3DPanelImGuiOverlays);

      for (RobotSide side : RobotSide.values)
      {
         goalFeetPosesSelected.put(side, new ImBoolean(false));

         RDXPose3DGizmo footGizmo = new RDXPose3DGizmo(actionDefinition.getGoalFootstepToParentTransforms().get(side), actionDefinition.getConditionalReferenceFrame().get());
         footGizmo.create(panel3D);
         goalFeetGizmos.put(side, footGizmo);

         RDXFootstepGraphic goalFootGraphic = new RDXFootstepGraphic(robotModel.getContactPointParameters().getControllerFootGroundContactPoints(), side);
         goalFootGraphic.create();
         goalFeetGraphics.put(side, goalFootGraphic);

         // Set initial placement of goal feet poses
         FramePose3D goalFootPose = new FramePose3D();
         goalFootPose.setToZero(actionDefinition.getConditionalReferenceFrame().get());
         goalFootPose.getPosition().addY(0.5 * side.negateIfRightSide(footstepPlannerParameters.getIdealFootstepWidth()));
         goalFootPose.get(footGizmo.getTransformToParent());
      }
   }

   @Override
   public void updateAfterLoading()
   {
      referenceFrameLibraryCombo.setSelectedReferenceFrame(actionDefinition.getConditionalReferenceFrame());
      footstepPlannerGoalGizmo.getPathControlRingGizmo().setGizmoFrame(actionDefinition.getConditionalReferenceFrame().get());
      for (RobotSide side : RobotSide.values)
      {
         goalFeetGizmos.get(side).setParentFrame(actionDefinition.getConditionalReferenceFrame().get());
      }
   }

   public void setIncludingFrame(ReferenceFrame parentFrame, RigidBodyTransform transformToParent)
   {
      actionDefinition.getConditionalReferenceFrame().setParentFrameName(parentFrame.getName());
      actionDefinition.setTransformToParent(transformToParent);
      update();
   }

   public void setToReferenceFrame(ReferenceFrame referenceFrame)
   {
      actionDefinition.getConditionalReferenceFrame().setParentFrameName(ReferenceFrame.getWorldFrame().getName());
      actionDefinition.setTransformToParent(referenceFrame.getTransformToWorldFrame());
      update();
   }

   @Override
   public void update(boolean concurrentActionIsNextForExecution)
   {
      actionDefinition.update(referenceFrameLibrary);

      if (!getSelected().get())
         goalFeetPosesSelected.forEach(imBoolean -> imBoolean.set(false));

      if (footstepPlannerGoalGizmo.getPathControlRingGizmo().getGizmoFrame() != actionDefinition.getConditionalReferenceFrame().get())
      {
         footstepPlannerGoalGizmo.getPathControlRingGizmo().setGizmoFrame(actionDefinition.getConditionalReferenceFrame().get());
         for (RobotSide side : RobotSide.values)
         {
            goalFeetGizmos.get(side).setParentFrame(actionDefinition.getConditionalReferenceFrame().get());
         }
      }

      footstepPlannerGoalGizmo.getPathControlRingGizmo().update();
      for (RobotSide side : RobotSide.values)
      {
         goalFeetGizmos.get(side).update();
         goalFeetGraphics.get(side).setPose(goalFeetGizmos.get(side).getPose());
      }
      footstepPlanGraphic.update();
   }

   @Override
   public void calculateVRPick(RDXVRContext vrContext)
   {
      footstepPlannerGoalGizmo.calculateVRPick(vrContext);
   }

   @Override
   public void processVRInput(RDXVRContext vrContext)
   {
      footstepPlannerGoalGizmo.processVRInput(vrContext);
   }

   @Override
   public void calculate3DViewPick(ImGui3DViewInput input)
   {
      footstepPlannerGoalGizmo.calculate3DViewPick(input);
      if (getSelected().get())
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

   @Override
   public void process3DViewInput(ImGui3DViewInput input)
   {
      footstepPlannerGoalGizmo.process3DViewInput(input);
      tooltip.setInput(input);
      if (getSelected().get())
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

   @Override
   public void renderImGuiSettingWidgets()
   {
      if (referenceFrameLibraryCombo.render())
      {
         actionDefinition.getConditionalReferenceFrame().setParentFrameName(referenceFrameLibraryCombo.getSelectedReferenceFrame().getParent().getName());
      }
      if (ImGui.button(labels.get("Plan")))
      {
         // TODO: Plan preview message
      }
      ImGui.sameLine();
      for (RobotSide side : RobotSide.values)
      {
         ImGui.checkbox(labels.get("Edit " + side.getPascalCaseName()), goalFeetPosesSelected.get(side));
         if (side == RobotSide.LEFT)
            ImGui.sameLine();
      }
      ImGui.pushItemWidth(80.0f);
      swingDurationWidget.renderImGuiWidget();
      transferDurationWidget.renderImGuiWidget();
      ImGui.popItemWidth();
   }

   public void render3DPanelImGuiOverlays()
   {
      if (footstepPlannerGoalGizmo.getPathControlRingGizmo().getRingHovered())
      {
         tooltip.render("%s Action\nIndex: %d\nDescription: %s".formatted(getActionTypeTitle(),
                                                                          getActionIndex(),
                                                                          actionDefinition.getDescription()));
      }
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      footstepPlanGraphic.getRenderables(renderables, pool);
      footstepPlannerGoalGizmo.getVirtualRenderables(renderables, pool);
      if (getSelected().get())
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
         goalFeetGraphics.get(side).getRenderables(renderables, pool);
   }

   @Override
   public ImBooleanWrapper getSelected()
   {
      return selectedWrapper;
   }

   @Override
   public WalkActionDefinition getActionDefinition()
   {
      return actionDefinition;
   }

   @Override
   public String getActionTypeTitle()
   {
      return "Walk Goal";
   }
}
