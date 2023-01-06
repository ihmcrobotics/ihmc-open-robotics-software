package us.ihmc.rdx.ui.behavior.editor.actions;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.type.ImBoolean;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.behaviors.sequence.ReferenceFrameLibrary;
import us.ihmc.behaviors.sequence.actions.WalkActionData;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.rdx.imgui.ImDoubleWrapper;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.behavior.editor.ImGuiReferenceFrameLibraryCombo;
import us.ihmc.rdx.ui.behavior.editor.RDXBehaviorAction;
import us.ihmc.rdx.ui.gizmo.RDXPathControlRingGizmo;
import us.ihmc.rdx.ui.gizmo.RDXPose3DGizmo;
import us.ihmc.rdx.ui.graphics.RDXFootstepGraphic;
import us.ihmc.rdx.ui.graphics.RDXFootstepPlanGraphic;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class RDXWalkAction extends RDXBehaviorAction
{
   private final WalkActionData actionData = new WalkActionData();
   private final RDXFootstepPlanGraphic footstepPlanGraphic;
   private final ImGuiReferenceFrameLibraryCombo referenceFrameLibraryCombo;
   private final SideDependentList<RDXFootstepGraphic> goalFeetGraphics = new SideDependentList<>();
   private final SideDependentList<FramePose3D> goalFeetPoses = new SideDependentList<>();
   private final RDXPathControlRingGizmo footstepPlannerGoalGizmo;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final SideDependentList<ImBoolean> editGoalFootPoses = new SideDependentList<>();
   private final SideDependentList<RDXPose3DGizmo> editGoalFootGizmos = new SideDependentList<>();
   private final ImDoubleWrapper swingDurationWidget = new ImDoubleWrapper(actionData::getSwingDuration,
                                                                           actionData::setSwingDuration,
                                                                           imDouble -> ImGui.inputDouble(labels.get("Swing duration"), imDouble));
   private final ImDoubleWrapper transferDurationWidget = new ImDoubleWrapper(actionData::getTransferDuration,
                                                                              actionData::setTransferDuration,
                                                                              imDouble -> ImGui.inputDouble(labels.get("Transfer duration"), imDouble));

   public RDXWalkAction(RDX3DPanel panel3D,
                        DRCRobotModel robotModel,
                        ReferenceFrameLibrary referenceFrameLibrary)
   {
      footstepPlanGraphic = new RDXFootstepPlanGraphic(robotModel.getContactPointParameters().getControllerFootGroundContactPoints());
      referenceFrameLibraryCombo = new ImGuiReferenceFrameLibraryCombo(referenceFrameLibrary);

      footstepPlannerGoalGizmo = new RDXPathControlRingGizmo(actionData.getTransformToParent(), ReferenceFrame.getWorldFrame());
      footstepPlannerGoalGizmo.create(panel3D.getCamera3D());
      FootstepPlannerParametersBasics footstepPlannerParameters = robotModel.getFootstepPlannerParameters();

      for (RobotSide side : RobotSide.values)
      {
         editGoalFootPoses.put(side, new ImBoolean(false));
         RDXPose3DGizmo footGizmo = new RDXPose3DGizmo(actionData.getGoalFootstepToGizmos().get(side), footstepPlannerGoalGizmo.getGizmoFrame());
         footGizmo.create(panel3D);
         editGoalFootGizmos.put(side, footGizmo);

         RDXFootstepGraphic goalFootGraphic = new RDXFootstepGraphic(robotModel.getContactPointParameters().getControllerFootGroundContactPoints(),
                                                                     side);
         goalFootGraphic.create();
         goalFeetGraphics.put(side, goalFootGraphic);
         FramePose3D goalFootPose = new FramePose3D();
         goalFootPose.setToZero(footstepPlannerGoalGizmo.getGizmoFrame());
         goalFootPose.getPosition().addY(0.5 * side.negateIfRightSide(footstepPlannerParameters.getIdealFootstepWidth()));
         goalFootPose.get(footGizmo.getTransformToParent());
         goalFootPose.changeFrame(ReferenceFrame.getWorldFrame());
         goalFootGraphic.setPose(goalFootPose);
         goalFeetPoses.put(side, goalFootPose);
      }
   }

   @Override
   public void update()
   {
      if (!getSelected().get())
         editGoalFootPoses.forEach(imBoolean -> imBoolean.set(false));

      footstepPlannerGoalGizmo.updateTransforms();
      for (RobotSide side : RobotSide.values)
      {
         editGoalFootGizmos.get(side).updateTransforms();

         FramePose3D goalFootPose = goalFeetPoses.get(side);
         goalFootPose.setToZero(editGoalFootGizmos.get(side).getGizmoFrame());
         goalFootPose.changeFrame(ReferenceFrame.getWorldFrame());
         goalFeetGraphics.get(side).setPose(goalFootPose);
      }
      footstepPlanGraphic.update();

      actionData.setParentFrameName(referenceFrameLibraryCombo.getSelectedReferenceFrame().getName());
   }

   @Override
   public void updateAfterLoading()
   {
      referenceFrameLibraryCombo.setSelectedReferenceFrame(actionData.getParentFrameName());
      updateParentFrame(referenceFrameLibraryCombo.getSelectedReferenceFrame());
   }

   @Override
   public void calculate3DViewPick(ImGui3DViewInput input)
   {
      if (getSelected().get())
      {
         boolean goalFootEditingEnabled = false;
         for (RobotSide side : RobotSide.values)
         {
            if (editGoalFootPoses.get(side).get())
            {
               goalFootEditingEnabled = true;
               editGoalFootGizmos.get(side).calculate3DViewPick(input);
            }
         }
         if (!goalFootEditingEnabled)
            footstepPlannerGoalGizmo.calculate3DViewPick(input);
      }
   }

   @Override
   public void process3DViewInput(ImGui3DViewInput input)
   {
      if (getSelected().get())
      {
         boolean goalFootEditingEnabled = false;
         for (RobotSide side : RobotSide.values)
         {
            if (editGoalFootPoses.get(side).get())
            {
               goalFootEditingEnabled = true;
               editGoalFootGizmos.get(side).process3DViewInput(input);
            }
         }
         if (!goalFootEditingEnabled)
            footstepPlannerGoalGizmo.process3DViewInput(input);
      }
   }

   @Override
   public void renderImGuiSettingWidgets()
   {
      if (referenceFrameLibraryCombo.combo())
      {
         FramePose3D poseToKeep = new FramePose3D();
         poseToKeep.setToZero(footstepPlannerGoalGizmo.getGizmoFrame());
         updateParentFrame(referenceFrameLibraryCombo.getSelectedReferenceFrame());
         poseToKeep.changeFrame(footstepPlannerGoalGizmo.getGizmoFrame().getParent());
         poseToKeep.get(footstepPlannerGoalGizmo.getTransformToParent());
      }
      if (ImGui.button(labels.get("Plan")))
      {
         // TODO: Plan preview message
      }
      ImGui.sameLine();
      for (RobotSide side : RobotSide.values)
      {
         ImGui.checkbox(labels.get("Edit " + side.getPascalCaseName()), editGoalFootPoses.get(side));
         if (side == RobotSide.LEFT)
            ImGui.sameLine();
      }
      ImGui.pushItemWidth(80.0f);
      swingDurationWidget.renderImGuiWidget();
      transferDurationWidget.renderImGuiWidget();
      ImGui.popItemWidth();
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      footstepPlanGraphic.getRenderables(renderables, pool);
      if (getSelected().get())
      {
         footstepPlannerGoalGizmo.getRenderables(renderables, pool);
         for (RobotSide side : RobotSide.values)
         {
            if (editGoalFootPoses.get(side).get())
            {
               editGoalFootGizmos.get(side).getRenderables(renderables, pool);
            }
         }
      }
      for (RobotSide side : RobotSide.values)
         goalFeetGraphics.get(side).getRenderables(renderables, pool);
   }

   @Override
   public WalkActionData getActionData()
   {
      return actionData;
   }

   private void updateParentFrame(ReferenceFrame newParentFrame)
   {
      footstepPlannerGoalGizmo.setParentFrame(newParentFrame);
      for (RobotSide side : RobotSide.values)
      {
         editGoalFootGizmos.get(side).setParentFrame(footstepPlannerGoalGizmo.getGizmoFrame());
      }
   }

   @Override
   public String getNameForDisplay()
   {
      return "Walk Goal";
   }
}
