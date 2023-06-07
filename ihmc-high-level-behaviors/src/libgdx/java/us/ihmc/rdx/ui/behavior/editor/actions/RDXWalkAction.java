package us.ihmc.rdx.ui.behavior.editor.actions;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.type.ImBoolean;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.behaviors.sequence.actions.WalkActionData;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.rdx.imgui.ImDoubleWrapper;
import us.ihmc.rdx.imgui.ImGuiReferenceFrameLibraryCombo;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.behavior.editor.RDXBehaviorAction;
import us.ihmc.rdx.ui.gizmo.RDXPathControlRingGizmo;
import us.ihmc.rdx.ui.gizmo.RDXPose3DGizmo;
import us.ihmc.rdx.ui.graphics.RDXFootstepGraphic;
import us.ihmc.rdx.ui.graphics.RDXFootstepPlanGraphic;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class RDXWalkAction extends RDXBehaviorAction
{
   private final WalkActionData actionData = new WalkActionData();
   private final RDXFootstepPlanGraphic footstepPlanGraphic;
   private final ImGuiReferenceFrameLibraryCombo referenceFrameLibraryCombo;
   private final SideDependentList<RDXFootstepGraphic> goalFeetGraphics = new SideDependentList<>();
   private final RDXPathControlRingGizmo footstepPlannerGoalGizmo = new RDXPathControlRingGizmo(actionData.getReferenceFrame(),
                                                                                                actionData.getTransformToParent());
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final SideDependentList<ImBoolean> goalFeetPosesSelected = new SideDependentList<>();
   private final SideDependentList<RDXPose3DGizmo> goalFeetGizmos = new SideDependentList<>();
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
      actionData.setReferenceFrameLibrary(referenceFrameLibrary);
      footstepPlanGraphic = new RDXFootstepPlanGraphic(robotModel.getContactPointParameters().getControllerFootGroundContactPoints());
      referenceFrameLibraryCombo = new ImGuiReferenceFrameLibraryCombo(referenceFrameLibrary);

      footstepPlannerGoalGizmo.create(panel3D);
      FootstepPlannerParametersBasics footstepPlannerParameters = robotModel.getFootstepPlannerParameters();

      for (RobotSide side : RobotSide.values)
      {
         goalFeetPosesSelected.put(side, new ImBoolean(false));

         RDXPose3DGizmo footGizmo = new RDXPose3DGizmo(actionData.getGoalFootstepToParentTransforms().get(side), actionData.getReferenceFrame());
         footGizmo.create(panel3D);
         goalFeetGizmos.put(side, footGizmo);

         RDXFootstepGraphic goalFootGraphic = new RDXFootstepGraphic(robotModel.getContactPointParameters().getControllerFootGroundContactPoints(), side);
         goalFootGraphic.create();
         goalFeetGraphics.put(side, goalFootGraphic);

         // Set initial placement of goal feet poses
         FramePose3D goalFootPose = new FramePose3D();
         goalFootPose.setToZero(actionData.getReferenceFrame());
         goalFootPose.getPosition().addY(0.5 * side.negateIfRightSide(footstepPlannerParameters.getIdealFootstepWidth()));
         goalFootPose.get(footGizmo.getTransformToParent());
      }
   }

   @Override
   public void updateAfterLoading()
   {
      referenceFrameLibraryCombo.setSelectedReferenceFrame(actionData.getParentReferenceFrame().getName());
      footstepPlannerGoalGizmo.setGizmoFrame(actionData.getReferenceFrame());
      for (RobotSide side : RobotSide.values)
      {
         goalFeetGizmos.get(side).setParentFrame(actionData.getReferenceFrame());
      }
   }

   @Override
   public void update()
   {
      if (!getSelected().get())
         goalFeetPosesSelected.forEach(imBoolean -> imBoolean.set(false));

      footstepPlannerGoalGizmo.update();
      for (RobotSide side : RobotSide.values)
      {
         goalFeetGizmos.get(side).update();
         goalFeetGraphics.get(side).setPose(goalFeetGizmos.get(side).getPose());
      }
      footstepPlanGraphic.update();
   }

   @Override
   public void calculate3DViewPick(ImGui3DViewInput input)
   {
      if (getSelected().get())
      {
         boolean goalFootEditingEnabled = false;
         for (RobotSide side : RobotSide.values)
         {
            if (goalFeetPosesSelected.get(side).get())
            {
               goalFootEditingEnabled = true;
               goalFeetGizmos.get(side).calculate3DViewPick(input);
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
            if (goalFeetPosesSelected.get(side).get())
            {
               goalFootEditingEnabled = true;
               goalFeetGizmos.get(side).process3DViewInput(input);
            }
         }
         if (!goalFootEditingEnabled)
            footstepPlannerGoalGizmo.process3DViewInput(input);
      }
   }

   @Override
   public void renderImGuiSettingWidgets()
   {
      if (referenceFrameLibraryCombo.render())
      {
         actionData.changeParentFrameWithoutMoving(referenceFrameLibraryCombo.getSelectedReferenceFrame());
         footstepPlannerGoalGizmo.setGizmoFrame(actionData.getReferenceFrame());
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

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      footstepPlanGraphic.getRenderables(renderables, pool);
      if (getSelected().get())
      {
         footstepPlannerGoalGizmo.getRenderables(renderables, pool);
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
   public WalkActionData getActionData()
   {
      return actionData;
   }

   @Override
   public String getNameForDisplay()
   {
      return "Walk Goal";
   }
}
