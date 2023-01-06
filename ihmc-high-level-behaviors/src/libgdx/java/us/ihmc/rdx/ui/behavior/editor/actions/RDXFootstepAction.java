package us.ihmc.rdx.ui.behavior.editor.actions;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.sequence.ReferenceFrameLibrary;
import us.ihmc.behaviors.sequence.actions.FootstepActionData;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.affordances.RDXInteractableHighlightModel;
import us.ihmc.rdx.ui.affordances.RDXInteractableTools;
import us.ihmc.rdx.ui.behavior.editor.ImGuiReferenceFrameLibraryCombo;
import us.ihmc.rdx.ui.behavior.editor.RDXBehaviorAction;
import us.ihmc.rdx.ui.gizmo.RDXPose3DGizmo;
import us.ihmc.robotics.robotSide.RobotSide;

public class RDXFootstepAction extends RDXBehaviorAction
{
   private final FootstepActionData actionData = new FootstepActionData();
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private RDXInteractableHighlightModel ankleFrameHighlightModel;
   private final RDXPose3DGizmo solePoseGizmo;
   private final DRCRobotModel robotModel;
   private final ImGuiReferenceFrameLibraryCombo referenceFrameLibraryCombo;
   private final ROS2SyncedRobotModel syncedRobot;
   private final RigidBodyTransform soleTransformToWorld = new RigidBodyTransform();
   private final RigidBodyTransform ankleToSoleFrameTransform = new RigidBodyTransform();
   private final boolean wasInitializedToPreviousStep;

   public RDXFootstepAction(RDX3DPanel panel3D,
                            DRCRobotModel robotModel,
                            ROS2SyncedRobotModel syncedRobot,
                            ReferenceFrameLibrary referenceFrameLibrary,
                            RDXFootstepAction possiblyNullPreviousFootstepAction)
   {
      this.syncedRobot = syncedRobot;
      this.robotModel = robotModel;
      referenceFrameLibraryCombo = new ImGuiReferenceFrameLibraryCombo(referenceFrameLibrary);
      solePoseGizmo = new RDXPose3DGizmo(actionData.getTransformToParent(), ReferenceFrame.getWorldFrame());
      solePoseGizmo.create(panel3D);

      wasInitializedToPreviousStep = possiblyNullPreviousFootstepAction != null;
      if (wasInitializedToPreviousStep)
      {
         setToReferenceFrame(possiblyNullPreviousFootstepAction.getReferenceFrame());
      }
   }

   @Override
   public void updateAfterLoading()
   {
      referenceFrameLibraryCombo.setSelectedReferenceFrame(actionData.getParentFrameName());
      solePoseGizmo.setParentFrame(referenceFrameLibraryCombo.getSelectedReferenceFrame());
   }

   public void setSide(RobotSide side, boolean authoring)
   {
      actionData.setSide(side);
      String footBodyName = syncedRobot.getFullRobotModel().getFoot(side).getName();
      String modelFileName = RDXInteractableTools.getModelFileName(robotModel.getRobotDefinition().getRigidBodyDefinition(footBodyName));
      ankleFrameHighlightModel = new RDXInteractableHighlightModel(modelFileName);
      ankleToSoleFrameTransform.set(robotModel.getJointMap().getSoleToParentFrameTransform(side));
      ankleToSoleFrameTransform.invert();

      if (!wasInitializedToPreviousStep && authoring)
      {
         setToReferenceFrame(syncedRobot.getReferenceFrames().getSoleFrame(side));
      }
   }

   @Override
   public void update()
   {
      solePoseGizmo.updateTransforms();
      solePoseGizmo.getGizmoFrame().getTransformToDesiredFrame(soleTransformToWorld, ReferenceFrame.getWorldFrame());
      ankleFrameHighlightModel.setPose(soleTransformToWorld, ankleToSoleFrameTransform);
      actionData.setParentFrameName(referenceFrameLibraryCombo.getSelectedReferenceFrame().getName());
   }

   @Override
   public void calculate3DViewPick(ImGui3DViewInput input)
   {
      if (getSelected().get())
      {
         solePoseGizmo.calculate3DViewPick(input);
      }
   }

   @Override
   public void process3DViewInput(ImGui3DViewInput input)
   {
      if (getSelected().get())
      {
         solePoseGizmo.process3DViewInput(input);
      }
   }

   @Override
   public void renderImGuiSettingWidgets()
   {
      if (referenceFrameLibraryCombo.combo())
      {
         FramePose3D poseToKeep = new FramePose3D();
         poseToKeep.setToZero(solePoseGizmo.getGizmoFrame());
         solePoseGizmo.setParentFrame(referenceFrameLibraryCombo.getSelectedReferenceFrame());
         poseToKeep.changeFrame(solePoseGizmo.getGizmoFrame().getParent());
         poseToKeep.get(solePoseGizmo.getTransformToParent());
      }
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      ankleFrameHighlightModel.getRenderables(renderables, pool);
      if (getSelected().get())
         solePoseGizmo.getRenderables(renderables, pool);
   }

   @Override
   public FootstepActionData getActionData()
   {
      return actionData;
   }

   private void setToReferenceFrame(ReferenceFrame referenceFrame)
   {
      if (referenceFrameLibraryCombo.setSelectedReferenceFrame(referenceFrame.getName()))
      {
         solePoseGizmo.setParentFrame(referenceFrameLibraryCombo.getSelectedReferenceFrame());
         solePoseGizmo.getTransformToParent().set(referenceFrame.getTransformToParent());
      }
      else
      {
         solePoseGizmo.setParentFrame(ReferenceFrame.getWorldFrame());
         solePoseGizmo.getTransformToParent().set(referenceFrame.getTransformToWorldFrame());
      }
   }

   @Override
   public String getNameForDisplay()
   {
      return actionData.getSide().getPascalCaseName() + " Footstep";
   }

   public ReferenceFrame getReferenceFrame()
   {
      return solePoseGizmo.getGizmoFrame();
   }
}
