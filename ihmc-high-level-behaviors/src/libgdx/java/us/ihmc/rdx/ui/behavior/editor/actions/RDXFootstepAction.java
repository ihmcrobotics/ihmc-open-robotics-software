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
   private RDXInteractableHighlightModel highlightModel;
   private final RDXPose3DGizmo poseGizmo = new RDXPose3DGizmo();
   private final DRCRobotModel robotModel;
   private final ImGuiReferenceFrameLibraryCombo referenceFrameLibraryCombo;
   private final ROS2SyncedRobotModel syncedRobot;
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();
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
      poseGizmo.create(panel3D);

      wasInitializedToPreviousStep = possiblyNullPreviousFootstepAction != null;
      if (wasInitializedToPreviousStep)
      {
         setToReferenceFrame(possiblyNullPreviousFootstepAction.getReferenceFrame());
      }
   }

   public void setSide(RobotSide side, boolean authoring)
   {
      actionData.setSide(side);
      String footBodyName = syncedRobot.getFullRobotModel().getFoot(side).getName();
      String modelFileName = RDXInteractableTools.getModelFileName(robotModel.getRobotDefinition().getRigidBodyDefinition(footBodyName));
      highlightModel = new RDXInteractableHighlightModel(modelFileName);
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
      poseGizmo.updateTransforms();
      poseGizmo.getGizmoFrame().getTransformToDesiredFrame(tempTransform, ReferenceFrame.getWorldFrame());
      highlightModel.setPose(tempTransform, ankleToSoleFrameTransform);
   }

   @Override
   public void calculate3DViewPick(ImGui3DViewInput input)
   {
      if (getSelected().get())
      {
         poseGizmo.calculate3DViewPick(input);
      }
   }

   @Override
   public void process3DViewInput(ImGui3DViewInput input)
   {
      if (getSelected().get())
      {
         poseGizmo.process3DViewInput(input);
      }
   }

   @Override
   public void renderImGuiSettingWidgets()
   {
      if (referenceFrameLibraryCombo.combo())
      {
         FramePose3D poseToKeep = new FramePose3D();
         poseToKeep.setToZero(poseGizmo.getGizmoFrame());
         poseGizmo.setParentFrame(referenceFrameLibraryCombo.getSelectedReferenceFrame());
         poseToKeep.changeFrame(poseGizmo.getGizmoFrame().getParent());
         poseToKeep.get(poseGizmo.getTransformToParent());
      }
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      highlightModel.getRenderables(renderables, pool);
      if (getSelected().get())
         poseGizmo.getRenderables(renderables, pool);
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
         poseGizmo.setParentFrame(referenceFrameLibraryCombo.getSelectedReferenceFrame());
         poseGizmo.getTransformToParent().set(referenceFrame.getTransformToParent());
      }
      else
      {
         poseGizmo.setParentFrame(ReferenceFrame.getWorldFrame());
         poseGizmo.getTransformToParent().set(referenceFrame.getTransformToWorldFrame());
      }
   }

   private void setReferenceFrame(String referenceFrameName)
   {
      if (referenceFrameLibraryCombo.setSelectedReferenceFrame(referenceFrameName))
      {
         poseGizmo.setParentFrame(referenceFrameLibraryCombo.getSelectedReferenceFrame());
      }
   }

   @Override
   public String getNameForDisplay()
   {
      return actionData.getSide().getPascalCaseName() + " Footstep";
   }

   public ReferenceFrame getReferenceFrame()
   {
      return poseGizmo.getGizmoFrame();
   }
}
