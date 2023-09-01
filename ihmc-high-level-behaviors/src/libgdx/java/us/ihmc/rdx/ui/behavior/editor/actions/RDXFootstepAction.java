package us.ihmc.rdx.ui.behavior.editor.actions;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.sequence.actions.FootstepActionData;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.rdx.imgui.ImGuiReferenceFrameLibraryCombo;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.affordances.RDXInteractableHighlightModel;
import us.ihmc.rdx.ui.affordances.RDXInteractableTools;
import us.ihmc.rdx.ui.behavior.editor.RDXBehaviorAction;
import us.ihmc.rdx.ui.gizmo.RDXPose3DGizmo;
import us.ihmc.robotics.referenceFrames.ModifiableReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class RDXFootstepAction extends RDXBehaviorAction
{
   private final FootstepActionData actionData = new FootstepActionData();
   private final RDXPose3DGizmo solePoseGizmo = new RDXPose3DGizmo(actionData.getReferenceFrame(), actionData.getTransformToParent());
   private final ImGuiReferenceFrameLibraryCombo referenceFrameLibraryCombo;
   private final SideDependentList<RigidBodyTransform> ankleToSoleFrameTransforms = new SideDependentList<>();
   private final ModifiableReferenceFrame graphicFrame = new ModifiableReferenceFrame(actionData.getReferenceFrame());
   private final SideDependentList<RDXInteractableHighlightModel> highlightModels = new SideDependentList<>();

   public RDXFootstepAction(RDX3DPanel panel3D, DRCRobotModel robotModel, ROS2SyncedRobotModel syncedRobot, ReferenceFrameLibrary referenceFrameLibrary)
   {
      actionData.setReferenceFrameLibrary(referenceFrameLibrary);

      for (RobotSide side : RobotSide.values)
      {
         RigidBodyTransform ankleToSoleFrameTransform = new RigidBodyTransform(robotModel.getJointMap().getSoleToParentFrameTransform(side));
         ankleToSoleFrameTransform.invert();
         ankleToSoleFrameTransforms.put(side, ankleToSoleFrameTransform);
         graphicFrame.update(transformToParent -> transformToParent.set(ankleToSoleFrameTransform));

         String footBodyName = syncedRobot.getFullRobotModel().getFoot(side).getName();
         String modelFileName = RDXInteractableTools.getModelFileName(robotModel.getRobotDefinition().getRigidBodyDefinition(footBodyName));
         highlightModels.put(side, new RDXInteractableHighlightModel(modelFileName));
      }

      referenceFrameLibraryCombo = new ImGuiReferenceFrameLibraryCombo(referenceFrameLibrary);
      solePoseGizmo.create(panel3D);
   }

   @Override
   public void updateAfterLoading()
   {
      referenceFrameLibraryCombo.setSelectedReferenceFrame(actionData.getParentReferenceFrame().getName());
   }

   public void setSide(RobotSide side)
   {
      actionData.setSide(side);
   }

   public void setIncludingFrame(ReferenceFrame parentFrame, RigidBodyTransform transformToParent)
   {
      actionData.changeParentFrame(parentFrame);
      actionData.setTransformToParent(transformToParentToPack -> transformToParentToPack.set(transformToParent));
      update();
   }

   public void setToReferenceFrame(ReferenceFrame referenceFrame)
   {
      actionData.changeParentFrame(ReferenceFrame.getWorldFrame());
      actionData.setTransformToParent(transformToParentToPack -> transformToParentToPack.set(referenceFrame.getTransformToWorldFrame()));
      update();
   }

   @Override
   public void update()
   {
      if (solePoseGizmo.getGizmoFrame() != actionData.getReferenceFrame())
      {
         solePoseGizmo.setGizmoFrame(actionData.getReferenceFrame());
         graphicFrame.changeParentFrame(actionData.getReferenceFrame());
      }

      solePoseGizmo.update();
      highlightModels.get(actionData.getSide()).setPose(graphicFrame.getReferenceFrame());
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
      if (referenceFrameLibraryCombo.render())
      {
         actionData.changeParentFrameWithoutMoving(referenceFrameLibraryCombo.getSelectedReferenceFrame().get());
         update();
      }
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      highlightModels.get(actionData.getSide()).getRenderables(renderables, pool);
      if (getSelected().get())
         solePoseGizmo.getRenderables(renderables, pool);
   }

   @Override
   public String getActionTypeTitle()
   {
      return actionData.getSide().getPascalCaseName() + " Footstep";
   }

   public ReferenceFrame getReferenceFrame()
   {
      return solePoseGizmo.getGizmoFrame();
   }

   @Override
   public FootstepActionData getActionData()
   {
      return actionData;
   }
}
