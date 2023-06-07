package us.ihmc.rdx.ui.behavior.editor.actions;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.behaviors.sequence.actions.HandPoseActionData;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.log.LogTools;
import us.ihmc.rdx.imgui.ImDoubleWrapper;
import us.ihmc.rdx.imgui.ImGuiReferenceFrameLibraryCombo;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.affordances.RDXInteractableHighlightModel;
import us.ihmc.rdx.ui.affordances.RDXInteractableTools;
import us.ihmc.rdx.ui.behavior.editor.RDXBehaviorAction;
import us.ihmc.rdx.ui.gizmo.RDXPose3DGizmo;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.EuclidCoreMissingTools;
import us.ihmc.robotics.referenceFrames.ModifiableReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.wholeBodyController.HandTransformTools;

public class RDXHandPoseAction extends RDXBehaviorAction
{
   private final HandPoseActionData actionData = new HandPoseActionData();
   private final SideDependentList<RDXInteractableHighlightModel> highlightModels = new SideDependentList<>();
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   /** Gizmo is control frame */
   private final RDXPose3DGizmo poseGizmo = new RDXPose3DGizmo(actionData.getReferenceFrame(), actionData.getTransformToParent());
   private final SideDependentList<String> handNames = new SideDependentList<>();
   private final SideDependentList<RigidBodyTransform> handGraphicTransformToControlFrames = new SideDependentList<>();
   private final ModifiableReferenceFrame graphicFrame = new ModifiableReferenceFrame(actionData.getReferenceFrame());
   private final ImGuiReferenceFrameLibraryCombo referenceFrameLibraryCombo;
   private final ImDoubleWrapper trajectoryDurationWidget = new ImDoubleWrapper(actionData::getTrajectoryDuration,
                                                                                actionData::setTrajectoryDuration,
                                                                                imDouble -> ImGui.inputDouble(labels.get("Trajectory duration"), imDouble));

   public RDXHandPoseAction(RDX3DPanel panel3D, DRCRobotModel robotModel, FullHumanoidRobotModel fullRobotModel, ReferenceFrameLibrary referenceFrameLibrary)
   {
      actionData.setReferenceFrameLibrary(referenceFrameLibrary);

      for (RobotSide side : RobotSide.values)
      {
         handNames.put(side, fullRobotModel.getHand(side).getName());

         RigidBodyTransform graphicToControlFrameTransform = new RigidBodyTransform();
         HandTransformTools.getHandGraphicToControlFrameTransform(fullRobotModel, robotModel.getUIParameters(), side, graphicToControlFrameTransform);
         handGraphicTransformToControlFrames.put(side, graphicToControlFrameTransform);
         graphicFrame.update(transformToParent -> transformToParent.set(handGraphicTransformToControlFrames.get(side)));

         String handBodyName = handNames.get(side);
         String modelFileName = RDXInteractableTools.getModelFileName(robotModel.getRobotDefinition().getRigidBodyDefinition(handBodyName));
         highlightModels.put(side, new RDXInteractableHighlightModel(modelFileName));
      }
      referenceFrameLibraryCombo = new ImGuiReferenceFrameLibraryCombo(referenceFrameLibrary);
      poseGizmo.create(panel3D);
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
   }

   public void setToReferenceFrame(ReferenceFrame referenceFrame)
   {
      actionData.changeParentFrame(ReferenceFrame.getWorldFrame());
      actionData.setTransformToParent(transformToParent -> referenceFrame.getTransformToWorldFrame());
   }

   @Override
   public void update()
   {
      if (poseGizmo.getGizmoFrame() != actionData.getReferenceFrame())
      {
         poseGizmo.setGizmoFrame(actionData.getReferenceFrame());
         graphicFrame.changeParentFrame(actionData.getReferenceFrame());
      }

      poseGizmo.update();
      highlightModels.get(actionData.getSide()).setPose(graphicFrame.getReferenceFrame());
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
      if (referenceFrameLibraryCombo.render())
      {
         actionData.changeParentFrameWithoutMoving(referenceFrameLibraryCombo.getSelectedReferenceFrame());
      }
      ImGui.pushItemWidth(80.0f);
      trajectoryDurationWidget.renderImGuiWidget();
      ImGui.popItemWidth();
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      highlightModels.get(actionData.getSide()).getRenderables(renderables, pool);
      if (getSelected().get())
         poseGizmo.getRenderables(renderables, pool);
   }

   @Override
   public String getNameForDisplay()
   {
      return actionData.getSide().getPascalCaseName() + " Hand Pose";
   }

   public ReferenceFrame getReferenceFrame()
   {
      return poseGizmo.getGizmoFrame();
   }

   @Override
   public HandPoseActionData getActionData()
   {
      return actionData;
   }
}
