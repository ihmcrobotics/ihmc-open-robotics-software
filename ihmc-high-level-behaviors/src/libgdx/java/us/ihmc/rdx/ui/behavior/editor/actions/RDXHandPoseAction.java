package us.ihmc.rdx.ui.behavior.editor.actions;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import imgui.ImGui;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.sequence.ReferenceFrameLibrary;
import us.ihmc.behaviors.sequence.actions.HandPoseActionData;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.rdx.imgui.ImDoubleWrapper;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.affordances.RDXInteractableHighlightModel;
import us.ihmc.rdx.ui.affordances.RDXInteractableTools;
import us.ihmc.rdx.ui.behavior.editor.ImGuiReferenceFrameLibraryCombo;
import us.ihmc.rdx.ui.behavior.editor.RDXBehaviorAction;
import us.ihmc.rdx.ui.gizmo.RDXPose3DGizmo;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.referenceFrames.ModifiableReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class RDXHandPoseAction extends RDXBehaviorAction
{
   private final HandPoseActionData actionData = new HandPoseActionData();
   private RDXInteractableHighlightModel highlightModel;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   /** Gizmo is control frame */
   private final RDXPose3DGizmo poseGizmo = new RDXPose3DGizmo();
   private final SideDependentList<String> handNames = new SideDependentList<>();
   private final SideDependentList<RigidBodyTransform> handControlTransformToHandFrames = new SideDependentList<>();
   private final SideDependentList<RigidBodyTransform> handGraphicTransformToHandFrames = new SideDependentList<>();
   private final ModifiableReferenceFrame handFrame;
   private final ModifiableReferenceFrame graphicFrame;
   private final DRCRobotModel robotModel;
   private final ImGuiReferenceFrameLibraryCombo referenceFrameLibraryCombo;
   private final ROS2SyncedRobotModel syncedRobot;
   private final ImDoubleWrapper trajectoryDurationWidget = new ImDoubleWrapper(actionData::getTrajectoryDuration,
                                                                                actionData::setTrajectoryDuration,
                                                                                imDouble -> ImGui.inputDouble(labels.get("Trajectory duration"), imDouble));

   public RDXHandPoseAction(RDX3DPanel panel3D,
                            DRCRobotModel robotModel,
                            ROS2SyncedRobotModel syncedRobot,
                            FullHumanoidRobotModel fullRobotModel,
                            ReferenceFrameLibrary referenceFrameLibrary)
   {
      this.robotModel = robotModel;
      this.syncedRobot = syncedRobot;

      for (RobotSide side : RobotSide.values)
      {
         handNames.put(side, fullRobotModel.getHand(side).getName());
         handControlTransformToHandFrames.put(side, fullRobotModel.getHandControlFrame(side).getTransformToParent());
         handGraphicTransformToHandFrames.put(side, robotModel.getUIParameters().getHandGraphicToHandFrameTransform(side));
      }
      referenceFrameLibraryCombo = new ImGuiReferenceFrameLibraryCombo(referenceFrameLibrary);
      poseGizmo.create(panel3D);

      handFrame = new ModifiableReferenceFrame(poseGizmo.getGizmoFrame());
      graphicFrame = new ModifiableReferenceFrame(handFrame.getReferenceFrame());
   }

   public void setSide(RobotSide side, boolean authoring, RDXHandPoseAction possiblyNullPreviousAction)
   {
      actionData.setSide(side);
      handFrame.getTransformToParent().set(handControlTransformToHandFrames.get(side));
      handFrame.getTransformToParent().invert(); // We actually need hand to control
      handFrame.getReferenceFrame().update();
      graphicFrame.getTransformToParent().set(handGraphicTransformToHandFrames.get(side));
      graphicFrame.getReferenceFrame().update();

      String handBodyName = handNames.get(side);
      String modelFileName = RDXInteractableTools.getModelFileName(robotModel.getRobotDefinition().getRigidBodyDefinition(handBodyName));
      highlightModel = new RDXInteractableHighlightModel(modelFileName);

      if (possiblyNullPreviousAction != null)
      {
         setToReferenceFrame(possiblyNullPreviousAction.getReferenceFrame());
      }
      else if (authoring)
      {
         setToReferenceFrame(syncedRobot.getReferenceFrames().getHandFrame(side));
      }
   }

   @Override
   public void update()
   {
      poseGizmo.updateTransforms();
      highlightModel.setPose(graphicFrame.getReferenceFrame());
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
         changeDerivativeFrames();
      }
      ImGui.pushItemWidth(80.0f);
      trajectoryDurationWidget.renderImGuiWidget();
      ImGui.popItemWidth();
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      highlightModel.getRenderables(renderables, pool);
      if (getSelected().get())
         poseGizmo.getRenderables(renderables, pool);
   }

   @Override
   public void saveToFile(ObjectNode jsonNode)
   {
      actionData.saveToFile(jsonNode);
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
      actionData.loadFromFile(jsonNode);
   }

   private void setToReferenceFrame(ReferenceFrame referenceFrame)
   {
      if (referenceFrameLibraryCombo.setSelectedReferenceFrame(referenceFrame.getName()))
      {
         poseGizmo.setParentFrame(referenceFrameLibraryCombo.getSelectedReferenceFrame());
         poseGizmo.getTransformToParent().set(referenceFrame.getTransformToParent());
         changeDerivativeFrames();
      }
      else
      {
         poseGizmo.setParentFrame(ReferenceFrame.getWorldFrame());
         poseGizmo.getTransformToParent().set(referenceFrame.getTransformToWorldFrame());
         changeDerivativeFrames();
      }
   }

   private void setReferenceFrame(String referenceFrameName)
   {
      if (referenceFrameLibraryCombo.setSelectedReferenceFrame(referenceFrameName))
      {
         poseGizmo.setParentFrame(referenceFrameLibraryCombo.getSelectedReferenceFrame());
         changeDerivativeFrames();
      }
   }

   /**
    * This must be done when the gizmo is reattached to different frames,
    * because ReferenceFrames have to entirely recreated when changing parent
    * frames.
    */
   private void changeDerivativeFrames()
   {
      handFrame.changeParentFrame(poseGizmo.getGizmoFrame());
      graphicFrame.changeParentFrame(handFrame.getReferenceFrame());
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

   public HandPoseActionData getActionData()
   {
      return actionData;
   }
}
