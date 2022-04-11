package us.ihmc.gdx.ui.behavior.editor;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import controller_msgs.msg.dds.FrameInformation;
import controller_msgs.msg.dds.HandTrajectoryMessage;
import controller_msgs.msg.dds.SE3TrajectoryPointMessage;
import imgui.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImDouble;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.gdx.GDXFocusBasedCamera;
import us.ihmc.gdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.gdx.input.ImGui3DViewInput;
import us.ihmc.gdx.ui.affordances.GDXInteractableHighlightModel;
import us.ihmc.gdx.ui.affordances.GDXInteractableTools;
import us.ihmc.gdx.ui.gizmo.GDXPose3DGizmo;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.io.JSONTools;

import java.util.List;

public class GDXHandPoseAction implements GDXBehaviorAction
{
   private RigidBodyTransform controlToHandTranform;
   private final RigidBodyTransform handGraphicToControlTransform = new RigidBodyTransform();
   private GDXInteractableHighlightModel highlightModel;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final GDXPose3DGizmo poseGizmo = new GDXPose3DGizmo();
   private RobotSide side;
   private FullHumanoidRobotModel fullRobotModel;
   private DRCRobotModel robotModel;
   private ImGuiReferenceFrameLibraryCombo referenceFrameLibraryCombo;
   private ROS2SyncedRobotModel syncedRobot;
   private ROS2ControllerHelper ros2ControllerHelper;
   private final ImBoolean selected = new ImBoolean();
   private final ImDouble trajectoryTime = new ImDouble(4.0);
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();

   public void create(GDXFocusBasedCamera camera3D,
                      DRCRobotModel robotModel,
                      ROS2SyncedRobotModel syncedRobot,
                      FullHumanoidRobotModel fullRobotModel,
                      ROS2ControllerHelper ros2ControllerHelper,
                      List<ReferenceFrame> referenceFrameLibrary)
   {
      this.ros2ControllerHelper = ros2ControllerHelper;
      this.fullRobotModel = fullRobotModel;
      this.robotModel = robotModel;
      this.syncedRobot = syncedRobot;
      referenceFrameLibraryCombo = new ImGuiReferenceFrameLibraryCombo(referenceFrameLibrary);
      poseGizmo.create(camera3D);
   }

   public void setSide(RobotSide side, boolean authoring, GDXHandPoseAction possiblyNullPreviousAction)
   {
      this.side = side;
      ReferenceFrame handControlFrame = fullRobotModel.getHandControlFrame(side);
      controlToHandTranform = handControlFrame.getTransformToParent();
      handGraphicToControlTransform.setAndInvert(controlToHandTranform);
      handGraphicToControlTransform.getRotation().appendYawRotation(side == RobotSide.LEFT ? 0.0 : Math.PI);
      handGraphicToControlTransform.getRotation().appendPitchRotation(-Math.PI / 2.0);
      handGraphicToControlTransform.getRotation().appendRollRotation(0.0);
      handGraphicToControlTransform.getTranslation().add(0.126, -0.00179, 0.0); // TODO: Fix and check
      String handBodyName = (side == RobotSide.LEFT) ? "l_hand" : "r_hand";
      String modelFileName = GDXInteractableTools.getModelFileName(robotModel.getRobotDefinition().getRigidBodyDefinition(handBodyName));
      highlightModel = new GDXInteractableHighlightModel(modelFileName);

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
      poseGizmo.getGizmoFrame().getTransformToDesiredFrame(tempTransform, ReferenceFrame.getWorldFrame());
      highlightModel.setPose(tempTransform, handGraphicToControlTransform);
   }

   @Override
   public void calculate3DViewPick(ImGui3DViewInput input)
   {
      if (selected.get())
      {
         poseGizmo.calculate3DViewPick(input);
      }
   }

   @Override
   public void process3DViewInput(ImGui3DViewInput input)
   {
      if (selected.get())
      {
         poseGizmo.process3DViewInput(input);
      }
   }

   @Override
   public void renderImGuiWidgets()
   {
      if (referenceFrameLibraryCombo.combo())
      {
         FramePose3D poseToKeep = new FramePose3D();
         poseToKeep.setToZero(poseGizmo.getGizmoFrame());
         poseGizmo.setParentFrame(referenceFrameLibraryCombo.getSelectedReferenceFrame());
         poseToKeep.changeFrame(poseGizmo.getGizmoFrame().getParent());
         poseToKeep.get(poseGizmo.getTransformToParent());
      }
      ImGui.pushItemWidth(80.0f);
      ImGui.inputDouble(labels.get("Trajectory time"), trajectoryTime);
      ImGui.popItemWidth();
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      highlightModel.getRenderables(renderables, pool);
      if (selected.get())
         poseGizmo.getRenderables(renderables, pool);
   }

   @Override
   public void saveToFile(ObjectNode jsonNode)
   {
      jsonNode.put("parentFrame", poseGizmo.getGizmoFrame().getParent().getName());
      jsonNode.put("side", side.getLowerCaseName());
      jsonNode.put("trajectoryTime", trajectoryTime.get());
      JSONTools.toJSON(jsonNode, poseGizmo.getTransformToParent());
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
      String referenceFrameName = jsonNode.get("parentFrame").asText();
      setReferenceFrame(referenceFrameName);
      setSide(RobotSide.getSideFromString(jsonNode.get("side").asText()), false, null);
      trajectoryTime.set(jsonNode.get("trajectoryTime").asDouble());
      JSONTools.toEuclid(jsonNode, poseGizmo.getTransformToParent());
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
   public void destroy()
   {
      highlightModel.dispose();
   }

   @Override
   public void performAction()
   {
      FramePose3D endHandPose = new FramePose3D();
      endHandPose.setToZero(poseGizmo.getGizmoFrame());
      endHandPose.changeFrame(ReferenceFrame.getWorldFrame());
      HandTrajectoryMessage handTrajectoryMessage = new HandTrajectoryMessage();
      handTrajectoryMessage.setRobotSide(side.toByte());
      handTrajectoryMessage.getSe3Trajectory().getFrameInformation().setTrajectoryReferenceFrameId(FrameInformation.CHEST_FRAME);
      handTrajectoryMessage.getSe3Trajectory().getFrameInformation().setDataReferenceFrameId(FrameInformation.WORLD_FRAME);
      SE3TrajectoryPointMessage trajectoryPoint = handTrajectoryMessage.getSe3Trajectory().getTaskspaceTrajectoryPoints().add();
      trajectoryPoint.setTime(trajectoryTime.get());
      trajectoryPoint.getPosition().set(endHandPose.getPosition());
      trajectoryPoint.getOrientation().set(endHandPose.getOrientation());
      trajectoryPoint.getLinearVelocity().set(EuclidCoreTools.zeroVector3D);
      trajectoryPoint.getAngularVelocity().set(EuclidCoreTools.zeroVector3D);
      ros2ControllerHelper.publishToController(handTrajectoryMessage);
   }

   @Override
   public ImBoolean getSelected()
   {
      return selected;
   }

   public RobotSide getSide()
   {
      return side;
   }

   @Override
   public String getNameForDisplay()
   {
      return side.getPascalCaseName() + " Hand Pose";
   }

   public ReferenceFrame getReferenceFrame()
   {
      return poseGizmo.getGizmoFrame();
   }
}
