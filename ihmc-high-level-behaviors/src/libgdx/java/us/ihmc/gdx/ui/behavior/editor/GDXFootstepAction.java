package us.ihmc.gdx.ui.behavior.editor;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import controller_msgs.msg.dds.FootstepDataListMessage;
import imgui.type.ImBoolean;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.footstepPlanning.FootstepDataMessageConverter;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.gdx.GDXFocusBasedCamera;
import us.ihmc.gdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.gdx.input.ImGui3DViewInput;
import us.ihmc.gdx.ui.GDX3DPanel;
import us.ihmc.gdx.ui.affordances.GDXInteractableHighlightModel;
import us.ihmc.gdx.ui.affordances.GDXInteractableTools;
import us.ihmc.gdx.ui.gizmo.GDXPose3DGizmo;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.io.JSONTools;

import java.util.List;
import java.util.UUID;

public class GDXFootstepAction implements GDXBehaviorAction
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private GDXInteractableHighlightModel highlightModel;
   private final GDXPose3DGizmo poseGizmo = new GDXPose3DGizmo();
   private RobotSide side;
   private DRCRobotModel robotModel;
   private ImGuiReferenceFrameLibraryCombo referenceFrameLibraryCombo;
   private ROS2SyncedRobotModel syncedRobot;
   private ROS2ControllerHelper ros2ControllerHelper;
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();
   private final RigidBodyTransform ankleToSoleFrameTransform = new RigidBodyTransform();
   private final ImBoolean selected = new ImBoolean();
   private boolean wasInitializedToPreviousStep;

   public void create(GDX3DPanel panel3D,
                      DRCRobotModel robotModel,
                      ROS2SyncedRobotModel syncedRobot,
                      ROS2ControllerHelper ros2ControllerHelper,
                      List<ReferenceFrame> referenceFrameLibrary,
                      GDXFootstepAction possiblyNullPreviousFootstepAction)
   {
      this.syncedRobot = syncedRobot;
      this.ros2ControllerHelper = ros2ControllerHelper;
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
      this.side = side;
      String footBodyName = (side == RobotSide.LEFT) ? "l_foot" : "r_foot";
      String modelFileName = GDXInteractableTools.getModelFileName(robotModel.getRobotDefinition().getRigidBodyDefinition(footBodyName));
      highlightModel = new GDXInteractableHighlightModel(modelFileName);
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
      JSONTools.toJSON(jsonNode, poseGizmo.getTransformToParent());
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
      String referenceFrameName = jsonNode.get("parentFrame").asText();
      setReferenceFrame(referenceFrameName);
      setSide(RobotSide.getSideFromString(jsonNode.get("side").asText()), false);
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
   public void performAction()
   {
      double swingDuration = 1.2;
      double transferDuration = 0.8;
      FootstepPlan footstepPlan = new FootstepPlan();
      footstepPlan.addFootstep(side, poseGizmo.getPose());
      FootstepDataListMessage footstepDataListMessage = FootstepDataMessageConverter.createFootstepDataListFromPlan(footstepPlan,
                                                                                                                    swingDuration,
                                                                                                                    transferDuration);
      footstepDataListMessage.getQueueingProperties().setExecutionMode(ExecutionMode.OVERRIDE.toByte());
      footstepDataListMessage.getQueueingProperties().setMessageId(UUID.randomUUID().getLeastSignificantBits());
      ros2ControllerHelper.publishToController(footstepDataListMessage);
   }

   @Override
   public void destroy()
   {
      highlightModel.dispose();
   }

   @Override
   public ImBoolean getSelected()
   {
      return selected;
   }

   @Override
   public String getNameForDisplay()
   {
      return side.getPascalCaseName() + " Footstep";
   }

   public ReferenceFrame getReferenceFrame()
   {
      return poseGizmo.getGizmoFrame();
   }
}
