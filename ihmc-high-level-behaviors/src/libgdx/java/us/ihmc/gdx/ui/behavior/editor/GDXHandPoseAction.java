package us.ihmc.gdx.ui.behavior.editor;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import controller_msgs.msg.dds.FrameInformation;
import controller_msgs.msg.dds.HandTrajectoryMessage;
import controller_msgs.msg.dds.SE3TrajectoryPointMessage;
import imgui.type.ImBoolean;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.gdx.FocusBasedGDXCamera;
import us.ihmc.gdx.input.ImGui3DViewInput;
import us.ihmc.gdx.ui.affordances.GDXInteractableHighlightModel;
import us.ihmc.gdx.ui.gizmo.GDXPose3DGizmo;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;

public class GDXHandPoseAction implements GDXBehaviorAction
{
   private RigidBodyTransform controlToHandTranform;
   private final RigidBodyTransform handGraphicToControlTransform = new RigidBodyTransform();
   private GDXInteractableHighlightModel highlightModel;
   private final GDXPose3DGizmo poseGizmo = new GDXPose3DGizmo();
   private RobotSide side;
   private ROS2ControllerHelper ros2ControllerHelper;
   private ImBoolean selected = new ImBoolean();

   public void create(FocusBasedGDXCamera camera3D,
                      DRCRobotModel robotModel,
                      FullHumanoidRobotModel fullRobotModel,
                      RobotSide side,
                      ROS2ControllerHelper ros2ControllerHelper)
   {
      this.side = side;
      this.ros2ControllerHelper = ros2ControllerHelper;
      ReferenceFrame handControlFrame = fullRobotModel.getHandControlFrame(side);
      controlToHandTranform = handControlFrame.getTransformToParent();
      handGraphicToControlTransform.setAndInvert(controlToHandTranform);
      handGraphicToControlTransform.getRotation().appendYawRotation(side == RobotSide.LEFT ? 0.0 : Math.PI);
      handGraphicToControlTransform.getRotation().appendPitchRotation(-Math.PI / 2.0);
      handGraphicToControlTransform.getRotation().appendRollRotation(0.0);
      handGraphicToControlTransform.getTranslation().add(0.126, -0.00179, 0.0); // TODO: Fix and check
      highlightModel = new GDXInteractableHighlightModel((side == RobotSide.LEFT) ? "palm.g3dj" : "palmRight.g3dj");
      poseGizmo.create(camera3D);
   }

   @Override
   public void process3DViewInput(ImGui3DViewInput input)
   {
      if (selected.get())
      {
         poseGizmo.process3DViewInput(input);
         highlightModel.setPose(poseGizmo.getTransformToParent(), handGraphicToControlTransform);
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
   public void destroy()
   {
      highlightModel.dispose();
   }

   public void moveHand(double trajectoryTime)
   {
      Pose3D endHandPose = new Pose3D(poseGizmo.getTransformToParent());
      HandTrajectoryMessage handTrajectoryMessage = new HandTrajectoryMessage();
      handTrajectoryMessage.setRobotSide(side.toByte());
      handTrajectoryMessage.getSe3Trajectory().getFrameInformation().setTrajectoryReferenceFrameId(FrameInformation.CHEST_FRAME);
      handTrajectoryMessage.getSe3Trajectory().getFrameInformation().setDataReferenceFrameId(FrameInformation.WORLD_FRAME);
      SE3TrajectoryPointMessage trajectoryPoint = handTrajectoryMessage.getSe3Trajectory().getTaskspaceTrajectoryPoints().add();
      trajectoryPoint.setTime(trajectoryTime);
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

   @Override
   public String getNameForDisplay()
   {
      return side.getPascalCaseName() + " Hand Pose";
   }
}
