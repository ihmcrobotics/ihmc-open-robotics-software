package us.ihmc.valkyrie.externalForceEstimation;

import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.avatar.contactEstimation.AvatarMeshProjectionVisualizer;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.valkyrie.ValkyrieRobotModel;

import java.util.ArrayList;
import java.util.List;

public class ValkyrieMeshProjectionVisualizer extends AvatarMeshProjectionVisualizer
{
   @Override
   protected DRCRobotModel getRobotModel()
   {
      return new ValkyrieRobotModel(RobotTarget.SCS);
   }

   @Override
   protected RobotCollisionModel getCollisionModel()
   {
      return getRobotModel().getHumanoidRobotKinematicsCollisionModel();
   }

   @Override
   protected List<Pair<RigidBodyBasics, FramePoint3D>> createListOfPointsToProject(FullHumanoidRobotModel fullRobotModel)
   {
      List<Pair<RigidBodyBasics, FramePoint3D>> queryPoints = new ArrayList<>();

      RigidBodyBasics chest = fullRobotModel.getChest();
      queryPoints.add(Pair.of(chest, new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.25, 0.1, 0.2)));
      queryPoints.add(Pair.of(chest, new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.25, 0.0, 0.0)));
      queryPoints.add(Pair.of(chest, new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.25, -0.05, 0.1)));
      queryPoints.add(Pair.of(chest, new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.25, 0.05, 0.4)));
      queryPoints.add(Pair.of(chest, new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.3, -0.3, 0.3)));

      RigidBodyBasics leftForearmLink = fullRobotModel.getOneDoFJointByName("leftForearmYaw").getSuccessor();
      queryPoints.add(Pair.of(leftForearmLink, new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.25, 0.6, 0.4)));
      queryPoints.add(Pair.of(leftForearmLink, new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.25, 0.7, 0.2)));
      queryPoints.add(Pair.of(leftForearmLink, new FramePoint3D(ReferenceFrame.getWorldFrame(), -0.25, 0.75, 0.2)));

      RigidBodyBasics leftHand = fullRobotModel.getHand(RobotSide.LEFT);
      queryPoints.add(Pair.of(leftHand, new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.25, 0.95, 0.35)));

      RigidBodyBasics rightForearmLink = fullRobotModel.getOneDoFJointByName("rightForearmYaw").getSuccessor();
      queryPoints.add(Pair.of(rightForearmLink, new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.25, -0.6, 0.4)));
      queryPoints.add(Pair.of(rightForearmLink, new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.25, -0.7, 0.2)));
      queryPoints.add(Pair.of(rightForearmLink, new FramePoint3D(ReferenceFrame.getWorldFrame(), -0.25, -0.75, 0.2)));

      RigidBodyBasics rightHand = fullRobotModel.getHand(RobotSide.RIGHT);
      queryPoints.add(Pair.of(rightHand, new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.25, -0.9, 0.2)));

      RigidBodyBasics pelvis = fullRobotModel.getPelvis();
      queryPoints.add(Pair.of(pelvis, new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.25, 0.0, -0.1)));
      queryPoints.add(Pair.of(pelvis, new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.25, 0.3, -0.15)));
      queryPoints.add(Pair.of(pelvis, new FramePoint3D(ReferenceFrame.getWorldFrame(), -0.25, 0.1, -0.2)));
      queryPoints.add(Pair.of(pelvis, new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.25, 0.0, -0.3)));

      RigidBodyBasics leftHipPitchLink = fullRobotModel.getOneDoFJointByName("leftHipPitch").getSuccessor();
      queryPoints.add(Pair.of(leftHipPitchLink, new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.0, 0.5, -0.3)));
      queryPoints.add(Pair.of(leftHipPitchLink, new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.1, 0.5, -0.35)));
      queryPoints.add(Pair.of(leftHipPitchLink, new FramePoint3D(ReferenceFrame.getWorldFrame(), -0.1, 0.5, -0.4)));
      queryPoints.add(Pair.of(leftHipPitchLink, new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.3, 0.3, -0.45)));
      queryPoints.add(Pair.of(leftHipPitchLink, new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.2, 0.35, -0.5)));
      queryPoints.add(Pair.of(leftHipPitchLink, new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.1, 0.45, -0.55)));

      RigidBodyBasics rightHipPitchLink = fullRobotModel.getOneDoFJointByName("rightHipPitch").getSuccessor();
      queryPoints.add(Pair.of(rightHipPitchLink, new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.0, -0.5, -0.3)));
      queryPoints.add(Pair.of(rightHipPitchLink, new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.1, -0.5, -0.35)));
      queryPoints.add(Pair.of(rightHipPitchLink, new FramePoint3D(ReferenceFrame.getWorldFrame(), -0.1, -0.5, -0.4)));
      queryPoints.add(Pair.of(rightHipPitchLink, new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.3, -0.3, -0.45)));
      queryPoints.add(Pair.of(rightHipPitchLink, new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.2, -0.35, -0.5)));
      queryPoints.add(Pair.of(rightHipPitchLink, new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.1, -0.45, -0.55)));

      RigidBodyBasics leftKneePitchLink = fullRobotModel.getOneDoFJointByName("leftKneePitch").getSuccessor();
      queryPoints.add(Pair.of(leftKneePitchLink, new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.25, 0.2, -0.7)));
      queryPoints.add(Pair.of(leftKneePitchLink, new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.1, 0.4, -0.75)));
      queryPoints.add(Pair.of(leftKneePitchLink, new FramePoint3D(ReferenceFrame.getWorldFrame(), -0.1, 0.4, -0.8)));
      queryPoints.add(Pair.of(leftKneePitchLink, new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.25, 0.3, -0.85)));
      queryPoints.add(Pair.of(leftKneePitchLink, new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.25, 0.1, -1.1)));

      RigidBodyBasics rightKneePitchLink = fullRobotModel.getOneDoFJointByName("rightKneePitch").getSuccessor();
      queryPoints.add(Pair.of(rightKneePitchLink, new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.25, -0.2, -0.7)));
      queryPoints.add(Pair.of(rightKneePitchLink, new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.1, -0.4, -0.75)));
      queryPoints.add(Pair.of(rightKneePitchLink, new FramePoint3D(ReferenceFrame.getWorldFrame(), -0.1, -0.4, -0.8)));
      queryPoints.add(Pair.of(rightKneePitchLink, new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.25, -0.3, -0.85)));
      queryPoints.add(Pair.of(rightKneePitchLink, new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.25, -0.1, -1.1)));

      return queryPoints;
   }

   public static void main(String[] args)
   {
      new ValkyrieMeshProjectionVisualizer();
   }
}
