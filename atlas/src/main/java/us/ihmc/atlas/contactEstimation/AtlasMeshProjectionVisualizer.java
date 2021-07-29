package us.ihmc.atlas.contactEstimation;

import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.contactEstimation.AvatarMeshProjectionVisualizer;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.physics.RobotCollisionModel;

import java.util.ArrayList;
import java.util.List;

public class AtlasMeshProjectionVisualizer extends AvatarMeshProjectionVisualizer
{
   @Override
   protected DRCRobotModel getRobotModel()
   {
      return new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS);
   }

   @Override
   protected List<Pair<RigidBodyBasics, FramePoint3D>> createListOfPointsToProject(FullHumanoidRobotModel fullRobotModel)
   {
      List<Pair<RigidBodyBasics, FramePoint3D>> queryPoints = new ArrayList<>();

      RigidBodyBasics leftKneePitchLink = fullRobotModel.getOneDoFJointByName("l_leg_kny").getSuccessor();
      queryPoints.add(Pair.of(leftKneePitchLink, new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.2, 0.1, -0.7)));
      queryPoints.add(Pair.of(leftKneePitchLink, new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.2, 0.0, -0.55)));
      queryPoints.add(Pair.of(leftKneePitchLink, new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.2, 0.15, -0.58)));
      queryPoints.add(Pair.of(leftKneePitchLink, new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.08, 0.25, -0.6)));
      queryPoints.add(Pair.of(leftKneePitchLink, new FramePoint3D(ReferenceFrame.getWorldFrame(), -0.08, 0.28, -0.63)));
      queryPoints.add(Pair.of(leftKneePitchLink, new FramePoint3D(ReferenceFrame.getWorldFrame(), -0.15, 0.1, -0.66)));

      RigidBodyBasics rightKneePitchLink = fullRobotModel.getOneDoFJointByName("r_leg_kny").getSuccessor();
      queryPoints.add(Pair.of(rightKneePitchLink, new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.2, -0.1, -0.7)));
      queryPoints.add(Pair.of(rightKneePitchLink, new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.2, 0.0, -0.55)));
      queryPoints.add(Pair.of(rightKneePitchLink, new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.2, -0.15, -0.58)));
      queryPoints.add(Pair.of(rightKneePitchLink, new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.08, -0.25, -0.6)));
      queryPoints.add(Pair.of(rightKneePitchLink, new FramePoint3D(ReferenceFrame.getWorldFrame(), -0.08, -0.28, -0.63)));
      queryPoints.add(Pair.of(rightKneePitchLink, new FramePoint3D(ReferenceFrame.getWorldFrame(), -0.15, -0.1, -0.66)));

      RigidBodyBasics pelvisLink = fullRobotModel.getOneDoFJointByName("l_leg_hpz").getPredecessor();
      queryPoints.add(Pair.of(pelvisLink, new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.2, 0.0, 0.0)));
      queryPoints.add(Pair.of(pelvisLink, new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.2, 0.1, 0.0)));
      queryPoints.add(Pair.of(pelvisLink, new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.1, 0.2, 0.0)));
      queryPoints.add(Pair.of(pelvisLink, new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.0, 0.3, 0.0)));

      return queryPoints;
   }

   @Override
   protected RobotCollisionModel getCollisionModel()
   {
      return new AtlasMultiContactCollisionModel(getRobotModel().getJointMap());
   }

   public static void main(String[] args)
   {
      new AtlasMeshProjectionVisualizer();
   }
}
