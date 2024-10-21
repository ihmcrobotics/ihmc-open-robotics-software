package us.ihmc.atlas;

import us.ihmc.avatar.AvatarStairsSimulation;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.wholeBodyController.AdditionalSimulationContactPoints;
import us.ihmc.wholeBodyController.FootContactPoints;

public class AtlasStairsSimulation
{
   public AtlasStairsSimulation()
   {
      FootContactPoints<RobotSide> simulationContactPoints = new AdditionalSimulationContactPoints<>(RobotSide.values, 8, 3, true, true);
      AtlasRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS,
                                                       RobotTarget.SCS,
                                                       false,
                                                       simulationContactPoints,
                                                       false,
                                                       false);

      AvatarStairsSimulation stairsSimulation = new AvatarStairsSimulation();

      stairsSimulation.setPlaceRobotAtTop(true);
      Pose3D startingPose = new Pose3D();
      startingPose.getPosition().set(0.0, 0.0, 0.15);
      startingPose.getOrientation().setYawPitchRoll(Math.toRadians(-45.0), 0.0, 0.0);
      stairsSimulation.setStartingPose(startingPose);

      stairsSimulation.setRobotModel(robotModel);
      stairsSimulation.startSimulation();
   }

   public static void main(String[] args)
   {
      new AtlasStairsSimulation();
   }
}
