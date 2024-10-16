package us.ihmc.atlas;

import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.simulationStarter.AvatarSimulationToolsSCS2;
import us.ihmc.avatar.simulationStarter.AvatarSimulationToolsSCS2.AvatarSimulationEnvironment;
import us.ihmc.commons.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.util.environments.DefaultCommonAvatarEnvironment;
import us.ihmc.wholeBodyController.AdditionalSimulationContactPoints;
import us.ihmc.wholeBodyController.FootContactPoints;

public class AtlasObstacleCourseNoUISCS2
{
   // Increase to 10 when you want the sims to run a little faster and don't need the data.
   private final int recordFrequencySpeedup = 1;

   // Set to true if you are walking over steps or things where your foot might overhang a little.
   private boolean addExtraContactPoints = false;

   public AtlasObstacleCourseNoUISCS2()
   {
      FootContactPoints<RobotSide> simulationContactPoints = null;
      if (addExtraContactPoints)
      {
         int nContactPointsX = 5;
         int nContactPointsY = 4;
         boolean edgePointsOnly = true;
         simulationContactPoints = new AdditionalSimulationContactPoints<>(RobotSide.values, nContactPointsX, nContactPointsY, edgePointsOnly, false);
      }

      AtlasRobotVersion version = AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ;
      AtlasRobotModel robotModel = AtlasRobotModelFactory.createDefaultRobotModel(version, simulationContactPoints);
      DefaultCommonAvatarEnvironment environment = new DefaultCommonAvatarEnvironment();

      AvatarSimulationEnvironment simEnvironment = AvatarSimulationToolsSCS2.setupSimulationEnvironmentWithGraphicSelector(robotModel,
                                                                                                                           environment,
                                                                                                                           null,
                                                                                                                           null,
                                                                                                                           DRCObstacleCourseStartingLocation.values());
      if (simEnvironment != null)
      {
         simEnvironment.getAvatarSimulationFactory().setSimulationDataRecordTimePeriod(recordFrequencySpeedup * robotModel.getControllerDT());
         simEnvironment.build();
         simEnvironment.start();
      }
   }

   public static void main(String[] args)
   {
      new AtlasObstacleCourseNoUISCS2();
   }
}
