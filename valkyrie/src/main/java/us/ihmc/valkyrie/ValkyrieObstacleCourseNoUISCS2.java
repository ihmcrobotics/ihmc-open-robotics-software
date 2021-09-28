package us.ihmc.valkyrie;

import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.simulationStarter.AvatarSimulationToolsSCS2;
import us.ihmc.avatar.simulationStarter.AvatarSimulationToolsSCS2.AvatarSimulationEnvironment;
import us.ihmc.simulationConstructionSetTools.util.environments.DefaultCommonAvatarEnvironment;
import us.ihmc.valkyrie.configuration.ValkyrieRobotVersion;
import us.ihmc.valkyrieRosControl.ValkyrieRosControlController;

public class ValkyrieObstacleCourseNoUISCS2
{
   // Increase to 10 when you want the sims to run a little faster and don't need the data.
   private final int recordFrequencySpeedup = 1;

   public ValkyrieObstacleCourseNoUISCS2()
   {
      ValkyrieRobotVersion version = ValkyrieRosControlController.VERSION;
      ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS, version);
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
      new ValkyrieObstacleCourseNoUISCS2();
   }
}
