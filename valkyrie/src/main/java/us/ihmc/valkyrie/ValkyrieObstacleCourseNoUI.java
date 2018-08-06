package us.ihmc.valkyrie;

import com.martiansoftware.jsap.JSAPException;
import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.simulationStarter.DRCSimulationStarter;
import us.ihmc.avatar.simulationStarter.DRCSimulationTools;
import us.ihmc.simulationConstructionSetTools.util.environments.DefaultCommonAvatarEnvironment;

public class ValkyrieObstacleCourseNoUI
{
   public static void main(final String[] args) throws JSAPException
   {
      DRCRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS, false);
      DRCSimulationStarter simulationStarter = new DRCSimulationStarter(robotModel, new DefaultCommonAvatarEnvironment());
      simulationStarter.setRunMultiThreaded(true);

      DRCSimulationTools.startSimulationWithGraphicSelector(simulationStarter, null, null, DRCObstacleCourseStartingLocation.values());
   }
}