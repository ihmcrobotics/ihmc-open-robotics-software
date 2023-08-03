package us.ihmc.valkyrieRosControl.upperBody;

import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.configuration.ValkyrieRobotVersion;

public class ValkyrieUpperBodySimulation
{
   public ValkyrieUpperBodySimulation()
   {
      ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS, ValkyrieRobotVersion.UPPER_BODY);

      // TODO
   }

   public static void main(String[] args)
   {
      new ValkyrieUpperBodySimulation();
   }
}
