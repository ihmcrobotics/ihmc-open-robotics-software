package us.ihmc.valkyrie;

import us.ihmc.darpaRoboticsChallenge.DRCLocalConfigParameters;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.obstacleCourseTests.DRCObstacleCourseFlatTest;

public class ValkyrieObstacleCourseFlatTest extends DRCObstacleCourseFlatTest
{

   private DRCRobotModel robotModel = new ValkyrieRobotModel(DRCLocalConfigParameters.RUNNING_ON_REAL_ROBOT);
   @Override
   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }

}
