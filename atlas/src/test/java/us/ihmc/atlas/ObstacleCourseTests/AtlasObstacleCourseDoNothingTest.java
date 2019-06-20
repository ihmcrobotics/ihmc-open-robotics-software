package us.ihmc.atlas.ObstacleCourseTests;

import org.junit.jupiter.api.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.obstacleCourseTests.DRCObstacleCourseDoNothingTest;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

public class AtlasObstacleCourseDoNothingTest extends DRCObstacleCourseDoNothingTest
{
   private DRCRobotModel robotModel;

   @Override
   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }

   @Override
   public String getSimpleRobotName()
   {
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.ATLAS);
   }

   @Test
   public void testDoNothing() throws SimulationExceededMaximumTimeException
   {
      robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);
      super.testDoNothing1();
   }

   @Test
   public void testDoNothingShapeCollision() throws SimulationExceededMaximumTimeException
   {
      robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false, false, true)
      {
         @Override
         public double getSimulateDT()
         {
            return 0.0001;
         }
      };
      super.testDoNothing1();
   }
}
