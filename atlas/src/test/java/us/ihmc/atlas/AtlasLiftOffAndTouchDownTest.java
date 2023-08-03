package us.ihmc.atlas;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import us.ihmc.atlas.parameters.AtlasPhysicalProperties;
import us.ihmc.atlas.parameters.AtlasWalkingControllerParameters;
import us.ihmc.avatar.AvatarLiftOffAndTouchDownTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulation;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.robotics.Assert;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

public class AtlasLiftOffAndTouchDownTest
{
   private final double footLength = new AtlasPhysicalProperties().getFootLengthForControl();

   @Tag("humanoid-flat-ground")
   @Test
   public void testForwardStepRotated() throws SimulationExceededMaximumTimeException
   {
      DRCRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS)
      {
         @Override
         public WalkingControllerParameters getWalkingControllerParameters()
         {
            return new AtlasWalkingControllerParameters(getTarget(), getJointMap(), getContactPointParameters())
            {
               @Override
               public double nominalHeightAboveAnkle()
               {
                  return 0.849;
               }
            };
         }
      };
      SCS2AvatarTestingSimulation testHelper = AvatarLiftOffAndTouchDownTest.setupTest(robotModel, Math.toRadians(90.0));

      double stepLength = 0.4;
      double startPitch = Math.toRadians(20.0);
      double finalPitch = Math.toRadians(-20.0);

      boolean success = AvatarLiftOffAndTouchDownTest.doStep(robotModel, testHelper, stepLength, startPitch, finalPitch, footLength);

      testHelper.finishTest();

      Assert.assertTrue("A check failed. See console output.", success);
   }
}
