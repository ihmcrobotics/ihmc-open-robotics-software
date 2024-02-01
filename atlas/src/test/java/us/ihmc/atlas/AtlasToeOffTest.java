package us.ihmc.atlas;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestInfo;

import us.ihmc.atlas.parameters.AtlasToeOffParameters;
import us.ihmc.atlas.parameters.AtlasWalkingControllerParameters;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.obstacleCourseTests.AvatarToeOffTest;
import us.ihmc.commonWalkingControlModules.configurations.ToeOffParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;

public class AtlasToeOffTest extends AvatarToeOffTest
{
   @Test
   @Override
   public void testShortSteps(TestInfo testInfo)
   {
      // The end of swing struggles because of the workspace limiter causing the foot to slow down way too much and delay touchdown
      setStepHeight(-0.01);
      super.testShortSteps(testInfo);
   }

    @Test
    public void testToeOffWithDifferentStepLengths(TestInfo testInfo)
    {
        super.testToeOffWithDifferentStepLengths(testInfo);
    }

    @Test
    @Disabled
    public void testToeOffTakingShortStepDownCheckingAnkleLimits(TestInfo testInfo)
    {
        setStepHeight(-0.1);
        setCheckAnkleLimits(true);
        super.testToeOffTakingStep(testInfo);
    }

    @Test
    @Disabled
    public void testToeOffTakingShortStepDownCheckingAnkleLimitsWithExperimentalPhysicsEngine(TestInfo testInfo)
    {
        setUseExperimentalPhysicsEngine(true);
        setCheckAnkleLimits(true);
        setStepHeight(-0.1);
        super.testToeOffTakingStep(testInfo);
    }

    @Test
    @Disabled
    public void testToeOffTakingMediumStepDownCheckingAnkleLimits(TestInfo testInfo)
    {
        setStepHeight(-0.2);
        setCheckAnkleLimits(true);
        super.testToeOffTakingStep(testInfo);
    }

    @Test
    @Disabled
    public void testToeOffTakingHighStepDownCheckingAnkleLimits(TestInfo testInfo)
    {
        setStepHeight(-0.3);
        setCheckAnkleLimits(true);
        super.testToeOffTakingStep(testInfo);
    }

    @Override
    public double getStepLength()
    {
        return 0.3;
    }

    @Override
    public int getNumberOfSteps()
    {
        return 4;
    }

    @Override
    public double getMaxStepLength()
    {
        return 0.9;
    }

    @Override
    public DRCRobotModel getRobotModel()
    {
        AtlasRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false)
        {
            @Override
            public WalkingControllerParameters getWalkingControllerParameters()
            {
                return new AtlasWalkingControllerParameters(getTarget(), getJointMap(), getContactPointParameters())
                {
                    @Override
                    public ToeOffParameters getToeOffParameters()
                    {
                        return new AtlasToeOffParameters(getJointMap())
                        {
                            @Override
                            public boolean doToeOffIfPossibleInSingleSupport()
                            {
                                return true;
                            }
                        };
                    }
                };
            }
        };
        return robotModel;
    }

    @Override
    public String getSimpleRobotName()
    {
        return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.ATLAS);
    }
}
