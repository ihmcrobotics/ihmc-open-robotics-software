package us.ihmc.atlas.ObstacleCourseTests;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestInfo;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.atlas.parameters.AtlasToeOffParameters;
import us.ihmc.atlas.parameters.AtlasWalkingControllerParameters;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.obstacleCourseTests.AvatarToeOffTest;
import us.ihmc.commonWalkingControlModules.configurations.ToeOffParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;

public class AtlasCustomSteppingStonesTest extends AvatarToeOffTest
{
    @Test
    public void testToeOffWithDifferentStepLengths(TestInfo testInfo) throws BlockingSimulationRunner.SimulationExceededMaximumTimeException
    {
        super.testToeOffWithDifferentStepLengths(testInfo);
    }

    @Test
    @Disabled
    public void testToeOffTakingShortStepDownCheckingAnkleLimits(TestInfo testInfo) throws BlockingSimulationRunner.SimulationExceededMaximumTimeException
    {
        setCheckAnkleLimits(true);
        super.testToeOffTakingStep(testInfo, -0.1);
    }

    @Test
    @Disabled
    public void testToeOffTakingShortStepDownCheckingAnkleLimitsWithExperimentalPhysicsEngine(TestInfo testInfo) throws BlockingSimulationRunner.SimulationExceededMaximumTimeException
    {
        setUseExperimentalPhysicsEngine(true);
        setCheckAnkleLimits(true);
        super.testToeOffTakingStep(testInfo, -0.1);
    }

    @Test
    @Disabled
    public void testToeOffTakingMediumStepDownCheckingAnkleLimits(TestInfo testInfo) throws BlockingSimulationRunner.SimulationExceededMaximumTimeException
    {
        setCheckAnkleLimits(true);
        super.testToeOffTakingStep(testInfo, -0.2);
    }

    @Test
    @Disabled
    public void testToeOffTakingHighStepDownCheckingAnkleLimits(TestInfo testInfo) throws BlockingSimulationRunner.SimulationExceededMaximumTimeException
    {
        setCheckAnkleLimits(true);
        super.testToeOffTakingStep(testInfo, 0.3);
    }

    @Test
    public void testToeOffTakingBigStepsUp(TestInfo testInfo) throws BlockingSimulationRunner.SimulationExceededMaximumTimeException
    {
        //TODO remove as it replicates {@link us.ihmc.atlas.roughTerrainWalking.AtlasEndToEndStairsTest#testUpStairs(TestInfo)}
        setNumberOfSteps(4);
        super.testToeOffTakingStep(testInfo, 0.2);
    }

    @Test
    public void testToeOffTakingBigStepUpAndStopping(TestInfo testInfo) throws BlockingSimulationRunner.SimulationExceededMaximumTimeException
    {   // NOTE this passes but violates knee joint limits
        setTakeSquareUpStep(false);
        super.testToeOffTakingStep(testInfo, 0.4);
    }

    @Test
    public void testToeOffTakingBigStepUpAndSquaringUp(TestInfo testInfo) throws BlockingSimulationRunner.SimulationExceededMaximumTimeException
    {   // NOTE this passes but the shins collide with stair step
        super.testToeOffTakingStep(testInfo, 0.4);
    }

    @Test
    public void testToeOffTakingBigSideStepUp(TestInfo testInfo) throws BlockingSimulationRunner.SimulationExceededMaximumTimeException
    {
        super.testToeOffTakingStep(testInfo, 0.4, -0.2);
    }

    @Override
    public double getStepLength()
    {
        return 0.4;
    }

    @Override
    public int getNumberOfSteps()
    {
        return 4;
    }

    @Override
    public double getMaxStepLength()
    {
        return 1.1;
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

                            @Override
                            public double getECMPProximityForToeOff()
                            {
                                return 0.04;
                            }

                            @Override
                            public double getICPProximityForToeOff()
                            {
                                return 0.0;
                            }

                            @Override
                            public double getICPPercentOfStanceForSSToeOff()
                            {
                                return 0.25;
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
