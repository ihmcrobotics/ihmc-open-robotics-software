package us.ihmc.atlas;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestInfo;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.obstacleCourseTests.AvatarToeOffTest;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;

public class AtlasToeOffTest extends AvatarToeOffTest
{
    @Test
    public void testToeOffTakingShortStepDownCheckingAnkleLimits(TestInfo testInfo) throws BlockingSimulationRunner.SimulationExceededMaximumTimeException
    {
        setStepHeight(-0.1);
        super.testToeOffTakingStepAndCheckingAnkleLimits(testInfo);
    }

    @Test
    public void testToeOffTakingMediumStepDownCheckingAnkleLimits(TestInfo testInfo) throws BlockingSimulationRunner.SimulationExceededMaximumTimeException
    {
        setStepHeight(-0.2);
        super.testToeOffTakingStepAndCheckingAnkleLimits(testInfo);
    }

    @Test
    public void testToeOffTakingHighStepDownCheckingAnkleLimits(TestInfo testInfo) throws BlockingSimulationRunner.SimulationExceededMaximumTimeException
    {
        setStepHeight(-0.3);
        super.testToeOffTakingStepAndCheckingAnkleLimits(testInfo);
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
        return 1.1;
    }

    @Override
    public DRCRobotModel getRobotModel()
    {
        AtlasRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);
        return robotModel;
    }

    @Override
    public String getSimpleRobotName()
    {
        return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.ATLAS);
    }
}
