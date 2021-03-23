package us.ihmc.atlas;

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
