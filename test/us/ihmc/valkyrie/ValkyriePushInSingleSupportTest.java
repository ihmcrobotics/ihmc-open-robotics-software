package us.ihmc.valkyrie;

import us.ihmc.bambooTools.BambooTools;
import us.ihmc.darpaRoboticsChallenge.DRCPushRecoveryTest;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;

public class ValkyriePushInSingleSupportTest extends DRCPushRecoveryTest
{

	   @Override
	   public DRCRobotModel getRobotModel()
	   {
	      return new ValkyrieRobotModel(false,false);
	   }

	   @Override
	   public String getSimpleRobotName()
	   {
	      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.VALKYRIE);
	   }
}