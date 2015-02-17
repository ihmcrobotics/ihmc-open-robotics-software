package us.ihmc.atlas.behaviorTests;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.atlas.parameters.AtlasContactPointParameters;
import us.ihmc.darpaRoboticsChallenge.behaviorTests.DRCHandLoadBearingBehaviorTest;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;

public class AtlasHandLoadBearingBehaviorTest extends DRCHandLoadBearingBehaviorTest
{
   private final AtlasRobotModel robotModel;
   
   public AtlasHandLoadBearingBehaviorTest()
   {
      robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_INVISIBLE_CONTACTABLE_PLANE_HANDS, AtlasRobotModel.AtlasTarget.SIM, false);
      AtlasContactPointParameters contactPointParameters = robotModel.getContactPointParameters();
      contactPointParameters.createHandKnobContactPoints();
   }
   
   
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

}
