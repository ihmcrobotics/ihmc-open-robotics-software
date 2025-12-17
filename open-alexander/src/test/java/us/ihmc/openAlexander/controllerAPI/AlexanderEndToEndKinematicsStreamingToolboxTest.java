package us.ihmc.openAlexander.controllerAPI;

import us.ihmc.openAlexander.ZuluVersion;
import us.ihmc.openAlexander.OpenAlexanderRobotModel;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.networkProcessor.kinematicsStreamingToolboxModule.KinematicsStreamingToolboxEndToEndTest;
import us.ihmc.simulationConstructionSetTools.tools.CITools;
import us.ihmc.simulationConstructionSetTools.tools.CITools.SimpleRobotNameKeys;

public class AlexanderEndToEndKinematicsStreamingToolboxTest extends KinematicsStreamingToolboxEndToEndTest
{
   private final ZuluVersion robotVersion = ZuluVersion.V1_FULL_ROBOT;
   private final DRCRobotModel robotModel = new OpenAlexanderRobotModel(robotVersion, RobotTarget.SCS);

   @Override
   public DRCRobotModel newRobotModel()
   {
      return new OpenAlexanderRobotModel(robotVersion, RobotTarget.SCS);
   }

   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }

   public String getSimpleRobotName()
   {
      return CITools.getSimpleRobotNameFor(SimpleRobotNameKeys.ALEXANDER);
   }
}
