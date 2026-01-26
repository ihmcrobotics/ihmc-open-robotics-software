package us.ihmc.zulu.controllerAPI;

import us.ihmc.zulu.ZuluVersion;
import us.ihmc.zulu.ZuluRobotModel;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.networkProcessor.kinematicsStreamingToolboxModule.KinematicsStreamingToolboxEndToEndTest;
import us.ihmc.simulationConstructionSetTools.tools.CITools;
import us.ihmc.simulationConstructionSetTools.tools.CITools.SimpleRobotNameKeys;

public class ZuluEndToEndKinematicsStreamingToolboxTest extends KinematicsStreamingToolboxEndToEndTest
{
   private final ZuluVersion robotVersion = ZuluVersion.V1_FULL_ROBOT;
   private final DRCRobotModel robotModel = new ZuluRobotModel(robotVersion, RobotTarget.SCS);

   @Override
   public DRCRobotModel newRobotModel()
   {
      return new ZuluRobotModel(robotVersion, RobotTarget.SCS);
   }

   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }

   public String getSimpleRobotName()
   {
      return CITools.getSimpleRobotNameFor(SimpleRobotNameKeys.ZULU);
   }
}
