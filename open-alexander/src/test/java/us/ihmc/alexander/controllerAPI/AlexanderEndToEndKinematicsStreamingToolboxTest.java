package us.ihmc.alexander.controllerAPI;

import us.ihmc.alexander.OpenAlexanderVersion;
import us.ihmc.alexander.OpenAlexanderRobotModel;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.networkProcessor.kinematicsStreamingToolboxModule.KinematicsStreamingToolboxEndToEndTest;
import us.ihmc.simulationConstructionSetTools.tools.CITools;
import us.ihmc.simulationConstructionSetTools.tools.CITools.SimpleRobotNameKeys;

public class AlexanderEndToEndKinematicsStreamingToolboxTest extends KinematicsStreamingToolboxEndToEndTest
{
   private final OpenAlexanderVersion robotVersion = OpenAlexanderVersion.V0_FULL_ROBOT;
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
