package us.ihmc.atlas.pushRecovery;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.pushRecovery.HumanoidMomentumRecoveryTest;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

public class AtlasMomentumRecoveryTest extends HumanoidMomentumRecoveryTest
{
   private final DRCRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);

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

   @Override
   public void testPushDuringDoubleSupport() throws SimulationExceededMaximumTimeException
   {
      super.testPushDuringDoubleSupport();
   }
   
   @Override
   public void testPushDuringDoubleSupportExpectFall() throws SimulationExceededMaximumTimeException
   {
      super.testPushDuringDoubleSupportExpectFall();
   }
   
   @Override
   public void testPushDuringSwingExpectFall() throws SimulationExceededMaximumTimeException
   {
      super.testPushDuringSwingExpectFall();
   }
   
   @Override
   public void testRegularWalk() throws SimulationExceededMaximumTimeException
   {
      super.testRegularWalk();
   }
   
   @Override
   public void testPushDuringSwing() throws SimulationExceededMaximumTimeException
   {
      super.testPushDuringSwing();
   }
}
