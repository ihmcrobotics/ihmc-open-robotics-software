package us.ihmc.zulu;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import us.ihmc.avatar.DRCBumpyAndShallowRampsWalkingTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.simulationConstructionSetTools.tools.CITools;
import us.ihmc.simulationconstructionset.util.ControllerFailureException;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

@Tag("humanoid-obstacle")
public class ZuluBumpyAndShallowRampsWalkingTest extends DRCBumpyAndShallowRampsWalkingTest
{
   @Override
   public DRCRobotModel getRobotModel()
   {
      return new ZuluRobotModel(ZuluVersion.V1_FULL_ROBOT, RobotTarget.SCS);
   }

   @Override
   public String getSimpleRobotName()
   {
      return CITools.getSimpleRobotNameFor(CITools.SimpleRobotNameKeys.ALEXANDER);
   }

   // This test is disabled because the height map environmetn doesn't work in SCS2
   @Tag("humanoid-obstacle")
   @Disabled
   @Test
   @Override
   public void testDRCBumpyGroundWalking() throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      super.testDRCBumpyGroundWalking();
   }
   
   // This has never worked. Would be nice if we can get it to work.")
   @Tag("humanoid-obstacle")
   @Disabled
   @Test
   @Override
   public void testDRCOverRandomBlocks() throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      super.testDRCOverRandomBlocks();
   }

   @Tag("humanoid-obstacle")
   @Test
   @Override
   public void testDRCOverShallowRamp() throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      super.testDRCOverShallowRamp();
   }
}
