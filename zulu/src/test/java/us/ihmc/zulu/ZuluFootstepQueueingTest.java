package us.ihmc.zulu;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import us.ihmc.avatar.AvatarFootstepQueueingTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

@Tag("humanoid-flat-ground-slow-2")
public class ZuluFootstepQueueingTest extends AvatarFootstepQueueingTest
{
   private final ZuluVersion version = ZuluVersion.V1_FULL_ROBOT;
   private final RobotTarget target = RobotTarget.SCS;
   private final ZuluRobotModel robotModel = new ZuluRobotModel(version, target);

   private static final double stepWidth = 0.25;
   private static final double stepLength = 0.5;

   @Override
   @Test
   public void testTwoPlans() throws SimulationExceededMaximumTimeException
   {
      super.testTwoPlans();
   }

   @Override
   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }

   @Override
   public String getSimpleRobotName()
   {
      return robotModel.getSimpleRobotName();
   }


   @Override
   public double getStepWidth()
   {
      return stepWidth;
   }

   @Override
   public double getStepLength()
   {
      return stepLength;
   }
}
