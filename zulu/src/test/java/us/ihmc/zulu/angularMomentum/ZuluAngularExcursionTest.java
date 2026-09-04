package us.ihmc.zulu.angularMomentum;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import us.ihmc.zulu.ZuluVersion;
import us.ihmc.zulu.ZuluRobotModel;
import us.ihmc.avatar.angularMomentumTest.AvatarAngularExcursionTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

@Tag("humanoid-flat-ground-slow-2")
public class ZuluAngularExcursionTest extends AvatarAngularExcursionTest
{
   private final DRCRobotModel robotModel = new ZuluRobotModel(ZuluVersion.V1_FULL_ROBOT, RobotTarget.SCS);

   @Override
   protected double getStepLength()
   {
      return 0.4;
   }

   @Override
   protected double getStepWidth()
   {
      return 0.25;
   }

   @Override
   protected double getMoveInPlaceYawTolerance()
   {
      return 1.5e-2;
   }

   @Override
   protected double getMoveInPlacePitchTolerance()
   {
      return 1.0e-2;
   }

   @Override
   protected double getForwardWalkPitchTolerance()
   {
      return 1.2e-1;
   }

   @Override
   protected double getWalkInASquareYawTolerance()
   {
      return 3.5e-1;
   }

   @Override
   @Test
   public void testWalkInASquare() throws SimulationExceededMaximumTimeException
   {
      super.testWalkInASquare();
   }


   @Override
   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }

   @Override
   public String getSimpleRobotName()
   {
      return "zulu";
   }
}
