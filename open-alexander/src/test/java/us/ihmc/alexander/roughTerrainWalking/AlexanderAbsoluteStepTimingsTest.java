package us.ihmc.alexander.roughTerrainWalking;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import us.ihmc.alexander.AlexanderVersion;
import us.ihmc.alexander.OpenAlexanderRobotModel;
import us.ihmc.alexander.parameters.simulation.AlexanderInitialSetup;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.initialSetup.HumanoidRobotInitialSetup;
import us.ihmc.avatar.roughTerrainWalking.AvatarAbsoluteStepTimingsTest;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.simulationConstructionSetTools.tools.CITools;

import java.io.InputStream;

import static org.junit.jupiter.api.Assertions.*;

@Tag("humanoid-rough-terrain")
public class AlexanderAbsoluteStepTimingsTest extends AvatarAbsoluteStepTimingsTest
{
   @Tag("humanoid-rough-terrain")
   @Override
   @Test
   public void testTakingStepsWithAbsoluteTimings()
   {
      super.testTakingStepsWithAbsoluteTimings();
   }

   @Tag("humanoid-rough-terrain")
   @Override
   @Test
   public void testMinimumTransferTimeIsRespected()
   {
      super.testMinimumTransferTimeIsRespected();
   }

   @Tag("humanoid-rough-terrain")
   @Override
   @Test
   public void testPausingWalkDuringLongTransfers()
   {
      super.testPausingWalkDuringLongTransfers();
   }

   @Override
   public DRCRobotModel getRobotModel()
   {
      return new OpenAlexanderRobotModel(AlexanderVersion.V0_FULL_ROBOT, RobotTarget.SCS)
      {
         @Override
         public InputStream getParameterOverwrites()
         {
            InputStream overwrites = AvatarAbsoluteStepTimingsTest.class.getResourceAsStream("/absolute_step_timing_test.xml");
            assertNotNull(overwrites);
            return overwrites;
         }

         @Override
         public HumanoidRobotInitialSetup getDefaultRobotInitialSetup()
         {
            AlexanderInitialSetup initialSetup = new AlexanderInitialSetup(getRobotDefinition(), getJointMap());
            initialSetup.setOffset(new Vector3D(-0.05, 0.0, 0.0));
            return initialSetup;
         }

      };
   }

   @Override
   public String getSimpleRobotName()
   {
      return CITools.getSimpleRobotNameFor(CITools.SimpleRobotNameKeys.ALEXANDER);
   }
}
