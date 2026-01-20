package us.ihmc.zulu.roughTerrainWalking;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import us.ihmc.zulu.ZuluVersionInterface;
import us.ihmc.zulu.ZuluVersion;
import us.ihmc.zulu.ZuluRobotModel;
import us.ihmc.zulu.parameters.simulation.ZuluInitialSetup;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.initialSetup.HumanoidRobotInitialSetup;
import us.ihmc.avatar.roughTerrainWalking.AvatarAbsoluteStepTimingsTest;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.simulationConstructionSetTools.tools.CITools;

import java.io.InputStream;

import static org.junit.jupiter.api.Assertions.*;

@Tag("humanoid-rough-terrain")
public class ZuluAbsoluteStepTimingsTest extends AvatarAbsoluteStepTimingsTest
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
      ZuluVersionInterface robotVersion = ZuluVersion.V1_FULL_ROBOT;
      return new ZuluRobotModel(robotVersion, RobotTarget.SCS)
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
            ZuluInitialSetup initialSetup = new ZuluInitialSetup(robotVersion, getRobotDefinition(), getJointMap());
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
