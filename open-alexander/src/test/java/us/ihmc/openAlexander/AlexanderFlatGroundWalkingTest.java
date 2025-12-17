package us.ihmc.openAlexander;

import org.junit.jupiter.api.Assumptions;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import org.opentest4j.TestAbortedException;
import us.ihmc.avatar.DRCFlatGroundWalkingTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.simulationConstructionSetTools.tools.CITools;

// This test is slow but very important, let's keep it in the FAST build please. (Sylvain)
public class AlexanderFlatGroundWalkingTest extends DRCFlatGroundWalkingTest
{
   private DRCRobotModel robotModel;
   private boolean doPelvisWarmup;

   @Override
   public boolean doPelvisWarmup()
   {
      return doPelvisWarmup;
   }
   
   public void setDoPelvisWarmup(boolean doPelvisWarmup)
   {
      this.doPelvisWarmup = doPelvisWarmup;
   }

   @Tag("fast")
   @Override
   @Test
   public void testFlatGroundWalking()
   {
      robotModel = new OpenAlexanderRobotModel(ZuluVersion.V1_FULL_ROBOT, RobotTarget.SCS);
      setDoPelvisWarmup(true);
      super.testFlatGroundWalking();
   }

   @Disabled
   @Override
   @Test
   public void testFlatGroundWalkingBullet()
   {
      robotModel = new OpenAlexanderRobotModel(ZuluVersion.V1_FULL_ROBOT, RobotTarget.SCS);
      setDoPelvisWarmup(false);
      super.testFlatGroundWalkingBullet();
   }

   @Tag("fast")
   @Override
   @Test
   public void testReset()
   {
      robotModel = new OpenAlexanderRobotModel(ZuluVersion.V1_FULL_ROBOT, RobotTarget.SCS);
      super.testReset();
   }


   @Test
   @Disabled // Not working because of multithreading. Should be switched over to use the DRCSimulationTestHelper.
   public void testFlatGroundWalkingRunsSameWayTwice()
   {
      try
      {
         Assumptions.assumeTrue(CITools.isNightlyBuild());
         CITools.reportTestStartedMessage(getSimulationTestingParameters().getShowWindows());

         robotModel = new OpenAlexanderRobotModel(ZuluVersion.V1_FULL_ROBOT, RobotTarget.SCS);

         setupAndTestFlatGroundSimulationTrackTwice(robotModel);
      }
      catch (TestAbortedException e)
      {
         System.out.println("Not Nightly Build, skipping AlexanderlatGroundWalkingTest.testFlatGroundWalkingRunsSameWayTwice");
      }
   }

   @Override
   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }

   @Override
   public String getSimpleRobotName()
   {
      return CITools.getSimpleRobotNameFor(CITools.SimpleRobotNameKeys.ALEXANDER);
   }
}
