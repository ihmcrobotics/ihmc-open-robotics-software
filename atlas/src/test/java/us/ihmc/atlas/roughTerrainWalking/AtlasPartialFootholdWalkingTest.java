package us.ihmc.atlas.roughTerrainWalking;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.roughTerrainWalking.HumanoidPartialFootholdWalkingTest;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.wholeBodyController.AdditionalSimulationContactPoints;
import us.ihmc.wholeBodyController.FootContactPoints;
import us.ihmc.yoVariables.variable.YoVariable;

@Tag("humanoid-rough-terrain-slow")
public class AtlasPartialFootholdWalkingTest extends HumanoidPartialFootholdWalkingTest
{

   @Override
   public DRCRobotModel getRobotModel()
   {
      FootContactPoints<RobotSide> simulationContactPoints = new AdditionalSimulationContactPoints<>(RobotSide.values, 10, 5, true, false);
      AtlasRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false, simulationContactPoints)
      {
         @Override
         public double getSimulateDT()
         {
            return 0.00025;
         }
      };

      return robotModel;
   }

   @Override
   public String getSimpleRobotName()
   {
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.ATLAS);
   }

   @Test
   public void testSteppingOntoWithInsideOfFoot() throws SimulationExceededMaximumTimeException
   {
      setUseExperimentalPhysicsEngine(false);
      super.testSteppingOntoWithInsideOfFoot();
   }

   @Test
   public void testSteppingOntoWithInsideOfFootWithExperimentalPhysicsEngine() throws SimulationExceededMaximumTimeException
   {
      setUseExperimentalPhysicsEngine(true);
      super.testSteppingOntoWithInsideOfFoot();
   }

   @Test
   public void testSteppingOntoWithOutsideOfFoot() throws SimulationExceededMaximumTimeException
   {
      setUseExperimentalPhysicsEngine(false);
      super.testSteppingOntoWithOutsideOfFoot();
   }

   @Test
   public void testSteppingOntoWithOutsideOfFootWithExperimentalPhysicsEngine() throws SimulationExceededMaximumTimeException
   {
      setUseExperimentalPhysicsEngine(true);
      super.testSteppingOntoWithOutsideOfFoot();
   }

   @Test
   public void testSteppingOntoBlock() throws SimulationExceededMaximumTimeException
   {
      setUseExperimentalPhysicsEngine(false);
      super.testSteppingOntoBlock();
   }

   @Test
   public void testSteppingOntoBlockWithExperimentalPhysicsEngine() throws SimulationExceededMaximumTimeException
   {
      setUseExperimentalPhysicsEngine(true);
      super.testSteppingOntoBlock();
   }

   @Test
   public void testSteppingOntoNarrowBlock() throws  SimulationExceededMaximumTimeException
   {
      setUseExperimentalPhysicsEngine(false);
      super.testSteppingOntoNarrowBlock();
   }

   @Test
   public void testSteppingOntoNarrowBlockWithExperimentalPhysicsEngine() throws  SimulationExceededMaximumTimeException
   {
      setUseExperimentalPhysicsEngine(true);
      super.testSteppingOntoNarrowBlock();
   }


   @Test
   public void testWalkingOverBlock() throws SimulationExceededMaximumTimeException
   {
      setUseExperimentalPhysicsEngine(false);
      super.testWalkingOverBlock();
   }

   @Test
   public void testWalkingOverBlockWithExperimentalPhysicsEngine() throws SimulationExceededMaximumTimeException
   {
      setUseExperimentalPhysicsEngine(true);
      super.testWalkingOverBlock();
   }

}
