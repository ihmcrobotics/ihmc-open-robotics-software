package us.ihmc.valkyrie.roughTerrainWalking;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.factory.RobotDefinitionTools;
import us.ihmc.avatar.roughTerrainWalking.EndToEndCinderBlockFieldTest;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.configuration.ValkyrieRobotVersion;

public class ValkyrieEndToEndCinderBlockFieldTest extends EndToEndCinderBlockFieldTest
{
   private boolean useVal2Scale = false;
   private boolean removeAnkleJointLimits = false;

   @Override
   public DRCRobotModel getRobotModel()
   {
      ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS, ValkyrieRobotVersion.FINGERLESS);
      if (useVal2Scale)
         robotModel.setVal2Scale();

      if (removeAnkleJointLimits)
      {
         robotModel.setRobotDefinitionMutator(robotModel.getRobotDefinitionMutator()
                                                        .andThen(RobotDefinitionTools.jointLimitMutator("Ankle", -Math.PI, Math.PI)));
      }

      return robotModel;
   }

   @BeforeEach
   public void initializeTest()
   {
      useVal2Scale = false;
      removeAnkleJointLimits = false;
   }

   @Override
   public double getPelvisOffsetHeight()
   {
      return -0.05;
   }

   @Override
   public double getStepHeightOffset()
   {
      return 0.0;
   }

   @Override
   public String getSimpleRobotName()
   {
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.VALKYRIE);
   }

   @Override
   @Tag("humanoid-rough-terrain")
   @Test
   public void testWalkingOverCinderBlockField() throws Exception
   {
      super.testWalkingOverCinderBlockField();
   }

   @Disabled
   @Override
   @Tag("humanoid-rough-terrain-slow")
   @Test
   public void testSteppingStonesA() throws Exception
   {
      super.testSteppingStonesA();
   }

   @Disabled
   @Override
   @Tag("humanoid-rough-terrain-slow")
   @Test
   public void testSteppingStonesB() throws Exception
   {
      removeAnkleJointLimits = true;
      super.testSteppingStonesB();
   }

   @Disabled
   @Tag("humanoid-rough-terrain-slow")
   @Test
   public void testSlantedCinderBlockFieldA() throws Exception
   {
      removeAnkleJointLimits = true;
      super.testSlantedCinderBlockField(false);
   }

   @Disabled
   @Tag("humanoid-rough-terrain-slow")
   @Test
   public void testSlantedCinderBlockFieldB() throws Exception
   {
      removeAnkleJointLimits = true;
      super.testSlantedCinderBlockField(true);
   }

   @Tag("humanoid-rough-terrain")
   @Test
   public void testWalkingOverCinderBlockFieldVal2Scale() throws Exception
   {
      useVal2Scale = true;
      removeAnkleJointLimits = true; // TODO Need to improve swing to better avoid toe-stub.
      super.testWalkingOverCinderBlockField();
   }

   @Tag("humanoid-rough-terrain-slow")
   @Test
   public void testSteppingStonesAVal2Scale() throws Exception
   {
      useVal2Scale = true;
      super.testSteppingStonesA();
   }

   @Tag("humanoid-rough-terrain-slow")
   @Test
   public void testSteppingStonesBVal2Scale() throws Exception
   {
      useVal2Scale = true;
      removeAnkleJointLimits = true;
      super.testSteppingStonesB();
   }

   @Tag("humanoid-rough-terrain-slow")
   @Test
   public void testSlantedCinderBlockFieldAVal2Scale() throws Exception
   {
      useVal2Scale = true;
      removeAnkleJointLimits = true;
      super.testSlantedCinderBlockField(false);
   }

   @Tag("humanoid-rough-terrain-slow")
   @Test
   public void testSlantedCinderBlockFieldBVal2Scale() throws Exception
   {
      useVal2Scale = true;
      removeAnkleJointLimits = true;
      super.testSlantedCinderBlockField(true);
   }
}
