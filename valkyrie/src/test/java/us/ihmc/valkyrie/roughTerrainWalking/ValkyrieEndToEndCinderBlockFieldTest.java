package us.ihmc.valkyrie.roughTerrainWalking;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.roughTerrainWalking.EndToEndCinderBlockFieldTest;
import us.ihmc.modelFileLoaders.SdfLoader.GeneralizedSDFRobotModel;
import us.ihmc.modelFileLoaders.SdfLoader.SDFDescriptionMutator;
import us.ihmc.modelFileLoaders.SdfLoader.SDFDescriptionMutatorList;
import us.ihmc.modelFileLoaders.SdfLoader.SDFJointHolder;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.configuration.ValkyrieRobotVersion;

public class ValkyrieEndToEndCinderBlockFieldTest extends EndToEndCinderBlockFieldTest
{
   private boolean removeAnkleJointLimits = false;

   @Override
   public DRCRobotModel getRobotModel()
   {
      ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS, ValkyrieRobotVersion.FINGERLESS);

      if (removeAnkleJointLimits)
      {
         robotModel.setSDFDescriptionMutator(new SDFDescriptionMutatorList(robotModel.getSDFDescriptionMutator(), new SDFDescriptionMutator()
         {
            @Override
            public void mutateJointForModel(GeneralizedSDFRobotModel model, SDFJointHolder jointHolder)
            {
               if (jointHolder.getName().contains("Ankle"))
               {
                  jointHolder.setLimits(-Math.PI, Math.PI);
               }
            }
         }));
      }

      return robotModel;
   }

   @BeforeEach
   public void initializeTest()
   {
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
   @Test
   public void testWalkingOverCinderBlockField() throws Exception
   {
      super.testWalkingOverCinderBlockField();
   }

   @Override
   @Tag("humanoid-rough-terrain-slow")
   @Test
   public void testSteppingStonesA() throws Exception
   {
      super.testSteppingStonesA();
   }

   @Override
   @Tag("humanoid-rough-terrain-slow")
   @Test
   public void testSteppingStonesB() throws Exception
   {
      removeAnkleJointLimits = true;
      super.testSteppingStonesB();
   }

   @Tag("humanoid-rough-terrain-slow")
   @Test
   public void testSlantedCinderBlockLeveledFieldA() throws Exception
   {
      removeAnkleJointLimits = true;
      super.testSlantedCinderBlockLeveledField(false);
   }

   @Tag("humanoid-rough-terrain-slow")
   @Test
   public void testSlantedCinderBlockLeveledFieldB() throws Exception
   {
      removeAnkleJointLimits = true;
      super.testSlantedCinderBlockLeveledField(true);
   }
}
