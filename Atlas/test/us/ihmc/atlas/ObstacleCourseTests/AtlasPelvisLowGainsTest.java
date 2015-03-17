package us.ihmc.atlas.ObstacleCourseTests;

import static org.junit.Assert.assertTrue;
import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.atlas.AtlasInverseDynamicsCalculatorTestHelper;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotModel.AtlasTarget;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.obstacleCourseTests.DRCPelvisLowGainsTest;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.UnreasonableAccelerationException;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.utilities.code.agileTesting.BambooAnnotations.BambooPlan;
import us.ihmc.utilities.code.agileTesting.BambooPlanType;
import us.ihmc.utilities.screwTheory.InverseDynamicsCalculator;
import us.ihmc.utilities.screwTheory.InverseDynamicsCalculatorListener;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

@BambooPlan(planType = {BambooPlanType.InDevelopment, BambooPlanType.VideoA})
public class AtlasPelvisLowGainsTest extends DRCPelvisLowGainsTest
{
   private final DRCRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, AtlasRobotModel.AtlasTarget.SIM, false);

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
   protected DoubleYoVariable getPelvisOrientationErrorVariableName(SimulationConstructionSet scs)
   {
      return (DoubleYoVariable) scs.getVariable(
          "MomentumBasedControllerFactory.PelvisOrientationManager.RootJointAngularAccelerationControlModule.pelvisAxisAngleOrientationController",
          "pelvisOrientationErrorMagnitude");
   }

   @Override
   public InverseDynamicsCalculatorListener getInverseDynamicsCalculatorListener(SDFFullRobotModel controllersFullRobotModel)
   {
      return null;
//      InverseDynamicsCalculatorListener inverseDynamicsCalculatorListener = new FancyInverseDynamicsCalculatorListener(controllersFullRobotModel);
//      
//      return inverseDynamicsCalculatorListener;
   }
   
//   private class FancyInverseDynamicsCalculatorListener implements InverseDynamicsCalculatorListener
//   {
////      private final Random random = new Random(1099L);
//      private final boolean visualize = true;
//      private final double gravityZ = 9.81;
//      private final AtlasInverseDynamicsCalculatorTestHelper atlasInverseDynamicsCalculatorTestHelper;
////      private final FullRobotModel controllersFullRobotModel;
//      private final SimulationConstructionSet scs;
//      private final SDFRobot robot;
//      
//      public FancyInverseDynamicsCalculatorListener(SDFFullRobotModel controllersFullRobotModel)
//      {
////         this.controllersFullRobotModel = controllersFullRobotModel;
//         
//         boolean headless = false;
//         AtlasRobotModel atlasRobotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, AtlasTarget.SIM, headless);
////         SDFFullRobotModel fullRobotModel = atlasRobotModel.createFullRobotModel();
//         
//         boolean createCollisionMeshes = false;
//         atlasRobotModel.setEnableJointDamping(false);
//         robot = atlasRobotModel.createSdfRobot(createCollisionMeshes);
//         robot.setGravity(gravityZ);
//         
//         atlasInverseDynamicsCalculatorTestHelper = new AtlasInverseDynamicsCalculatorTestHelper(controllersFullRobotModel, robot, visualize, gravityZ);
//         
////         atlasInverseDynamicsCalculatorTestHelper =  AtlasInverseDynamicsCalculatorTestHelper.createAtlasInverseDynamicsCalculatorTestHelperUsingAtlasUnplugged(visualize, gravityZ);
//         scs = atlasInverseDynamicsCalculatorTestHelper.getSimulationConstructionSet();
//         
//         atlasInverseDynamicsCalculatorTestHelper.startSimulationOnAThread();
//      }
//      
//      @Override
//      public void inverseDynamicsCalculatorIsDone(InverseDynamicsCalculator inverseDynamicsCalculator)
//      {
////         atlasInverseDynamicsCalculatorTestHelper.setFullRobotModelToMatch(controllersFullRobotModel);
//         
//         
//
////         atlasInverseDynamicsCalculatorTestHelper.setFullRobotModelStateRandomly(random, maxJointVelocity, maxRootJointLinearAndAngularVelocity);
////         atlasInverseDynamicsCalculatorTestHelper.setFullRobotModelExternalForcesRandomly(random, maxFeetExternalForce, maxFeetExternalTorque);
////         atlasInverseDynamicsCalculatorTestHelper.setFullRobotModelAccelerationRandomly(random, maxPelvisLinearAcceleration, maxPelvisAngularAcceleration, maxJointAcceleration);
//
//         SDFFullRobotModel fullRobotModelInAtlasInverseDynamicsCalculatorTestHelper = atlasInverseDynamicsCalculatorTestHelper.getFullRobotModel();
//         fullRobotModelInAtlasInverseDynamicsCalculatorTestHelper.updateFrames();
//
////         atlasInverseDynamicsCalculatorTestHelper.computeTwistCalculatorAndInverseDynamicsCalculator();
//
//         atlasInverseDynamicsCalculatorTestHelper.setRobotStateToMatchFullRobotModel();
//         atlasInverseDynamicsCalculatorTestHelper.setRobotsExternalForcesToMatchFullRobotModel();
//         atlasInverseDynamicsCalculatorTestHelper.setRobotTorquesToMatchFullRobotModel();
//
//         Robot robot = atlasInverseDynamicsCalculatorTestHelper.getRobot();
//         try
//         {
//            robot.doDynamicsButDoNotIntegrate();
//         }
//         catch (UnreasonableAccelerationException e)
//         {
//            throw new RuntimeException();
//         }
//
//         double epsilon = 1e-7;
//         boolean makeAssertions = false;
//         
//         boolean accelerationsMatch = atlasInverseDynamicsCalculatorTestHelper.checkAccelerationsMatchBetweenFullRobotModelAndSimulatedRobot(epsilon);
//         if (makeAssertions)
//            assertTrue(accelerationsMatch);
//
//         if (scs != null)
//            scs.tickAndUpdate();
//      }
//      
//   }
}
