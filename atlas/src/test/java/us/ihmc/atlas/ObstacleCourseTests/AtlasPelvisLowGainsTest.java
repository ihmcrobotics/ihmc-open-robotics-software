package us.ihmc.atlas.ObstacleCourseTests;

import static org.junit.Assert.assertTrue;

import org.junit.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.obstacleCourseTests.DRCInverseDynamicsCalculatorTestHelper;
import us.ihmc.avatar.obstacleCourseTests.DRCPelvisLowGainsTest;
import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerToolbox;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.screwTheory.InverseDynamicsCalculator;
import us.ihmc.robotics.screwTheory.InverseDynamicsCalculatorListener;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.UnreasonableAccelerationException;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.yoVariables.variable.YoDouble;

@ContinuousIntegrationPlan(categories = {IntegrationCategory.FAST, IntegrationCategory.VIDEO})
public class AtlasPelvisLowGainsTest extends DRCPelvisLowGainsTest
{
   private final DRCRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false)
   {
      // Disable joint damping to make sure that damping isn't causing the problem.
      private static final boolean enableJointDamping = false;

      @Override
      public HumanoidFloatingRootJointRobot createHumanoidFloatingRootJointRobot(boolean createCollisionMeshes)
      {
         return createHumanoidFloatingRootJointRobot(createCollisionMeshes, enableJointDamping);
      }
   };

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
   @ContinuousIntegrationTest(estimatedDuration = 38.0)
   @Test(timeout = 160000)
   public void testStandingWithLowPelvisOrientationGains() throws SimulationExceededMaximumTimeException
   {
      super.testStandingWithLowPelvisOrientationGains();
   }

   @Override
   protected YoDouble getPelvisOrientationErrorVariableName(SimulationConstructionSet scs)
   {

      return (YoDouble) scs.getVariable(FeedbackControllerToolbox.class.getSimpleName(), "pelvisErrorRotationVectorZ");
   }

   @Override
   public InverseDynamicsCalculatorListener getInverseDynamicsCalculatorListener(FullRobotModel controllersFullRobotModel, FloatingRootJointRobot robot)
   {
      return null;
//      InverseDynamicsCalculatorListener inverseDynamicsCalculatorListener = new FancyInverseDynamicsCalculatorListener(controllersFullRobotModel, robot);
//      return inverseDynamicsCalculatorListener;
   }

   private class FancyInverseDynamicsCalculatorListener implements InverseDynamicsCalculatorListener
   {
      private final boolean visualize = true;
      private final double gravityZ = 9.81;
      private final DRCInverseDynamicsCalculatorTestHelper atlasInverseDynamicsCalculatorTestHelper;
      private final FullRobotModel controllersFullRobotModel;
      private final SimulationConstructionSet scs;
      private final HumanoidFloatingRootJointRobot robot, simulatedRobot;
      private boolean firstTick = true;

      public FancyInverseDynamicsCalculatorListener(FullHumanoidRobotModel controllersFullRobotModel, HumanoidFloatingRootJointRobot simulatedRobot)
      {
         this.controllersFullRobotModel = controllersFullRobotModel;
         this.simulatedRobot = simulatedRobot;

         boolean headless = false;
         AtlasRobotModel atlasRobotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, headless);
//         FullRobotModel fullRobotModel = atlasRobotModel.createFullRobotModel();

         boolean createCollisionMeshes = false;
         robot = atlasRobotModel.createHumanoidFloatingRootJointRobot(createCollisionMeshes, false);
         robot.setGravity(gravityZ);

         atlasInverseDynamicsCalculatorTestHelper = new DRCInverseDynamicsCalculatorTestHelper(controllersFullRobotModel, robot, visualize, gravityZ);

//         atlasInverseDynamicsCalculatorTestHelper =  AtlasInverseDynamicsCalculatorTestHelper.createAtlasInverseDynamicsCalculatorTestHelperUsingAtlasUnplugged(visualize, gravityZ);
         scs = atlasInverseDynamicsCalculatorTestHelper.getSimulationConstructionSet();

         atlasInverseDynamicsCalculatorTestHelper.startSimulationOnAThread();
      }

      @Override
      public void inverseDynamicsCalculatorIsDone(InverseDynamicsCalculator inverseDynamicsCalculator)
      {
         FullRobotModel fullRobotModelInAtlasInverseDynamicsCalculatorTestHelper = atlasInverseDynamicsCalculatorTestHelper.getFullRobotModel();
         fullRobotModelInAtlasInverseDynamicsCalculatorTestHelper.updateFrames();

         // Either of these two seem to work. Matching full robot model gives near exact results. Matching robot state gives really close results.
//         atlasInverseDynamicsCalculatorTestHelper.setRobotStateToMatchOtherRobot(simulatedRobot);
         atlasInverseDynamicsCalculatorTestHelper.setRobotStateToMatchFullRobotModel();

         atlasInverseDynamicsCalculatorTestHelper.setRobotsExternalForcesToMatchOtherRobot(simulatedRobot);
//         atlasInverseDynamicsCalculatorTestHelper.setRobotsExternalForcesToMatchFullRobotModel(inverseDynamicsCalculator);

         atlasInverseDynamicsCalculatorTestHelper.setRobotTorquesToMatchFullRobotModel();
//       atlasInverseDynamicsCalculatorTestHelper.setRobotTorquesToMatchOtherRobot(simulatedRobot);

         double epsilon = 1e-7;

//       atlasInverseDynamicsCalculatorTestHelper.setRobotTorquesToMatchFullRobotModelButCheckAgainstOtherRobot(simulatedRobot, 0.1);

         boolean makeAssertions = false;


//         if (!firstTick)
//         {
//            //         atlasInverseDynamicsCalculatorTestHelper.setRobotTorquesToMatchOtherRobot(simulatedRobot);
//            boolean torquesMatch = atlasInverseDynamicsCalculatorTestHelper.checkTorquesMatchBetweenFullRobotModelAndSimulatedRobot(epsilon);
//            //         boolean torquesMatch = atlasInverseDynamicsCalculatorTestHelper.compareTorquesBetweenOtherRobotAndFullRobotModel(simulatedRobot, epsilon);
//            if (makeAssertions)
//               assertTrue(torquesMatch);
//         }

         try
         {
            robot.doDynamicsButDoNotIntegrate();
         }
         catch (UnreasonableAccelerationException e)
         {
            throw new RuntimeException();
         }

         if (!firstTick)
         {
            boolean accelerationsMatch = atlasInverseDynamicsCalculatorTestHelper.checkAccelerationsMatchBetweenFullRobotModelAndSimulatedRobot(epsilon);
            if (makeAssertions)
               assertTrue(accelerationsMatch);
         }

         if (scs != null)
         {
            scs.setTime(scs.getTime() + 0.006);
            scs.tickAndUpdate();
         }

         firstTick = false;
      }

   }
}
