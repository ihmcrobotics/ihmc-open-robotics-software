package us.ihmc.atlas.ObstacleCourseTests;

import static org.junit.Assert.assertTrue;
import us.ihmc.SdfLoader.SDFBaseRobot;
import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotModel.AtlasTarget;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.obstacleCourseTests.DRCInverseDynamicsCalculatorTestHelper;
import us.ihmc.darpaRoboticsChallenge.obstacleCourseTests.DRCPelvisLowGainsTest;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.UnreasonableAccelerationException;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.tools.agileTesting.BambooAnnotations.BambooPlan;
import us.ihmc.tools.agileTesting.BambooPlanType;
import us.ihmc.robotics.screwTheory.InverseDynamicsCalculator;
import us.ihmc.robotics.screwTheory.InverseDynamicsCalculatorListener;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

@BambooPlan(planType = {BambooPlanType.Slow, BambooPlanType.VideoA})
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
   public String getKpPelvisOrientationName()
   {
      return "kpPelvisOrientation";
   }

   @Override
   public String getZetaPelvisOrientationName()
   {
      return "zetaPelvisOrientation";
   }


   @Override
   public InverseDynamicsCalculatorListener getInverseDynamicsCalculatorListener(SDFFullRobotModel controllersFullRobotModel, SDFBaseRobot robot)
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
      private final SDFFullRobotModel controllersFullRobotModel;
      private final SimulationConstructionSet scs;
      private final SDFRobot robot, simulatedRobot;
      private boolean firstTick = true;

      public FancyInverseDynamicsCalculatorListener(SDFFullRobotModel controllersFullRobotModel, SDFRobot simulatedRobot)
      {
         this.controllersFullRobotModel = controllersFullRobotModel;
         this.simulatedRobot = simulatedRobot;
         
         boolean headless = false;
         AtlasRobotModel atlasRobotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, AtlasTarget.SIM, headless);
//         SDFFullRobotModel fullRobotModel = atlasRobotModel.createFullRobotModel();
         
         boolean createCollisionMeshes = false;
         atlasRobotModel.setEnableJointDamping(false);
         robot = atlasRobotModel.createSdfRobot(createCollisionMeshes);
         robot.setGravity(gravityZ);
         
         atlasInverseDynamicsCalculatorTestHelper = new DRCInverseDynamicsCalculatorTestHelper(controllersFullRobotModel, robot, visualize, gravityZ);
         
//         atlasInverseDynamicsCalculatorTestHelper =  AtlasInverseDynamicsCalculatorTestHelper.createAtlasInverseDynamicsCalculatorTestHelperUsingAtlasUnplugged(visualize, gravityZ);
         scs = atlasInverseDynamicsCalculatorTestHelper.getSimulationConstructionSet();
         
         atlasInverseDynamicsCalculatorTestHelper.startSimulationOnAThread();
      }
      
      @Override
      public void inverseDynamicsCalculatorIsDone(InverseDynamicsCalculator inverseDynamicsCalculator)
      {
         SDFFullRobotModel fullRobotModelInAtlasInverseDynamicsCalculatorTestHelper = atlasInverseDynamicsCalculatorTestHelper.getFullRobotModel();
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
