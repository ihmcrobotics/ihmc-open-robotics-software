package us.ihmc.avatar.obstacleCourseTests;

import static org.junit.Assert.assertTrue;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.avatar.testTools.ScriptedFootstepGenerator;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.BoundingBox3d;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationDoneCriterion;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.thread.ThreadTools;

public abstract class DRCObstacleCourseWobblyFootTest implements MultiRobotTestInterface
{
   private SimulationTestingParameters simulationTestingParameters;
   private DRCSimulationTestHelper drcSimulationTestHelper;

   @Before
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }

   @After
   public void destroySimulationAndRecycleMemory()
   {
      if (simulationTestingParameters.getKeepSCSUp())
      {
         ThreadTools.sleepForever();
      }

      // Do this here in case a test fails. That way the memory will be recycled.
      if (drcSimulationTestHelper != null)
      {
         drcSimulationTestHelper.destroySimulation();
         drcSimulationTestHelper = null;
      }

      simulationTestingParameters = null;

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }


   @ContinuousIntegrationTest(estimatedDuration = 20.5)
   @Test(timeout = 100000)
   public void testStandingForACoupleSecondsWithWobblyFeet() throws SimulationExceededMaximumTimeException
   {
      simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;

      drcSimulationTestHelper = new DRCSimulationTestHelper("DRCStandingTest", selectedLocation, simulationTestingParameters, getRobotModel());

      setupCameraForWalkingUpToRamp();

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0);

      ThreadTools.sleep(2000);


      // drcSimulationTestHelper.createVideo(getSimpleRobotName(), simulationConstructionSet, 1);
      drcSimulationTestHelper.checkNothingChanged();

      assertTrue(success);

      Point3D center = new Point3D(-8.956281888358388E-4, -3.722237566790175E-7, 0.8882009563211146);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
      BoundingBox3d boundingBox = BoundingBox3d.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }


   @ContinuousIntegrationTest(estimatedDuration = 44.4, categoriesOverride = {IntegrationCategory.FLAKY, IntegrationCategory.VIDEO})
   @Test(timeout = 143110)
   public void testWalkingUpToRampWithShortStepsWithWobblyFeet() throws SimulationExceededMaximumTimeException
   {
      simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;

      drcSimulationTestHelper = new DRCSimulationTestHelper("DRCWalkingUpToRampShortStepsTest", selectedLocation, simulationTestingParameters,
              getRobotModel());

      ScriptedFootstepGenerator scriptedFootstepGenerator = drcSimulationTestHelper.createScriptedFootstepGenerator();

      setupCameraForWalkingUpToRamp();

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0);

      FootstepDataListMessage footstepDataList = createFootstepsForWalkingUpToRampShortSteps(scriptedFootstepGenerator);
      drcSimulationTestHelper.send(footstepDataList);
      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(20.0);

      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 1);
      drcSimulationTestHelper.checkNothingChanged();

      assertTrue(success);

      Point3D center = new Point3D(3.281440097950577, 0.08837997229569997, 0.7855496116044516);
      Vector3D plusMinusVector = new Vector3D(0.3, 0.3, 0.5);
      BoundingBox3d boundingBox = BoundingBox3d.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }



   @ContinuousIntegrationTest(estimatedDuration = 47.9, categoriesOverride = {IntegrationCategory.FLAKY, IntegrationCategory.VIDEO})
   @Test(timeout = 240000)
   public void testTurningInPlaceAndPassingPIWithWobblyFeet() throws SimulationExceededMaximumTimeException
   {
      simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      FlatGroundEnvironment flatGround = new FlatGroundEnvironment();
      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT_BUT_ALMOST_PI;

      drcSimulationTestHelper = new DRCSimulationTestHelper(flatGround, "DRCTurningInPlaceAndPassingPITest", selectedLocation, simulationTestingParameters,
              getRobotModel());

      SimulationConstructionSet simulationConstructionSet = drcSimulationTestHelper.getSimulationConstructionSet();
      ScriptedFootstepGenerator scriptedFootstepGenerator = drcSimulationTestHelper.createScriptedFootstepGenerator();

      setupCameraForTurningInPlaceAndPassingPI();

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0);

      FootstepDataListMessage footstepDataList = createFootstepsForTurningInPlaceAndPassingPI(scriptedFootstepGenerator);
      drcSimulationTestHelper.send(footstepDataList);

      final DoubleYoVariable pelvisOrientationError = getPelvisOrientationErrorVariableName(simulationConstructionSet);

      SimulationDoneCriterion checkPelvisOrientationError = new SimulationDoneCriterion()
      {
         @Override
         public boolean isSimulationDone()
         {
            return (Math.abs(pelvisOrientationError.getDoubleValue()) > 0.3);
         }
      };

      simulationConstructionSet.setSimulateDoneCriterion(checkPelvisOrientationError);

      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(14.0);

      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 1);
      drcSimulationTestHelper.checkNothingChanged();

      assertTrue(success);

      Point3D center = new Point3D(-0.09807959403314585, 0.002501752329158081, 0.7867972043876718);
      Vector3D plusMinusVector = new Vector3D(0.3, 0.3, 0.5);
      BoundingBox3d boundingBox = BoundingBox3d.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }


   private void setupCameraForWalkingUpToRamp()
   {
      Point3D cameraFix = new Point3D(1.8375, -0.16, 0.89);
      Point3D cameraPosition = new Point3D(1.10, 8.30, 1.37);

      drcSimulationTestHelper.setupCameraForUnitTest(cameraFix, cameraPosition);
   }


   private void setupCameraForTurningInPlaceAndPassingPI()
   {
      Point3D cameraFix = new Point3D(0.036, 0.0, 0.89);
      Point3D cameraPosition = new Point3D(-7, -0.3575, 1.276);

      drcSimulationTestHelper.setupCameraForUnitTest(cameraFix, cameraPosition);
   }



   private FootstepDataListMessage createFootstepsForWalkingUpToRampShortSteps(ScriptedFootstepGenerator scriptedFootstepGenerator)
   {
      double[][][] footstepLocationsAndOrientations = new double[][][]
      {
         {
            {0.2148448504580547, -0.09930268518393547, 0.08399999999999999},
            {3.405174677589428E-21, -6.767715309751755E-21, 0.0025166698394258787, 0.9999968331814453}
         },
         {
            {0.4481532647842352, 0.10329823409587219, 0.08400000000000005},
            {-1.705361817083927E-23, 6.776242118837171E-21, 0.0025166698394258787, 0.9999968331814453}
         },
         {
            {0.6834821762408051, -0.09551979778612019, 0.08399999999999999},
            {3.405174677589428E-21, -6.767715309751755E-21, 0.0025166698394258787, 0.9999968331814453}
         },
         {
            {0.9167977582017036, 0.10565710343022289, 0.08400000000000005},
            {-1.705361817083927E-23, 6.776242118837171E-21, 0.0025166698394258787, 0.9999968331814453}
         },
         {
            {1.1521266696582735, -0.09316092845176947, 0.08399999999999999},
            {3.405174677589428E-21, -6.767715309751755E-21, 0.0025166698394258787, 0.9999968331814453}
         },
         {
            {1.385442251619172, 0.1080159727645736, 0.08400000000000005},
            {-1.705361817083927E-23, 6.776242118837171E-21, 0.0025166698394258787, 0.9999968331814453}
         },
         {
            {1.620771163075742, -0.09080205911741877, 0.08399999999999999},
            {3.405174677589428E-21, -6.767715309751755E-21, 0.0025166698394258787, 0.9999968331814453}
         },
         {
            {1.8540867450366407, 0.11037484209892431, 0.08400000000000005},
            {-1.705361817083927E-23, 6.776242118837171E-21, 0.0025166698394258787, 0.9999968331814453}
         },
         {
            {2.0894156564932107, -0.08844318978306806, 0.08399999999999999},
            {3.405174677589428E-21, -6.767715309751755E-21, 0.0025166698394258787, 0.9999968331814453}
         },
         {
            {2.322731238454109, 0.11273371143327501, 0.08400000000000005},
            {-1.705361817083927E-23, 6.776242118837171E-21, 0.0025166698394258787, 0.9999968331814453}
         },
         {
            {2.558060149910679, -0.08608432044871735, 0.08398952447640476},
            {-5.047008501650524E-21, 4.53358964226292E-22, 0.0025166698394258787, 0.9999968331814453}
         },
         {
            {2.7913757318715775, 0.11509258076762573, 0.08400000000000005},
            {-1.705361817083927E-23, 6.776242118837171E-21, 0.0025166698394258787, 0.9999968331814453}
         },
         {
            {3.0267046433281477, -0.08372545111436663, 0.08398952447640476},
            {-6.38257081820882E-21, -2.5377866560433405E-20, 0.0025166698394258787, 0.9999968331814453}
         },
         {
            {3.260020225289046, 0.11745145010197644, 0.08400000000000005},
            {-1.705361817083927E-23, 6.776242118837171E-21, 0.0025166698394258787, 0.9999968331814453}
         },
         {
            {3.2610268900368817, -0.08254601644719128, 0.08398952447640476},
            {3.49577202412201E-21, 2.923107094657073E-20, 0.0025166698394258787, 0.9999968331814453}
         }
      };

      RobotSide[] robotSides = drcSimulationTestHelper.createRobotSidesStartingFrom(RobotSide.RIGHT, footstepLocationsAndOrientations.length);

      return scriptedFootstepGenerator.generateFootstepsFromLocationsAndOrientations(robotSides, footstepLocationsAndOrientations);
   }



   private FootstepDataListMessage createFootstepsForTurningInPlaceAndPassingPI(ScriptedFootstepGenerator scriptedFootstepGenerator)
   {
      double[][][] footstepLocationsAndOrientations = new double[][][]
      {
         {
            {0.053884346896697966, 0.19273164589134978, 0.08574185103923426},
            {-6.938862977443471E-11, -8.7126898825953E-11, 0.9990480941331229, 0.04362230632342559}
         },
         {
            {0.05388201845443364, -0.20574623329424319, 0.08574185073944539},
            {1.6604742582112774E-10, 1.4170466407843545E-10, 0.9990483490180827, -0.04361646849807009}
         },
         {
            {0.0017235494647287533, 0.19045456181341558, 0.08574185040535603},
            {-5.0383363690493444E-11, -1.0843741493223105E-10, 0.9961949527487116, -0.0871528319562377}
         },
         {
            {0.10485496441611886, -0.19444611557725083, 0.08574185102571344},
            {1.5201027889830733E-10, 1.4860298371617872E-10, 0.9848082603764649, -0.1736453002366632}
         },
         {
            {-0.04807055917333275, 0.17475485972777594, 0.08574185070322422},
            {-3.05160242173266E-11, -1.2789253687750615E-10, 0.976296639469044, -0.21643676157587363}
         },
         {
            {0.15116636401480588, -0.17033827066486662, 0.08574185038049925},
            {1.3537219389951473E-10, 1.5295866511692108E-10, 0.9537178292633579, -0.3007030131960579}
         },
         {
            {-0.09210459251806524, 0.14670244796138915, 0.08574185100111767},
            {-1.0126547178247246E-11, -1.4515938198837407E-10, 0.9396936200915386, -0.3420173977435346}
         },
         {
            {0.18966017152321202, -0.13506560904726644, 0.0857418506668508},
            {1.1641785319333712E-10, 1.5469718133894557E-10, 0.9063090217942931, -0.42261561378428936}
         },
         {
            {-0.12737770450507258, 0.10820905279560836, 0.08574185036731347},
            {-1.0436197890210116E-11, 1.599425098341044E-10, -0.8870121823838428, 0.46174602142590504}
         },
         {
            {0.21771309767509972, -0.09103190305599193, 0.08574185095383173},
            {-9.547157074708167E-11, -1.5378878590499154E-10, -0.8433930157263759, 0.5372971440683164}
         },
         {
            {-0.15148609051286105, 0.061897935068802395, 0.085741850664082},
            {-3.082037679075772E-11, 1.7198897704022203E-10, -0.8191537200820054, 0.573574043063153}
         },
         {
            {0.23341338156809216, -0.0412379781596809, 0.08574185031046283},
            {-7.289174317616828E-11, -1.5024902169235376E-10, -0.7660463210652019, 0.6427853716307409}
         },
         {
            {-0.16278680351003783, 0.010925120900156002, 0.08574185095977704},
            {-5.067721042808702E-11, 1.8109266512133938E-10, -0.7372793107685568, 0.6755880534117237}
         },
         {
            {0.23569107567555475, 0.010922792383292988, 0.08574185059966628},
            {-4.906471775149843E-11, -1.441384550795834E-10, -0.6755923617465286, 0.7372753629070672}
         },
         {
            {-0.1605097194824509, -0.0412356760683499, 0.08574185032282551},
            {-6.966694301257708E-11, 1.87097807520974E-10, -0.6427898477118872, 0.766042565187163}
         },
         {
            {0.20979454839765582, 0.013396779318557463, 0.08574185088931394},
            {-2.91671807071375E-11, -1.3694134194254838E-10, -0.5937694707170026, 0.804635206565342}
         },
         {
            {-0.0496373406094997, -0.06666317759167362, 0.08574185062507425},
            {-7.826318574113734E-11, 1.8865011916447275E-10, -0.5937694705296589, 0.8046352067035897}
         }
      };

      RobotSide[] robotSides = drcSimulationTestHelper.createRobotSidesStartingFrom(RobotSide.RIGHT, footstepLocationsAndOrientations.length);

      return scriptedFootstepGenerator.generateFootstepsFromLocationsAndOrientations(robotSides, footstepLocationsAndOrientations);
   }


   protected abstract DoubleYoVariable getPelvisOrientationErrorVariableName(SimulationConstructionSet scs);
}
