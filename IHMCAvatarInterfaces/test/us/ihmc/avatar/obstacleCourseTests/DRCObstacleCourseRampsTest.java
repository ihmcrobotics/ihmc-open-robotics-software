package us.ihmc.avatar.obstacleCourseTests;

import static org.junit.Assert.*;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.avatar.testTools.ScriptedFootstepGenerator;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListCorruptor;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionsettools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.thread.ThreadTools;

public abstract class DRCObstacleCourseRampsTest implements MultiRobotTestInterface 
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();
   
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

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }
   
   // The default height seems to be a bit too low for the ramp
//   private final ComHeightPacket comHeightPacket = new ComHeightPacket(0.05, 1.0);


	@ContinuousIntegrationTest(estimatedDuration = 57.9)
	@Test(timeout = 290000)
   public void testWalkingUpRampWithShortSteps() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
      doUpRampTest(null, StepLength.SHORT);
      
      Point3D center = new Point3D(6.135997212353164, 0.008329425009630976, 1.3724038542384285);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);
      
      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

	@ContinuousIntegrationTest(estimatedDuration = 73.5)
	@Test(timeout = 370000)
   public void testWalkingDownRampWithMediumSteps() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
      
      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.RAMP_TOP;
      
      drcSimulationTestHelper = new DRCSimulationTestHelper("DRCWalkingDownRampWithMediumSteps", selectedLocation,  simulationTestingParameters, getRobotModel());
      
      SimulationConstructionSet simulationConstructionSet = drcSimulationTestHelper.getSimulationConstructionSet();
      ScriptedFootstepGenerator scriptedFootstepGenerator = drcSimulationTestHelper.createScriptedFootstepGenerator();

      setupCameraForWalkingOverRamp(simulationConstructionSet);
      ThreadTools.sleep(1000);

      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0);

      FootstepDataListMessage footstepDataList = createFootstepsForWalkingDownRampMediumSteps(scriptedFootstepGenerator);
      drcSimulationTestHelper.send(footstepDataList);

      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(18.0);

      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 1);
      drcSimulationTestHelper.checkNothingChanged();

      assertTrue(success);
      
      Point3D center = new Point3D(3.36, 0.0212, 0.993);
      Vector3D plusMinusVector = new Vector3D(0.3, 0.3, 0.5);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);
      
      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

	@ContinuousIntegrationTest(estimatedDuration = 61.8)
	@Test(timeout = 310000)
   public void testWalkingUpRampWithMediumSteps() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
      doUpRampTest(null, StepLength.MEDIUM);
      
      Point3D center = new Point3D(7.579638943201888, 0.020725665285290903, 1.46537366331119);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);
      
      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

	@ContinuousIntegrationTest(estimatedDuration = 76.9)
	@Test(timeout = 380000)
   public void testWalkingUpRampWithShortStepsALittleTooHigh() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      Vector3D minLocationCorruption = new Vector3D(0.0, 0.0, 0.0);
      Vector3D maxLocationCorruption = new Vector3D(0.0, 0.0, 0.05);
      double maxRotationCorruption = getMaxRotationCorruption();
      FootstepDataListCorruptor footstepDataListCorruptor = new FootstepDataListCorruptor(minLocationCorruption, maxLocationCorruption, maxRotationCorruption);

      doUpRampTest(footstepDataListCorruptor, StepLength.SHORT);

      Point3D center = new Point3D(6.135997212353164, 0.008329425009630976, 1.3724038542384285);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

	@ContinuousIntegrationTest(estimatedDuration = 68.7)
	@Test(timeout = 340000)
   public void testWalkingUpRampWithShortStepsALittleTooLow() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      Vector3D minLocationCorruption = new Vector3D(0.0, 0.0, -0.06);
      Vector3D maxLocationCorruption = new Vector3D(0.0, 0.0, 0.0);
      double maxRotationCorruption = getMaxRotationCorruption();
      FootstepDataListCorruptor footstepDataListCorruptor = new FootstepDataListCorruptor(minLocationCorruption, maxLocationCorruption, maxRotationCorruption);

      doUpRampTest(footstepDataListCorruptor, StepLength.SHORT);
      
      Point3D center = new Point3D(6.135997212353164, 0.008329425009630976, 1.3724038542384285);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   private enum StepLength
   {
      SHORT, MEDIUM, LONG;
   }
   
   private void doUpRampTest(FootstepDataListCorruptor footstepDataListCorruptor, StepLength stepLength) throws SimulationExceededMaximumTimeException
   {
      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.RAMP_BOTTOM;
      
      drcSimulationTestHelper = new DRCSimulationTestHelper("DRCWalkingUpRampTest", selectedLocation, simulationTestingParameters, getRobotModel());

      SimulationConstructionSet simulationConstructionSet = drcSimulationTestHelper.getSimulationConstructionSet();
      ScriptedFootstepGenerator scriptedFootstepGenerator = drcSimulationTestHelper.createScriptedFootstepGenerator();

//      drcSimulationTestHelper.send(comHeightPacket);

      setupCameraForWalkingOverRamp(simulationConstructionSet);
      ThreadTools.sleep(1000);

      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0);

      FootstepDataListMessage footstepDataList = createFootstepsForWalkingUpRamp(scriptedFootstepGenerator, stepLength);

      if (footstepDataListCorruptor != null)
      {
         footstepDataList = footstepDataListCorruptor.corruptDataList(footstepDataList);
      }

      drcSimulationTestHelper.send(footstepDataList);

      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(16.0);

      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 2);

      drcSimulationTestHelper.checkNothingChanged();
      assertTrue(success);
   }

   private FootstepDataListMessage createFootstepsForWalkingUpRamp(ScriptedFootstepGenerator scriptedFootstepGenerator, StepLength stepLength)
   {
      switch(stepLength)
      {
      case SHORT:
      {
         return createFootstepsForWalkingUpRampShortSteps(scriptedFootstepGenerator);
      }
      
      case MEDIUM:
      {
         return createFootstepsForWalkingUpRampMediumSteps(scriptedFootstepGenerator);
      }
      case LONG:
      {
         return createFootstepsForWalkingUpRampLongSteps(scriptedFootstepGenerator);
      }
      
      }
      
      throw new RuntimeException();
   }

   private void setupCameraForWalkingOverRamp(SimulationConstructionSet scs)
   {
      Point3D cameraFix = new Point3D(5.0, -0.2, 0.89);
      Point3D cameraPosition = new Point3D(5.0, 7.8, 1.6);

      drcSimulationTestHelper.setupCameraForUnitTest(cameraFix, cameraPosition);
   }
   
   private void setupCameraForWalkingOverEasySteppingStones(SimulationConstructionSet scs)
   {
      Point3D cameraFix = new Point3D(-8.6, -0.1, 0.94);
      Point3D cameraPosition = new Point3D(-14.0, -5.0, 2.7);

      drcSimulationTestHelper.setupCameraForUnitTest(cameraFix, cameraPosition);
   }

   
   private FootstepDataListMessage createFootstepsForWalkingUpRampShortSteps(ScriptedFootstepGenerator scriptedFootstepGenerator)
   {
      double[][][] footstepLocationsAndOrientations = new double[][][]
      {
         {
            {2.8351427759651995, -0.09694911182412484, 0.08802441745426925},
            {-0.003121843074205739, 0.013029344215583364, 0.003893401251972101, 0.9999026611184826}
         },
         {
            {3.074520627354574, 0.10730668915721077, 0.08352181741194085},
            {-0.003138368218867929, -0.004117464859445681, 0.003840685316534686, 0.9999792229163331}
         },
         {
            {3.320864939704435, -0.09193371045698896, 0.08507793790294434},
            {0.003584052020572927, 0.0017180402341382128, 0.0038475364764751895, 0.9999846995689133}
         },
         {
            {3.545678648483338, 0.11240071557575276, 0.09590741902024505},
            {-0.01140194817540447, -0.10273541702708931, 0.002615002207584757, 0.9946399305424701}
         },
         {
            {3.7906598672926064, -0.08710050380011673, 0.14539339478058277},
            {-0.0032000970795792102, -0.10503815495588137, 0.0034516427122286324, 0.9944570536452203}
         },
         {
            {4.033521430244125, 0.11399420124778985, 0.19672970285385633},
            {0.0014221699189987877, -0.10151659122040031, 0.003938979344512634, 0.9948250316419642}
         },
         {
            {4.278848103525166, -0.0843365855044057, 0.24733780119159066},
            {0.002700255557592924, -0.1017216320343805, 0.0040697254090160066, 0.9948009125102776}
         },
         {
            {4.52241785253613, 0.11448913357058678, 0.29975135146746346}, {0.020795142183303052, -0.09404379375500953, 0.005767914571257194, 0.9953341439332021}
         },
         {
            {4.7669499200426095, -0.08112143629512504, 0.34980817204772424},
            {0.005929619423401327, -0.09892494034464223, 0.004386419554999209, 0.9950675630904617}
         },
         {
            {5.007512119256335, 0.11963061986871831, 0.39761424662922423},
            {0.012520157577486044, -0.10934673784718243, 0.005161876711006989, 0.9939114103405972}
         },
         {
            {5.253783473785037, -0.07589729216272391, 0.4503040379818391},
            {-0.002768215802721581, -0.1037404647965144, 0.003502485316391724, 0.9945943824201307}
         },
         {
            {5.496393274813743, 0.12528689501201606, 0.5016796856627599},
            {0.0013191212731254657, -0.10174062303095144, 0.003928498337639808, 0.9948023283271665}
         },
         {
            {5.742142760881751, -0.07169360534955915, 0.5523316338352129},
            {-0.005390474700669975, -0.09936215014013182, 0.003258174259210824, 0.9950314016163148}
         },
         {
            {5.983218795105767, 0.1272533065544719, 0.6018939390774926}, {0.01190743053633726, -0.1067114604566984, 0.005065817370162187, 0.9942058211454975}
         },
         {
            {6.227865242475841, -0.06899602335005377, 0.6534736822900773},
            {8.499381935746491E-4, -0.11096744195086783, 0.003877037816252163, 0.9938161162960275}
         },
         {
            {6.227890274639702, 0.13309848577114403, 0.6545998273165854}, {-0.011553591447604863, -0.1014539720762872, 0.002615910457716054, 0.9947697035430276}
         }
      };

      RobotSide[] robotSides = drcSimulationTestHelper.createRobotSidesStartingFrom(RobotSide.RIGHT, footstepLocationsAndOrientations.length);

      return scriptedFootstepGenerator.generateFootstepsFromLocationsAndOrientations(robotSides, footstepLocationsAndOrientations);
   }

   
   private FootstepDataListMessage createFootstepsForWalkingUpRampMediumSteps(ScriptedFootstepGenerator scriptedFootstepGenerator)
   {
      double[][][] footstepLocationsAndOrientations = new double[][][]
   {{{2.96830428414849, -0.09766543994755546, 0.08399999999999998}, {-2.720672111457406E-20, 5.415915594351002E-20, 0.0018763016130482887, 0.9999982397445792}},
      {{3.349989144127723, 0.1048803018182974, 0.084}, {0.0, 0.0, 0.0018763016130482887, 0.9999982397445792}},
      {{3.733179215666347, -0.09368313850247431, 0.08399999999999998}, {-2.720672111457406E-20, 5.415915594351002E-20, 0.0018763016130482887, 0.9999982397445792}},
      {{4.097845385981999, 0.10855694052559439, 0.21126371895363616}, {-0.004961863853467075, -0.10185092940802969, 0.0013389870407204197, 0.9947863967704449}},
      {{4.482409385830916, -0.08979946125987102, 0.2907229850807759}, {-0.006178121664549205, -0.09355272624775302, 0.001271081900870645, 0.9955943463957156}},
      {{4.862209672750061, 0.10835917126334947, 0.36806892250913914}, {0.013196743471666848, -0.1050242695228723, 0.003239072589792082, 0.9943768185057856}},
      {{5.245124320290557, -0.0877310654977739, 0.44776346824345264}, {-0.001447090852386699, -0.1066362325507391, 0.001688950777778143, 0.9942956136284218}},
      {{5.628010222560295, 0.11351051235407777, 0.529646092583943}, {-2.9713796970907996E-4, -0.09939803265092252, 0.001818698125945603, 0.9950460467492345}},
      {{6.00919845769285, -0.08799087419764393, 0.6029648538145679}, {0.0170627239767379, -0.11160025921878289, 0.003757776815021594, 0.9935995796628553}},
      {{6.397906847050047, 0.11747882088357856, 0.6897727409562333}, {-0.006763453237380005, -0.06919806861450803, 0.0013936914450255998, 0.997579039788068}},
      {{6.791772471706161, -0.0820029184512979, 0.709914829576844}, {-0.0011948636118072641, -0.005492872382347826, 0.0018696547555749577, 0.999982452368558}},
      {{7.16726116567599, 0.13224394932170272, 0.6980262408021847}, {-0.07769715333044895, -0.04268274134711664, -0.0014525809551700932, 0.9960618585027782}},
      {{7.555321531515414, -0.07933162374116386, 0.7110548590792264}, {-2.5158057233426777E-5, -0.013413125296120916, 0.0018754577477367784, 0.9999082808413475}},
      {{7.5508615685355345, 0.11059069063054038, 0.7018996990839376}, {0.059980529587038575, -0.035821498052187935, 0.00402990721894804, 0.9975484530565731}}
      };
      
      RobotSide[] robotSides = drcSimulationTestHelper.createRobotSidesStartingFrom(RobotSide.RIGHT, footstepLocationsAndOrientations.length);
      return scriptedFootstepGenerator.generateFootstepsFromLocationsAndOrientations(robotSides, footstepLocationsAndOrientations);
   }
   
   private FootstepDataListMessage createFootstepsForWalkingUpRampLongSteps(ScriptedFootstepGenerator scriptedFootstepGenerator)
   {
      double[][][] footstepLocationsAndOrientations = new double[][][]
   {{{}}
      };
      
      RobotSide[] robotSides = drcSimulationTestHelper.createRobotSidesStartingFrom(RobotSide.RIGHT, footstepLocationsAndOrientations.length);
      return scriptedFootstepGenerator.generateFootstepsFromLocationsAndOrientations(robotSides, footstepLocationsAndOrientations);
   }
   
   private FootstepDataListMessage createFootstepsForWalkingDownRampMediumSteps(ScriptedFootstepGenerator scriptedFootstepGenerator)
   {
      double[][][] footstepLocationsAndOrientations = new double[][][]
            {{{6.954747596485718, 0.11524925726571199, 0.7119679417361913}, {-0.004756259680920303, -1.677100279548098E-5, 0.9999824722629655, 0.003526028846796884}},
            {{6.570706490915851, -0.09613984168168907, 0.7111802069405166}, {-0.027270597709182255, -6.794653657152437E-4, 0.9996217167026284, 0.003503758537748503}},
            {{6.174537167241603, 0.10663518963448074, 0.6342597541404246}, {-0.14073889731487735, -4.131199692770531E-4, 0.9900409537060528, 0.003361859499252677}},
            {{5.804220529757834, -0.09056467012091034, 0.5718294010093754}, {-0.08100673557592118, 5.176508612160084E-4, 0.9967071562119236, 0.0035334947090369267}},
            {{5.419026368647578, 0.11198079913421716, 0.4697549077307898}, {-0.12855198732280726, -3.5547712115239664E-4, 0.9916969061305156, 0.0033921324467669225}},
            {{5.04723426554032, -0.08504675124055729, 0.39067108046147253}, {-0.07791096128552895, 0.0015702359675104768, 0.996952524232402, 0.0036167523018496104}},
            {{4.671487880420936, 0.11995026935199998, 0.3352839308619279}, {-0.06841841814321405, 0.015447288875779376, 0.9975266893276052, 0.004561294144847291}},
            {{4.288674875687302, -0.07995257295142243, 0.2710370124710936}, {-0.08426182615512512, 7.579942593184682E-5, 0.9964375163946988, 0.0034949719047255292}},
            {{3.911931009786938, 0.1258873396387373, 0.15974401557183804}, {-0.08078431565658183, 0.01892061868156026, 0.9965393339623263, 0.00502597239624625}},
            {{3.5409933792461827, -0.07371958963330688, 0.08965431506314861}, {-0.0255348909645515, 0.005579759512198356, 0.999651639718129, 0.0036653561996753147}},
            {{3.170499491279822, 0.12801175184184635, 0.0825363135740276}, {0.015249734772657272, 1.605057623917769E-4, 0.9998774982301046, 0.0035225502652209764}},
            {{3.1682135358725176, -0.07185742849219595, 0.0836276175220112}, {0.010034721188456488, 8.911915700751356E-4, 0.9999430696742437, 0.0035167540013307935}},
            };
      RobotSide[] robotSides = drcSimulationTestHelper.createRobotSidesStartingFrom(RobotSide.RIGHT, footstepLocationsAndOrientations.length);
      
      return scriptedFootstepGenerator.generateFootstepsFromLocationsAndOrientations(robotSides, footstepLocationsAndOrientations);
   }
   
   protected abstract double getMaxRotationCorruption();

}
