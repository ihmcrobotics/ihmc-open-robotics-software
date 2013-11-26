package us.ihmc.darpaRoboticsChallenge.obstacleCourseTests;

import static org.junit.Assert.assertTrue;

import javax.vecmath.Point3d;

import org.junit.After;
import org.junit.Before;
import org.junit.Ignore;
import org.junit.Test;

import us.ihmc.bambooTools.BambooTools;
import us.ihmc.commonWalkingControlModules.desiredFootStep.dataObjects.FootstepDataList;
import us.ihmc.darpaRoboticsChallenge.DRCDemo01StartingLocation;
import us.ihmc.darpaRoboticsChallenge.DRCEnvironmentModel;
import us.ihmc.darpaRoboticsChallenge.testTools.DRCSimulationTestHelper;
import us.ihmc.darpaRoboticsChallenge.testTools.ScriptedFootstepGenerator;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.MemoryTools;
import us.ihmc.utilities.ThreadTools;

import com.yobotics.simulationconstructionset.SimulationConstructionSet;
import com.yobotics.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

public class DRCObstacleCourseTrialsTerrainTest
{
   private static final boolean KEEP_SCS_UP = false;

   private static final boolean createMovie = BambooTools.doMovieCreation();
   private static final boolean checkNothingChanged = BambooTools.getCheckNothingChanged();

   private DRCSimulationTestHelper drcSimulationTestHelper;

   @Before
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }

   @After
   public void destroySimulationAndRecycleMemory()
   {
      if (KEEP_SCS_UP)
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


   @Ignore
   @Test
   public void testWalkingOntoAndOverSlopes() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      DRCDemo01StartingLocation selectedLocation = DRCDemo01StartingLocation.DRC_TRIALS_TRAINING_WALKING;
      DRCEnvironmentModel selectedEnvironment = DRCEnvironmentModel.OBSTACLE_COURSE;
      drcSimulationTestHelper = new DRCSimulationTestHelper("DRCWalkingOntoSlopesTest", selectedLocation, selectedEnvironment, checkNothingChanged,
              createMovie);

      SimulationConstructionSet simulationConstructionSet = drcSimulationTestHelper.getSimulationConstructionSet();
      ScriptedFootstepGenerator scriptedFootstepGenerator = drcSimulationTestHelper.createScriptedFootstepGenerator();

      setupCameraForWalkingOntoSlopes(simulationConstructionSet);

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0);

      FootstepDataList footstepDataList = createFootstepsForWalkingToTheSlopesNormally(scriptedFootstepGenerator);
      drcSimulationTestHelper.sendFootstepListToListeners(footstepDataList);

      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(6.5);

      if (success)
      {
         footstepDataList = createFootstepsForSteppingOverTheSlopeEdge(scriptedFootstepGenerator);
         drcSimulationTestHelper.sendFootstepListToListeners(footstepDataList);

         success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(6.0);
      }

      drcSimulationTestHelper.createMovie(simulationConstructionSet, 1);
      drcSimulationTestHelper.checkNothingChanged();

      assertTrue(success);

      BambooTools.reportTestFinishedMessage();
   }

   @Ignore
   @Test
   public void testWalkingOverSlopesInOneGo() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      DRCDemo01StartingLocation selectedLocation = DRCDemo01StartingLocation.DRC_TRIALS_TRAINING_WALKING;
      DRCEnvironmentModel selectedEnvironment = DRCEnvironmentModel.OBSTACLE_COURSE;
      drcSimulationTestHelper = new DRCSimulationTestHelper("DRCWalkingOntoSlopesTest", selectedLocation, selectedEnvironment, checkNothingChanged,
              createMovie);

      SimulationConstructionSet simulationConstructionSet = drcSimulationTestHelper.getSimulationConstructionSet();
      ScriptedFootstepGenerator scriptedFootstepGenerator = drcSimulationTestHelper.createScriptedFootstepGenerator();

      setupCameraForWalkingOntoSlopes(simulationConstructionSet);

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0);

      FootstepDataList footstepDataList = createFootstepsForWalkingOverTheSlopesInOneGo(scriptedFootstepGenerator);
      drcSimulationTestHelper.sendFootstepListToListeners(footstepDataList);

      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(45.0);

      drcSimulationTestHelper.createMovie(simulationConstructionSet, 1);
      drcSimulationTestHelper.checkNothingChanged();

      assertTrue(success);

      BambooTools.reportTestFinishedMessage();
   }

   @Test
   public void testWalkingOntoAndOverSlopesSideways() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      DRCDemo01StartingLocation selectedLocation = DRCDemo01StartingLocation.DRC_TRIALS_TRAINING_WALKING;
      DRCEnvironmentModel selectedEnvironment = DRCEnvironmentModel.OBSTACLE_COURSE;
      drcSimulationTestHelper = new DRCSimulationTestHelper("DRCWalkingOntoSlopesTest", selectedLocation, selectedEnvironment, checkNothingChanged,
              createMovie);

      SimulationConstructionSet simulationConstructionSet = drcSimulationTestHelper.getSimulationConstructionSet();
      ScriptedFootstepGenerator scriptedFootstepGenerator = drcSimulationTestHelper.createScriptedFootstepGenerator();

      setupCameraForWalkingOntoSlopes(simulationConstructionSet);

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0);

      FootstepDataList footstepDataList = createFootstepsForWalkingToTheSlopesSideways(scriptedFootstepGenerator);
      drcSimulationTestHelper.sendFootstepListToListeners(footstepDataList);

      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(20.0);

      if (success)
      {
         footstepDataList = createFootstepsForSteppingOverTheSlopesEdgeSideways(scriptedFootstepGenerator);
         drcSimulationTestHelper.sendFootstepListToListeners(footstepDataList);

         success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(8.0);
      }

      drcSimulationTestHelper.createMovie(simulationConstructionSet, 1);
      drcSimulationTestHelper.checkNothingChanged();

      assertTrue(success);

      BambooTools.reportTestFinishedMessage();
   }

   private void setupCameraForWalkingOntoSlopes(SimulationConstructionSet scs)
   {
      Point3d cameraFix = new Point3d(0.1, 3.2, 0.5);
      Point3d cameraPosition = new Point3d(-2.8, 4.8, 1.5);

      drcSimulationTestHelper.setupCameraForUnitTest(scs, cameraFix, cameraPosition);
   }


   private FootstepDataList createFootstepsForWalkingToTheSlopesNormally(ScriptedFootstepGenerator scriptedFootstepGenerator)
   {
      double[][][] footstepLocationsAndOrientations = new double[][][]
      {
         {
            {2.1265936534673218, 2.3428927767215417, 0.08591277243384399}, {0.020704851858501554, -0.008524153124099326, 0.3678797475257915, 0.9296037539099088}
         },
         {
            {2.6166814909099094, 2.3996906331962657, 0.08226766576256324},
            {2.4508705508237736E-4, 7.204771499494282E-4, 0.35318402355492856, 0.9355535614546948}
         },
         {
            {2.739624824329704, 2.8475642035481017, 0.09205292111010316}, {0.03188071876413438, -0.038174623620904354, 0.3675020633742088, 0.928691849484092}
         },
         {
            {3.193896638477976, 2.9213552160981826, 0.17477524869701494}, {0.04675359881912196, -0.116811005963886, 0.34844159401254376, 0.9288475361678918}
         },
         {
            {3.017845920846812, 3.1060500699939313, 0.17155859233956158}, {0.03567318375247706, -0.13777549770429312, 0.36045274233562125, 0.9218563644820306}
         },
      };

      RobotSide[] robotSides = drcSimulationTestHelper.createRobotSidesStartingFrom(RobotSide.LEFT, footstepLocationsAndOrientations.length);

      return scriptedFootstepGenerator.generateFootstepsFromLocationsAndOrientations(robotSides, footstepLocationsAndOrientations);
   }

   private FootstepDataList createFootstepsForSteppingOverTheSlopeEdge(ScriptedFootstepGenerator scriptedFootstepGenerator)
   {
      double[][][] footstepLocationsAndOrientations = new double[][][]
      {
         {
            {3.0196365393148823, 3.157527589669437, 0.18463968610481485}, {0.06878939191058485, -0.14859257123868208, 0.36996761202767287, 0.9145010844082091}
         },
         {
            {3.242221271901224, 2.9525762835818927, 0.19170633363319947}, {0.05074043948683175, -0.1388048392204765, 0.35405122373484255, 0.9234751514694487}
         },
         {
            {3.2884777064719466, 3.4423998984979947, 0.21600585958795815}, {-0.09042178480364092, 0.12404908940997492, 0.3587959323612516, 0.9207069040528045}
         },
         {
            {3.504460942426418, 3.2186466197482773, 0.21310781108121274}, {-0.10127177343643246, 0.10226561231768722, 0.3474545483212657, 0.9265857268991324}
         },
      };

      RobotSide[] robotSides = drcSimulationTestHelper.createRobotSidesStartingFrom(RobotSide.LEFT, footstepLocationsAndOrientations.length);

      return scriptedFootstepGenerator.generateFootstepsFromLocationsAndOrientations(robotSides, footstepLocationsAndOrientations);
   }

   private FootstepDataList createFootstepsForWalkingOverTheSlopesInOneGo(ScriptedFootstepGenerator scriptedFootstepGenerator)
   {
      double[][][] footstepLocationsAndOrientations = new double[][][]
      {
         {
            {2.3042952395272107, 2.099103639312993, 0.08300550115576277}, {0.008564712511807942, 0.020796880999479492, 0.39719493301084063, 0.9174586206640588}
         },
         {
            {2.2438392223423667, 2.4301081835735907, 0.08403924179877988}, {-2.3541587312585044E-5, 6.52874227824117E-5, 0.3826816600080084, 0.9238802640368833}
         },
         {
            {2.559957386054418, 2.3968313267771966, 0.08368946845620837}, {0.020708747923442607, -0.008521572992644433, 0.39719568008053274, 0.9174606924977972}
         },
         {
            {2.510944183358525, 2.71733237800806, 0.08573501648545975}, {-2.3541587312643723E-5, 6.528742278226826E-5, 0.3826816600080083, 0.9238802640368833}
         },
         {
            {2.8326089239049694, 2.689602000226789, 0.09142682048335463}, {-0.008609706068681243, -0.020665608404279303, 0.3971977519142708, 0.9174599454281049}
         },
         {
            {2.772493724911711, 2.98733761407194, 0.10631548779912296}, {0.05009344659025468, -0.11569658425567451, 0.3814764712443284, 0.9157404921085744}
         },
         {
            {3.073622258730516, 2.939977143309511, 0.15201480268475412}, {0.06882173350957592, -0.12492489579563805, 0.39690338131292324, 0.9067111145836662}
         },
         {
            {2.981204100121291, 3.22650456802866, 0.19488817010700143}, {0.05009344659025461, -0.11569658425567463, 0.38147647124432826, 0.9157404921085744}
         },
         {
            {3.1478581820011984, 3.077098056356127, 0.19780760848662082}, {0.027655797469679846, -0.10797053956021793, 0.39482152866572157, 0.91197230218656}
         },
         {
            {3.1978146686477316, 3.490781378124674, 0.21920041678533939}, {-0.022122479179226193, 0.13204979941530764, 0.3801610263565129, 0.9151781468265643}
         },
         {
            {3.3718549202145276, 3.325866285074077, 0.21902384088506444}, {-0.069544360689838, 0.1286465338790443, 0.40007058881209756, 0.9047414963317927}
         },
         {
            {3.3537557389708916, 3.6140416694284205, 0.1632400370292026}, {-0.05085584995787939, 0.11948685679992167, 0.38462614904803605, 0.9138923892043388}
         },
         {
            {3.553037582619451, 3.4644533438527967, 0.14766935679714555}, {-0.02833712041897954, 0.11185077742051305, 0.3978926496409115, 0.9101471587378175}
         },
         {
            {3.590341927483232, 3.8921459196255994, 0.1054025098673869}, {0.05121228832443446, -0.11547318705195618, 0.38034423542331747, 0.916177661360989}
         },
         {
            {3.7559171159136944, 3.7304646882065513, 0.1111273005664001}, {0.06993266865319621, -0.12474423743929339, 0.3957572123950205, 0.9071518758830045}
         },
         {
            {3.787138855941054, 4.114780949956089, 0.18245412905276812}, {0.05121228832443444, -0.11547318705195635, 0.3803442354233172, 0.916177661360989}
         },
         {
            {3.9694344486636455, 3.95876822138877, 0.18963931728265956}, {0.05786280916869607, -0.0956702181547433, 0.3973823852421062, 0.9108163067120754}
         },
         {
            {4.062521781233412, 4.400590147135958, 0.20540085578980427}, {-0.05166464606966071, 0.11915439876125945, 0.38358407269941464, 0.9143283068863556}
         },
         {
            {4.219699221659784, 4.25282703993481, 0.20816320433604793}, {-0.08710349428188968, 0.08707483700493901, 0.4040318177114087, 0.906415602425986}
         },
         {
            {4.139674102665004, 4.4851238194914425, 0.17471865565945816}, {-0.038855976356443125, 0.0904190923684487, 0.3863454391072473, 0.9170887647974332}
         },
         {
            {4.27064450721793, 4.321769835013736, 0.18785385709416594}, {-0.07027779971217145, 0.12839647920949127, 0.39993352251567393, 0.9047809417546478}
         },
      };

      RobotSide[] robotSides = drcSimulationTestHelper.createRobotSidesStartingFrom(RobotSide.RIGHT, footstepLocationsAndOrientations.length);

      return scriptedFootstepGenerator.generateFootstepsFromLocationsAndOrientations(robotSides, footstepLocationsAndOrientations);
   }

   private FootstepDataList createFootstepsForWalkingToTheSlopesSideways(ScriptedFootstepGenerator scriptedFootstepGenerator)
   {
      double[][][] footstepLocationsAndOrientations = new double[][][]
      {
         {
            {1.9823966635641284, 2.2652470283072947, 0.0837307038087527}, {0.020640132711869014, -0.008498293160241518, 0.3083105864353867, 0.951023841040206}
         },
         {
            {2.1498164650868086, 1.9068093446433945, 0.08347777099109424},
            {0.0017662883179677942, 0.006293715779592783, 0.10348136532245923, 0.9946099116730455}
         },
         {
            {2.0874687872550273, 2.298441155960018, 0.08427730698998047},
            {0.018633535386299895, -0.007684905338928583, 0.024333507423710935, 0.9995006823436388}
         },
         {
            {2.0314602664501606, 1.9146192377173497, 0.08341613830944179},
            {0.0034096687293227626, 0.011999890346433685, -0.18430794544823617, 0.9827893762325068}
         },
         {
            {2.2034072670888567, 2.281143757931813, 0.08394437770346314}, {0.015080397973109637, -0.006233688252807148, -0.26166319609415833, 0.96502129227159}
         },
         {
            {2.01915833306869, 2.0207468649842344, 0.08359962278744236}, {0.004579288707518639, 0.014818193944121557, -0.36640082496031345, 0.9303278382976451}
         },
         {
            {2.3514127370083746, 2.3597369501954506, 0.0860154559910794},
            {0.013290279881016632, -0.005689963592658761, -0.38133174097309863, 0.9243252112224485}
         },
         {
            {2.25188495618737, 2.254080704383737, 0.08394252424978803}, {-0.007521908318387368, -0.014514380020142312, -0.36682257924428485, 0.9301472727608521}
         },
         {
            {2.5571508061806436, 2.575341468058366, 0.08684300869106024}, {0.01329027988101649, -0.005689963592658824, -0.3813317409730987, 0.9243252112224485}
         },
         {
            {2.457577639970055, 2.4696407711863455, 0.0869664911152282}, {0.004579288707518356, 0.014818193944121443, -0.3664008249603138, 0.930327838297645}
         },
         {
            {2.768315526189699, 2.785782484898131, 0.08433104034516861}, {0.013290279881016347, -0.0056899635926588865, -0.3813317409730988, 0.9243252112224485}
         },
         {
            {2.5635712023164903, 2.5743143400835953, 0.0870408732178602}, {0.0045792887075182115, 0.014818193944121382, -0.36640082496031384, 0.930327838297645}
         },
         {
            {2.885879020571682, 2.856576938293644, 0.10386656274046036}, {0.12558777759117537, -0.06679059297300634, -0.3513269921130159, 0.92538428310775}
         },
         {
            {2.7488021282402655, 2.6966826406653803, 0.08370236609247309},
            {-0.002071973823236298, -0.003865630015040666, -0.34064306248913384, 0.9401824651667817}
         },
         {
            {2.986089110278983, 2.9635615047422394, 0.14058922355834996}, {0.10717475880427008, -0.026653519731933816, -0.35290983266496895, 0.9291166831832962}
         },
         {
            {2.844078254699551, 2.7987722839957434, 0.08895082990782543}, {0.05682759834686232, -0.026260994395297752, -0.34066061044879653, 0.9380998522162506}
         },
         {
            {3.079930510367088, 3.075738188913338, 0.18223629720937254}, {0.09637509142099876, -0.05639328437007134, -0.3547113886462364, 0.9282841536922889}
         },
         {
            {2.9344673839516484, 2.905450415197158, 0.12337655475135587}, {0.11580121216799653, -0.04880027856780968, -0.33742432635314157, 0.9329273476842961}
         },
         {
            {3.128623440850548, 3.133453453117145, 0.20285914961446738}, {0.12598064593469932, -0.06710112170905909, -0.35316275861517443, 0.9246093133008028}
         },
      };

      RobotSide[] robotSides = drcSimulationTestHelper.createRobotSidesStartingFrom(RobotSide.LEFT, footstepLocationsAndOrientations.length);

      return scriptedFootstepGenerator.generateFootstepsFromLocationsAndOrientations(robotSides, footstepLocationsAndOrientations);
   }

   private FootstepDataList createFootstepsForSteppingOverTheSlopesEdgeSideways(ScriptedFootstepGenerator scriptedFootstepGenerator)
   {
      double[][][] footstepLocationsAndOrientations = new double[][][]
      {
         {
            {3.0166188930803033, 3.0119111124382747, 0.15483943760187868}, {0.1089192633351566, -0.024058516458074313, -0.36480741929991994, 0.9243772653435911}
         },
         {
            {3.285102645187515, 3.361157755301027, 0.22549038617963604}, {-0.12112164390947337, 0.0472878892769468, -0.34692969244751093, 0.9288343185965263}
         },
         {
            {3.1055260564624887, 3.160607633126951, 0.20546113718754253}, {0.12642092274810612, -0.06414867390038669, -0.3595546575929555, 0.9222923322523894}
         },
         {
            {3.3695983984590763, 3.4737424555716165, 0.18833480541902758}, {-0.12112164390947357, 0.04728788927694677, -0.3469296924475111, 0.9288343185965262}
         },
         {
            {3.2811041904535196, 3.3537632182460775, 0.22458026669614373}, {-0.11259311979747, 0.025105614756717354, -0.36036496334456175, 0.9256509010829258}
         },
      };

      RobotSide[] robotSides = drcSimulationTestHelper.createRobotSidesStartingFrom(RobotSide.RIGHT, footstepLocationsAndOrientations.length);

      return scriptedFootstepGenerator.generateFootstepsFromLocationsAndOrientations(robotSides, footstepLocationsAndOrientations);
   }

}
