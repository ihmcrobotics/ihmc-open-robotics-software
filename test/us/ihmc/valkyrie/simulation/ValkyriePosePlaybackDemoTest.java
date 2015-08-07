package us.ihmc.valkyrie.simulation;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.commonWalkingControlModules.posePlayback.PlaybackPose;
import us.ihmc.commonWalkingControlModules.posePlayback.PlaybackPoseSequence;
import us.ihmc.commonWalkingControlModules.posePlayback.PosePlaybackPacket;
import us.ihmc.darpaRoboticsChallenge.DRCGuiInitialSetup;
import us.ihmc.darpaRoboticsChallenge.DRCPosePlaybackDemo;
import us.ihmc.darpaRoboticsChallenge.DRCSCSInitialSetup;
import us.ihmc.darpaRoboticsChallenge.DRCSimulationFactory;
import us.ihmc.darpaRoboticsChallenge.initialSetup.DRCRobotInitialSetup;
import us.ihmc.graphics3DAdapter.GroundProfile3D;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.util.ground.FlatGroundProfile;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationRunner.ControllerFailureException;
import us.ihmc.utilities.MemoryTools;
import us.ihmc.tools.random.RandomTools;
import us.ihmc.tools.thread.ThreadTools;
import us.ihmc.tools.agileTesting.BambooAnnotations.EstimatedDuration;
import us.ihmc.humanoidRobotics.model.FullRobotModel;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.TotalMassCalculator;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.posePlayback.ValkyrieWarmupPoseSequencePacket;
import us.ihmc.yoUtilities.controllers.GainCalculator;
import us.ihmc.yoUtilities.humanoidRobot.visualizer.RobotVisualizer;
import us.ihmc.yoUtilities.time.GlobalTimer;

public class ValkyriePosePlaybackDemoTest
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();   
   private BlockingSimulationRunner blockingSimulationRunner;

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
      if (blockingSimulationRunner != null)
      {
         blockingSimulationRunner.destroySimulation();
         blockingSimulationRunner = null;
      }
      
      GlobalTimer.clearTimers();
      
      

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }
   
   private DRCSimulationFactory drcSimulation;
   private RobotVisualizer robotVisualizer;
   private final Random random = new Random(6519651L);

   @After
   public void tearDown()
   {     
      if (drcSimulation != null)
      {
         drcSimulation.dispose();
         drcSimulation = null;
      }

      if (robotVisualizer != null)
      {
         robotVisualizer.close();
         robotVisualizer = null;
      }
   }

	@EstimatedDuration(duration = 24.2)
	@Test(timeout = 120970)
   public void testPosePlaybackControllerWithWarmupPacket() throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      int numberOfPoses = 5;
      double trajectoryTime = 1.0;
      ValkyrieRobotModel valkyrieRobotModel = new ValkyrieRobotModel(false, false);
      FullRobotModel fullRobotModel = valkyrieRobotModel.createFullRobotModel();
      PosePlaybackPacket posePlaybackPacket = new ValkyrieWarmupPoseSequencePacket("PoseSequences/valkerena.poseSequence", fullRobotModel, 1.0);

      SimulationConstructionSet scs = startPosePlaybackSim(posePlaybackPacket);
      blockingSimulationRunner = new BlockingSimulationRunner(scs, 1000.0);

      scs.getVariable("PosePlayback_startPosePlayback").setValueFromDouble(1.0);
      blockingSimulationRunner.simulateAndBlock(numberOfPoses * trajectoryTime + 0.5);

      createMovie(scs);
      BambooTools.reportTestFinishedMessage();
   }

   // FlakyUnitTest: Hung on https://bamboo.ihmc.us/browse/RC-ALLGRADLE-VAL-952

	@EstimatedDuration(duration = 25.8)
	@Test(timeout = 128797)
   public void testPosePlaybackControllerWithRandomPoses() throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      int numberOfPoses = 5;
      double delayTime = 0.25;
      double trajectoryTime = 1.0;
      ValkyrieRobotModel valkyrieRobotModel = new ValkyrieRobotModel(false, false);
      FullRobotModel fullRobotModel = valkyrieRobotModel.createFullRobotModel();
      List<OneDoFJoint> jointToControl = Arrays.asList(fullRobotModel.getOneDoFJoints());
      PosePlaybackPacket posePlaybackPacket = createRandomPosePlaybackPacket(fullRobotModel, jointToControl, numberOfPoses, delayTime, trajectoryTime);

      SimulationConstructionSet scs = startPosePlaybackSim(posePlaybackPacket);
      blockingSimulationRunner = new BlockingSimulationRunner(scs, 1000.0);

      scs.getVariable("PosePlayback_startPosePlayback").setValueFromDouble(1.0);
      blockingSimulationRunner.simulateAndBlock(numberOfPoses * trajectoryTime + 0.5);

      createMovie(scs);
      BambooTools.reportTestFinishedMessage();
   }

	@EstimatedDuration(duration = 23.7)
	@Test(timeout = 118283)
   public void testPosePlaybackControllerWithRandomPosesWithSomeJointsUncontrolled() throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      int numberOfPoses = 5;
      double delayTime = 0.25;
      double trajectoryTime = 1.0;
      ValkyrieRobotModel valkyrieRobotModel = new ValkyrieRobotModel(false, false);
      FullRobotModel fullRobotModel = valkyrieRobotModel.createFullRobotModel();
      ArrayList<OneDoFJoint> jointToControl = new ArrayList<>(Arrays.asList(fullRobotModel.getOneDoFJoints()));
      int numberOfUncontrolledJoints = RandomTools.generateRandomInt(random, 2, jointToControl.size() / 2);
      for (int i = 0; i < numberOfUncontrolledJoints; i++)
      {
         jointToControl.remove(RandomTools.generateRandomInt(random, 1, jointToControl.size()) - 1);
      }
      PosePlaybackPacket posePlaybackPacket = createRandomPosePlaybackPacket(fullRobotModel, jointToControl, numberOfPoses, delayTime, trajectoryTime);

      SimulationConstructionSet scs = startPosePlaybackSim(posePlaybackPacket);
      blockingSimulationRunner = new BlockingSimulationRunner(scs, 1000.0);

      scs.getVariable("PosePlayback_startPosePlayback").setValueFromDouble(1.0);
      blockingSimulationRunner.simulateAndBlock(numberOfPoses * trajectoryTime + 0.5);

      createMovie(scs);
      BambooTools.reportTestFinishedMessage();
   }

   private SimulationConstructionSet startPosePlaybackSim(PosePlaybackPacket posePlaybackPacket)
   {

      DRCGuiInitialSetup guiInitialSetup = new DRCGuiInitialSetup(true, false);
      guiInitialSetup.setCreateGUI(simulationTestingParameters.getCreateGUI());
      double floatingHeight = 0.3;
      double groundHeight = 0.0;
      double initialYaw = 0.0;
      ValkyrieRobotModel valkyrieRobotModel = new ValkyrieRobotModel(false, false);
      DRCRobotInitialSetup<SDFRobot> robotInitialSetup = valkyrieRobotModel.getDefaultRobotInitialSetup(groundHeight + floatingHeight, initialYaw);
      GroundProfile3D groundProfile = new FlatGroundProfile(groundHeight);
      DRCSCSInitialSetup scsInitialSetup = new DRCSCSInitialSetup(groundProfile, valkyrieRobotModel.getSimulateDT());
      scsInitialSetup.setDrawGroundProfile(true);
      scsInitialSetup.setInitializeEstimatorToActual(true);
      scsInitialSetup.setSimulationDataBufferSize(16000);
      scsInitialSetup.setTimePerRecordTick(valkyrieRobotModel.getControllerDT());

      DRCPosePlaybackDemo drcPosePlaybackDemo = new DRCPosePlaybackDemo(robotInitialSetup, guiInitialSetup, scsInitialSetup, posePlaybackPacket,
            valkyrieRobotModel);
      drcSimulation = drcPosePlaybackDemo.getDRCSimulation();
      return drcPosePlaybackDemo.getSimulationConstructionSet();
   }

   private void createMovie(SimulationConstructionSet scs)
   {
      if (simulationTestingParameters.getCreateSCSMovies())
      {
         BambooTools.createMovieAndDataWithDateTimeClassMethodAndShareOnSharedDriveIfAvailable(
               BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.VALKYRIE), scs, 1);
      }
   }

   private PosePlaybackPacket createRandomPosePlaybackPacket(final FullRobotModel fullRobotModel, final List<OneDoFJoint> jointToControl,
         final int numberOfPoses, final double delayBeforePoses, final double trajectoryTime)
   {
      PosePlaybackPacket posePlaybackPacket = new PosePlaybackPacket()
      {
         private final PlaybackPoseSequence playbackPoseSequence = new PlaybackPoseSequence(fullRobotModel);

         private final LinkedHashMap<OneDoFJoint, Double> jointKps = new LinkedHashMap<>(numberOfPoses);
         private final LinkedHashMap<OneDoFJoint, Double> jointKds = new LinkedHashMap<>(numberOfPoses);
         {
            for (int i = 0; i < numberOfPoses; i++)
            {
               LinkedHashMap<OneDoFJoint, Double> pose = new LinkedHashMap<>();
               for (OneDoFJoint joint : jointToControl)
               {
                  pose.put(joint, RandomTools.generateRandomDouble(random, joint.getJointLimitLower(), joint.getJointLimitUpper()));
               }

               PlaybackPose playbackPose = new PlaybackPose(pose);
               playbackPose.setPlayBackDuration(trajectoryTime);
               playbackPose.setPlaybackDelayBeforePose(delayBeforePoses);

               playbackPoseSequence.addPose(playbackPose);

               for (OneDoFJoint joint : jointToControl)
               {
                  double mass = TotalMassCalculator.computeSubTreeMass(joint.getSuccessor());
                  jointKps.put(joint, 4.0 * mass);
                  jointKds.put(joint, GainCalculator.computeDampingForSecondOrderSystem(mass, jointKps.get(joint), 0.05));
               }
            }
         }

         @Override
         public Map<OneDoFJoint, Double> getJointKps()
         {
            return jointKps;
         }

         @Override
         public Map<OneDoFJoint, Double> getJointKds()
         {
            return jointKds;
         }

         @Override
         public PlaybackPoseSequence getPlaybackPoseSequence()
         {
            return playbackPoseSequence;
         }

         @Override
         public double getInitialGainScaling()
         {
            return 1.0;
         }
      };
      return posePlaybackPacket;
   }
}
