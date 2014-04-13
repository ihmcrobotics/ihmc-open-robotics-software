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
import us.ihmc.bambooTools.BambooTools;
import us.ihmc.commonWalkingControlModules.automaticSimulationRunner.AutomaticSimulationRunner;
import us.ihmc.commonWalkingControlModules.dynamics.FullRobotModel;
import us.ihmc.commonWalkingControlModules.posePlayback.PlaybackPose;
import us.ihmc.commonWalkingControlModules.posePlayback.PlaybackPoseSequence;
import us.ihmc.commonWalkingControlModules.posePlayback.PosePlaybackPacket;
import us.ihmc.commonWalkingControlModules.visualizer.RobotVisualizer;
import us.ihmc.darpaRoboticsChallenge.DRCConfigParameters;
import us.ihmc.darpaRoboticsChallenge.DRCController;
import us.ihmc.darpaRoboticsChallenge.DRCGuiInitialSetup;
import us.ihmc.darpaRoboticsChallenge.DRCPosePlaybackDemo;
import us.ihmc.darpaRoboticsChallenge.DRCRobotInterface;
import us.ihmc.darpaRoboticsChallenge.DRCSCSInitialSetup;
import us.ihmc.darpaRoboticsChallenge.drcRobot.PlainDRCRobot;
import us.ihmc.darpaRoboticsChallenge.initialSetup.DRCRobotInitialSetup;
import us.ihmc.graphics3DAdapter.GroundProfile;
import us.ihmc.utilities.AsyncContinuousExecutor;
import us.ihmc.utilities.MemoryTools;
import us.ihmc.utilities.RandomTools;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.TimerTaskScheduler;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.utilities.screwTheory.TotalMassCalculator;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.posePlayback.ValkyrieWarmupPoseSequencePacket;

import com.yobotics.simulationconstructionset.SimulationConstructionSet;
import com.yobotics.simulationconstructionset.time.GlobalTimer;
import com.yobotics.simulationconstructionset.util.FlatGroundProfile;
import com.yobotics.simulationconstructionset.util.GainCalculator;
import com.yobotics.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import com.yobotics.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

public class ValkyriePosePlaybackDemoTest
{
   private static final boolean ALWAYS_SHOW_GUI = false;
   private static final boolean KEEP_SCS_UP = false;

   private static final boolean CREATE_MOVIE = BambooTools.doMovieCreation();
   private static final boolean checkNothingChanged = BambooTools.getCheckNothingChanged();

   private static final boolean SHOW_GUI = ALWAYS_SHOW_GUI || checkNothingChanged || CREATE_MOVIE;

   private BlockingSimulationRunner blockingSimulationRunner;
   private DRCController drcController;
   private RobotVisualizer robotVisualizer;
   private final Random random = new Random(6519651L);
   
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
      if (blockingSimulationRunner != null)
      {
         blockingSimulationRunner.destroySimulation();
         blockingSimulationRunner = null;
      }

      if (drcController != null)
      {
         drcController.dispose();
         drcController = null;
      }

      if (robotVisualizer != null)
      {
         robotVisualizer.close();
         robotVisualizer = null;
      }

      GlobalTimer.clearTimers();
      TimerTaskScheduler.cancelAndReset();
      AsyncContinuousExecutor.cancelAndReset();
      
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }


   @Test
   public void testPosePlaybackControllerWithWarmupPacket() throws SimulationExceededMaximumTimeException
   {
      DRCPosePlaybackDemo drcPosePlaybackDemo = setupPosePlaybackSim();
      
      drcController = drcPosePlaybackDemo.getDRCController();
      SimulationConstructionSet scs = drcPosePlaybackDemo.getSimulationConstructionSet();
      
      int numberOfPoses = 5;
      double trajectoryTime = 1.0;
      FullRobotModel fullRobotModel = drcPosePlaybackDemo.getControllerModel();
      List<OneDoFJoint> jointToControl = Arrays.asList(fullRobotModel.getOneDoFJoints());
//      PosePlaybackPacket posePlaybackPacket = new ValkyrieWarmupPoseSequencePacket(fullRobotModel, 1.0);
//      PosePlaybackPacket posePlaybackPacket = new ValkyrieWarmupPoseSequencePacket("valkercise02.poseSequence", fullRobotModel, 1.0);
      PosePlaybackPacket posePlaybackPacket = new ValkyrieWarmupPoseSequencePacket("valkerena.poseSequence", fullRobotModel, 1.0);
      
      drcPosePlaybackDemo.setupPosePlaybackController(posePlaybackPacket, true);

      blockingSimulationRunner = new BlockingSimulationRunner(scs, 1000.0);

      scs.getVariable("PosePlayback_startPosePlayback").setValueFromDouble(1.0);
      blockingSimulationRunner.simulateAndBlock(numberOfPoses * trajectoryTime + 0.5);

      createMovie(scs);
      BambooTools.reportTestFinishedMessage();
   }

   @Test
   public void testPosePlaybackControllerWithRandomPoses() throws SimulationExceededMaximumTimeException
   {
      DRCPosePlaybackDemo drcPosePlaybackDemo = setupPosePlaybackSim();
      
      drcController = drcPosePlaybackDemo.getDRCController();
      SimulationConstructionSet scs = drcPosePlaybackDemo.getSimulationConstructionSet();
      
      int numberOfPoses = 5;
      double delayTime = 0.25;
      double trajectoryTime = 1.0;
      FullRobotModel fullRobotModel = drcPosePlaybackDemo.getControllerModel();
      List<OneDoFJoint> jointToControl = Arrays.asList(fullRobotModel.getOneDoFJoints());
      PosePlaybackPacket posePlaybackPacket = createRandomPosePlaybackPacket(fullRobotModel, jointToControl, numberOfPoses, delayTime, trajectoryTime);
      drcPosePlaybackDemo.setupPosePlaybackController(posePlaybackPacket, true);

      blockingSimulationRunner = new BlockingSimulationRunner(scs, 1000.0);

      scs.getVariable("PosePlayback_startPosePlayback").setValueFromDouble(1.0);
      blockingSimulationRunner.simulateAndBlock(numberOfPoses * trajectoryTime + 0.5);

      createMovie(scs);
      BambooTools.reportTestFinishedMessage();
   }

   @Test
   public void testPosePlaybackControllerWithRandomPosesWithSomeJointsUncontrolled() throws SimulationExceededMaximumTimeException
   {
      DRCPosePlaybackDemo drcPosePlaybackDemo = setupPosePlaybackSim();
      
      drcController = drcPosePlaybackDemo.getDRCController();
      SimulationConstructionSet scs = drcPosePlaybackDemo.getSimulationConstructionSet();
      
      int numberOfPoses = 5;
      double delayTime = 0.25;
      double trajectoryTime = 1.0;
      FullRobotModel fullRobotModel = drcPosePlaybackDemo.getControllerModel();
      ArrayList<OneDoFJoint> jointToControl = new ArrayList<>(Arrays.asList(fullRobotModel.getOneDoFJoints()));
      int numberOfUncontrolledJoints = RandomTools.generateRandomInt(random, 2, jointToControl.size() / 2);
      for (int i = 0; i < numberOfUncontrolledJoints; i++)
      {
         jointToControl.remove(RandomTools.generateRandomInt(random, 1, jointToControl.size()) - 1);
      }
      PosePlaybackPacket posePlaybackPacket = createRandomPosePlaybackPacket(fullRobotModel, jointToControl, numberOfPoses, delayTime, trajectoryTime);
      drcPosePlaybackDemo.setupPosePlaybackController(posePlaybackPacket, true);

      blockingSimulationRunner = new BlockingSimulationRunner(scs, 1000.0);

      scs.getVariable("PosePlayback_startPosePlayback").setValueFromDouble(1.0);
      blockingSimulationRunner.simulateAndBlock(numberOfPoses * trajectoryTime + 0.5);

      createMovie(scs);
      BambooTools.reportTestFinishedMessage();
   }

   private DRCPosePlaybackDemo setupPosePlaybackSim()
   {
      ValkyrieRobotModel valkyrieRobotModel = new ValkyrieRobotModel();
      DRCRobotInterface valkyrieRobotInterface = new PlainDRCRobot(valkyrieRobotModel);

      AutomaticSimulationRunner automaticSimulationRunner = null;

      DRCGuiInitialSetup guiInitialSetup = new DRCGuiInitialSetup(true, false, null);
      double floatingHeight = 0.3;
      double groundHeight = 0.0;
      double initialYaw = 0.0;
      DRCRobotInitialSetup<SDFRobot> robotInitialSetup = valkyrieRobotModel.getDefaultRobotInitialSetup(groundHeight + floatingHeight, initialYaw);
      GroundProfile groundProfile = new FlatGroundProfile(groundHeight);
      DRCSCSInitialSetup scsInitialSetup = new DRCSCSInitialSetup(groundProfile, valkyrieRobotInterface.getSimulateDT());
      scsInitialSetup.setDrawGroundProfile(true);
      scsInitialSetup.setInitializeEstimatorToActual(true);
      
      DRCPosePlaybackDemo drcPosePlaybackDemo = new DRCPosePlaybackDemo(valkyrieRobotInterface, robotInitialSetup, guiInitialSetup, scsInitialSetup, automaticSimulationRunner, DRCConfigParameters.CONTROL_DT, 16000, valkyrieRobotModel);
      return drcPosePlaybackDemo;
   }

   private void createMovie(SimulationConstructionSet scs)
   {
      if (CREATE_MOVIE)
      {
         BambooTools.createMovieAndDataWithDateTimeClassMethodAndShareOnSharedDriveIfAvailable(scs, 1);
      }
   }

   private PosePlaybackPacket createRandomPosePlaybackPacket(final FullRobotModel fullRobotModel, final List<OneDoFJoint> jointToControl, final int numberOfPoses, final double delayBeforePoses, final double trajectoryTime)
   {
      PosePlaybackPacket posePlaybackPacket = new PosePlaybackPacket()
      {
         private final PlaybackPoseSequence playbackPoseSequence = new PlaybackPoseSequence(fullRobotModel);
         
         private final LinkedHashMap<OneDoFJoint, Double> jointKps = new LinkedHashMap<>(numberOfPoses);
         private final LinkedHashMap<OneDoFJoint, Double> jointKds = new LinkedHashMap<>(numberOfPoses);
         {
            for (int i = 0; i < numberOfPoses; i ++)
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
