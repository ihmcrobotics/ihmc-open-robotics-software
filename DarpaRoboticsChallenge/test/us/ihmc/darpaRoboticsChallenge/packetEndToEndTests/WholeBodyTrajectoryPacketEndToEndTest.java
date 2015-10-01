package us.ihmc.darpaRoboticsChallenge.packetEndToEndTests;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.Random;

import javax.vecmath.Point3d;
import javax.vecmath.Tuple3d;
import javax.vecmath.Vector3d;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.SdfLoader.SDFFullHumanoidRobotModel;
import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.darpaRoboticsChallenge.DRCObstacleCourseStartingLocation;
import us.ihmc.darpaRoboticsChallenge.MultiRobotTestInterface;
import us.ihmc.darpaRoboticsChallenge.testTools.DRCSimulationTestHelper;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.ArmJointTrajectoryPacket;
import us.ihmc.humanoidRobotics.communication.packets.wholebody.WholeBodyTrajectoryPacket;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.time.GlobalTimer;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationRunner.ControllerFailureException;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;
import us.ihmc.tools.thread.ThreadTools;
import us.ihmc.wholeBodyController.parameters.DefaultArmConfigurations.ArmConfigurations;

public abstract class WholeBodyTrajectoryPacketEndToEndTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();
   private DRCSimulationTestHelper drcSimulationTestHelper;
   
   // allowed deviations on the simulated robot from the trajectory for position and velocity:
   private final static double epsilonQ = 0.05;
   private final static double epsilonQd = 0.1;
   
   private final long seed = 126497;
   Random random = new Random(seed);

   static
   {
      simulationTestingParameters.setKeepSCSUp(false);
   }
   
	@DeployableTestMethod(estimatedDuration = 45.7)
   @Test(timeout = 230000)
   public void testArmPacket() throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      int waypoints = 2;
      int armJoints = getRobotModel().getDefaultArmConfigurations().getArmDefaultConfigurationJointAngles(ArmConfigurations.HOME, RobotSide.LEFT).length;
      
      WholeBodyTrajectoryPacket packet = createEmptyPacket(waypoints, armJoints);
      packet.allocateArmTrajectories();
      
      double[] leftArmHome = getRobotModel().getDefaultArmConfigurations().getArmDefaultConfigurationJointAngles(ArmConfigurations.HOME, RobotSide.LEFT);
      double[] rightArmHome = getRobotModel().getDefaultArmConfigurations().getArmDefaultConfigurationJointAngles(ArmConfigurations.HOME, RobotSide.RIGHT);
      
      // this test assumes the same number of waypoints for both arms.
      for (int jointIdx = 0; jointIdx < armJoints; jointIdx++)
      {
         packet.leftArmTrajectory.trajectoryPoints[0].positions[jointIdx] = 0.0;
         packet.rightArmTrajectory.trajectoryPoints[0].positions[jointIdx] = 0.0;
         
         packet.leftArmTrajectory.trajectoryPoints[1].positions[jointIdx] = leftArmHome[jointIdx];
         packet.rightArmTrajectory.trajectoryPoints[1].positions[jointIdx] = rightArmHome[jointIdx];
      }
      
      packet.timeAtWaypoint[0] = 3.0;
      packet.timeAtWaypoint[1] = 6.0;
      
      for (int i = 0; i < waypoints; i++)
      {
         packet.rightArmTrajectory.trajectoryPoints[i].time = packet.timeAtWaypoint[i];
         packet.leftArmTrajectory.trajectoryPoints[i].time = packet.timeAtWaypoint[i];
      }
      
      executePacket(packet);
   }
   
	@DeployableTestMethod(estimatedDuration = 27.3)
   @Test(timeout = 140000)
   public void testPelvisHeightPacket() throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      // currently the pelvis does not support multiple waypoints:
      int waypoints = 1;
      WholeBodyTrajectoryPacket packet = createEmptyPacket(waypoints, 0);
      
      packet.allocatePelvisTrajectory();
      Vector3d actualPelvis = drcSimulationTestHelper.getRobot().getPositionInWorld();
      
      packet.pelvisWorldPosition[0] = new Point3d(actualPelvis);
      packet.pelvisWorldPosition[0].add(new Point3d(0.0, 0.0, 0.1));
      
      packet.timeAtWaypoint[0] = 1.0;
      
      executePacket(packet);
   }
   
   private void executePacket(WholeBodyTrajectoryPacket packet) throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      drcSimulationTestHelper.send(packet);
      
      int waypoints = packet.numWaypoints; 
      for (int i = 0; i < waypoints; i++)
      {
         System.out.println("Starting execution of waypoint " + (i+1) + "/" + waypoints + "...");
         
         double startWaypointTime;
         double endWaypointTime = packet.timeAtWaypoint[i];
         if (i == 0)
         {
            startWaypointTime = 0.0;
         }
         else
         {
            startWaypointTime = packet.timeAtWaypoint[i-1];
         }
         
         drcSimulationTestHelper.simulateAndBlock(endWaypointTime - startWaypointTime);
         System.out.println("done - check if target reached");
         
         SDFFullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getSDFFullRobotModel();
         SDFRobot sdfRobot = drcSimulationTestHelper.getRobot();
         
         // check if both arms reached target
         for (RobotSide robotSide : RobotSide.values)
         {
            ArmJointTrajectoryPacket armJointPacket = null;
            if (robotSide.equals(RobotSide.LEFT))
            {
               armJointPacket = packet.leftArmTrajectory;
            }
            if (robotSide.equals(RobotSide.RIGHT))
            {
               armJointPacket = packet.rightArmTrajectory;
            }
            
            if (armJointPacket == null)
            {
               continue;
            }
            
            ArrayList<OneDoFJoint> armJoints = fullRobotModel.getArmJointIDs(robotSide);
            for (int jointIdx = 0; jointIdx < armJoints.size(); jointIdx++)
            {
               OneDegreeOfFreedomJoint joint = sdfRobot.getOneDegreeOfFreedomJoint(armJoints.get(jointIdx).getName());

               assertEquals(armJointPacket.trajectoryPoints[i].positions[jointIdx], joint.getQ().getDoubleValue(), epsilonQ);
               assertEquals(armJointPacket.trajectoryPoints[i].velocities[jointIdx], joint.getQD().getDoubleValue(), epsilonQd);
            }
         }
         
         // check if pelvis reached target
         if (packet.pelvisWorldPosition != null)
         {
            Tuple3d actualPelvis = sdfRobot.getPositionInWorld();
            Tuple3d desiredPelvis = packet.pelvisWorldPosition[i];
            
            assertTrue(actualPelvis.epsilonEquals(desiredPelvis, epsilonQ));
         }
         
         // check if chest reached target
         
         System.out.println("success");
      }
      
      drcSimulationTestHelper.simulateAndBlock(1.0);
   }
   
   private WholeBodyTrajectoryPacket createEmptyPacket(int waypoints, int armJoints)
   {
      WholeBodyTrajectoryPacket ret = new WholeBodyTrajectoryPacket(waypoints, armJoints);
      ret.pelvisWorldPosition = null;
      ret.pelvisLinearVelocity = null;
      ret.pelvisAngularVelocity = null;
      ret.pelvisWorldOrientation = null;
      ret.chestWorldOrientation = null;
      ret.chestAngularVelocity = null;
      ret.leftArmTrajectory = null;
      ret.rightArmTrajectory = null;
      
      return ret;
   }
   
   @Before
   public void setUp() throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
      BambooTools.reportTestStartedMessage();
      
      drcSimulationTestHelper = new DRCSimulationTestHelper("DRCWholeBodyTrajectoryPacketTest", null, DRCObstacleCourseStartingLocation.DEFAULT, simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.simulateAndBlock(1.0);
   }

   @After
   public void destroySimulationAndRecycleMemory()
   {
      BambooTools.reportTestFinishedMessage();
      
      if (simulationTestingParameters.getKeepSCSUp())
      {
         ThreadTools.sleepForever();
      }

      if (drcSimulationTestHelper != null)
      {
         drcSimulationTestHelper.destroySimulation();
         drcSimulationTestHelper = null;
      }

      GlobalTimer.clearTimers();
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }
}
