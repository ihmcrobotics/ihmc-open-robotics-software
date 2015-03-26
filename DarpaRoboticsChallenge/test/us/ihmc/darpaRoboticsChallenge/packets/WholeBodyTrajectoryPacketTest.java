package us.ihmc.darpaRoboticsChallenge.packets;

import static org.junit.Assert.assertEquals;

import java.util.ArrayList;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.communication.packets.wholebody.WholeBodyTrajectoryPacket;
import us.ihmc.communication.util.NetworkConfigParameters;
import us.ihmc.darpaRoboticsChallenge.DRCObstacleCourseStartingLocation;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.testTools.DRCSimulationTestHelper;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationRunner.ControllerFailureException;
import us.ihmc.utilities.MemoryTools;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.code.agileTesting.BambooAnnotations.EstimatedDuration;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.wholeBodyController.parameters.DefaultArmConfigurations.ArmConfigurations;
import us.ihmc.yoUtilities.time.GlobalTimer;

public abstract class WholeBodyTrajectoryPacketTest
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();
   private DRCSimulationTestHelper drcSimulationTestHelper;
   
   // allowed deviations on the simulated robot from the trajectory for position and velocity:
   private final static double epsilonQ = 0.05;
   private final static double epsilonQd = 0.1;

   static
   {
      simulationTestingParameters.setKeepSCSUp(false);
   }
   
   public abstract DRCRobotModel getRobotModel();
   public abstract String getSimpleRobotName();
   
   @EstimatedDuration(duration = 30.0)
   @Test(timeout = 90000)
   public void testArmPacket() throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      int waypoints = 2;
      int armJoints = getRobotModel().getDefaultArmConfigurations().getArmDefaultConfigurationJointAngles(ArmConfigurations.HOME, RobotSide.LEFT).length;
      
      WholeBodyTrajectoryPacket packet = createEmptyPacket(waypoints, armJoints);
      
      double[] leftArmHome = getRobotModel().getDefaultArmConfigurations().getArmDefaultConfigurationJointAngles(ArmConfigurations.HOME, RobotSide.LEFT);
      double[] rightArmHome = getRobotModel().getDefaultArmConfigurations().getArmDefaultConfigurationJointAngles(ArmConfigurations.HOME, RobotSide.RIGHT);
      
      packet.leftArmJointAngle = new double[armJoints][waypoints];
      packet.rightArmJointAngle = new double[armJoints][waypoints];
      
      for (int jointIdx = 0; jointIdx < armJoints; jointIdx++)
      {
         packet.leftArmJointAngle[jointIdx][0] = 0.0;
         packet.rightArmJointAngle[jointIdx][0] = 0.0;
         
         packet.leftArmJointAngle[jointIdx][1] = leftArmHome[jointIdx];
         packet.rightArmJointAngle[jointIdx][1] = rightArmHome[jointIdx];
      }
      
      packet.timeAtWaypoint[0] = 2.0;
      packet.timeAtWaypoint[1] = 4.0;
      
      executePacket(packet);
   }
   
   private void executePacket(WholeBodyTrajectoryPacket packet) throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      drcSimulationTestHelper = new DRCSimulationTestHelper("DRCWholeBodyTrajectoryPacketTest", null, DRCObstacleCourseStartingLocation.DEFAULT, simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.simulateAndBlock(1.0);
      drcSimulationTestHelper.sendWholeBodyTrajectoryPacketToListeners(packet);
      
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
         
         SDFFullRobotModel fullRobotModel = drcSimulationTestHelper.getSDFFullRobotModel();
         SDFRobot sdfRobot = drcSimulationTestHelper.getRobot();
         
         // check if both arms reached target
         for (RobotSide robotSide : RobotSide.values)
         {
            ArrayList<OneDoFJoint> armJoints = fullRobotModel.armJointIDsList.get(robotSide);
            double[][] desiredQ;
            double[][] desiredQd;
            if (robotSide == RobotSide.LEFT)
            {
               desiredQ = packet.leftArmJointAngle;
               desiredQd = packet.leftArmJointVelocity;
            }
            else
            {
               desiredQ = packet.rightArmJointAngle;
               desiredQd = packet.rightArmJointVelocity;
            }
            
            for (int jointIdx = 0; jointIdx < armJoints.size(); jointIdx++)
            {
               OneDegreeOfFreedomJoint joint = sdfRobot.getOneDegreeOfFreedomJoint(armJoints.get(jointIdx).getName());
               // if the desired position is null the waypoint is set to the actual position internally
               if (desiredQ != null)
               {
                  assertEquals(joint.getQ().getDoubleValue(), desiredQ[jointIdx][i], epsilonQ);
               }
               
               // if the desired velocity is null the waypoint is set to velocity zero internally
               if (desiredQd != null)
               {
                  assertEquals(joint.getQD().getDoubleValue(), desiredQd[jointIdx][i], epsilonQd);
               }
               else
               {
                  assertEquals(joint.getQD().getDoubleValue(), 0.0, epsilonQd);
               }
            }
         }
         
         // check if pelvis reached target
         
         
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
      ret.leftArmJointAngle = null;
      ret.rightArmJointAngle = null;
      ret.leftArmJointVelocity = null;
      ret.rightArmJointVelocity = null;
      
      return ret;
   }
   
   @Before
   public void setUp() throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      if (NetworkConfigParameters.USE_BEHAVIORS_MODULE)
      {
         throw new RuntimeException("Must set NetworkConfigParameters.USE_BEHAVIORS_MODULE = false in order to perform this test!");
      }

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
      BambooTools.reportTestStartedMessage();
   }

   @After
   public void destroySimulationAndRecycleMemory()
   {
      BambooTools.reportTestFinishedMessage();
      
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

      GlobalTimer.clearTimers();
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }
}
