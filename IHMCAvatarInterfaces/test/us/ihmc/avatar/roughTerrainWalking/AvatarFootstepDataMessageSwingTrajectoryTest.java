package us.ihmc.avatar.roughTerrainWalking;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.After;
import org.junit.Before;

import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.DRCStartingLocation;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.controllerAPI.EndToEndHandTrajectoryMessageTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule;
import us.ihmc.commonWalkingControlModules.desiredFootStep.FootstepListVisualizer;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearanceRGBColor;
import us.ihmc.humanoidRobotics.communication.packets.ExecutionTiming;
import us.ihmc.humanoidRobotics.communication.packets.SE3TrajectoryPointMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.trajectories.TrajectoryType;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.thread.ThreadTools;

public abstract class AvatarFootstepDataMessageSwingTrajectoryTest implements MultiRobotTestInterface
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();
   private DRCSimulationTestHelper drcSimulationTestHelper;

   public void testSwingTrajectoryInWorld() throws SimulationExceededMaximumTimeException
   {
      String className = getClass().getSimpleName();
      FlatGroundEnvironment environment = new FlatGroundEnvironment();
      DRCStartingLocation startingLocation = DRCObstacleCourseStartingLocation.DEFAULT_BUT_ALMOST_PI;
      DRCRobotModel robotModel = getRobotModel();
      drcSimulationTestHelper = new DRCSimulationTestHelper(environment, className, startingLocation, simulationTestingParameters, robotModel);
      drcSimulationTestHelper.getSimulationConstructionSet().setCameraPosition(0.0, -3.0, 1.0);
      drcSimulationTestHelper.getSimulationConstructionSet().setCameraFix(0.0, 0.0, 0.2);
      ThreadTools.sleep(1000);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5));
      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();
      
      double swingTime = robotModel.getWalkingControllerParameters().getDefaultSwingTime() * 3.0;
      double transferTime = robotModel.getWalkingControllerParameters().getDefaultTransferTime();
      double initialTransferTime = robotModel.getWalkingControllerParameters().getDefaultInitialTransferTime();
      FootstepDataListMessage footstepDataList = new FootstepDataListMessage(swingTime, transferTime);
      footstepDataList.setExecutionTiming(ExecutionTiming.CONTROL_ABSOLUTE_TIMINGS);
      
      // step in place but do some fancy foot motion
      RobotSide robotSide = RobotSide.LEFT;
      ReferenceFrame soleFrame = drcSimulationTestHelper.getControllerFullRobotModel().getSoleFrame(robotSide);
      FramePoint footPosition = new FramePoint(soleFrame);
      FrameOrientation footOrientation = new FrameOrientation(soleFrame);
      
      FootstepDataMessage footstep = new FootstepDataMessage();
      footstep.setRobotSide(robotSide);
      footstep.setTrajectoryType(TrajectoryType.WAYPOINTS);
      footstep.setTrajectoryReferenceFrameId(worldFrame);
      footstep.setTimings(swingTime, initialTransferTime);

      double radius = 0.075;
      double pitch = Math.toRadians(10.0);
      FramePoint circleCenter = new FramePoint(soleFrame, 0.0, 0.0, 0.125);
      int points = 8;
      double timeBetweenWaypoints = swingTime / (points + 1);
      
      SE3TrajectoryPointMessage[] waypoints = new SE3TrajectoryPointMessage[points];
      for (int i = 0; i < points; i++)
      {
         double percentInCircle = i / (points - 1.0);
         double percentInSwing = (i + 1.0) / (points + 1.0);
         double angleInCircle = 2.0 * Math.PI * percentInCircle;
         
         double xOffset = Math.sin(angleInCircle) * radius;
         double zOffset = -Math.cos(angleInCircle) * radius;
         FramePoint waypointPosition = new FramePoint(circleCenter);
         waypointPosition.add(xOffset, 0.0, zOffset);
         waypointPosition.changeFrame(worldFrame);
         
         FrameVector waypointLinearVelocity = new FrameVector(soleFrame);
         if (i > 0 && i < points - 1)
         {
            double scale = 1.0 / timeBetweenWaypoints;
            double xVelocity = scale * Math.cos(angleInCircle) * radius;
            double zVelocity = scale * Math.sin(angleInCircle) * radius;
            waypointLinearVelocity.set(xVelocity, 0.0, zVelocity);
         }
         waypointLinearVelocity.changeFrame(worldFrame);
         
         FrameOrientation waypointOrientation = new FrameOrientation(soleFrame);
         if (i % 2 == 0)
         {
            waypointOrientation.setYawPitchRoll(0.0, pitch, 0.0);
         }
         else
         {
            waypointOrientation.setYawPitchRoll(0.0, -pitch, 0.0);
         }
         waypointOrientation.changeFrame(worldFrame);
         
         SE3TrajectoryPointMessage waypoint = new SE3TrajectoryPointMessage();
         waypoint.setTime(percentInSwing * swingTime);
         waypoint.setPosition(waypointPosition.getPoint());
         waypoint.setLinearVelocity(waypointLinearVelocity.getVector());
         waypoint.setOrientation(waypointOrientation.getQuaternion());
         
         Graphics3DObject sphere = new Graphics3DObject();
         sphere.translate(waypointPosition.getPoint());
         sphere.addSphere(0.01, new YoAppearanceRGBColor(FootstepListVisualizer.defaultFeetColors.get(robotSide), 0.0));
         scs.addStaticLinkGraphics(sphere);
         
         waypoints[i] = waypoint;
      }

      footPosition.changeFrame(worldFrame);
      footOrientation.changeFrame(worldFrame);
      footstep.setLocation(footPosition.getPoint());
      footstep.setOrientation(footOrientation.getQuaternion());
      footstep.setSwingTrajectory(waypoints);
      
      footstepDataList.add(footstep);
      drcSimulationTestHelper.send(footstepDataList);
      
      String format = EuclidCoreIOTools.getStringFormat(6, 4);
      String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
      String swingStateNamespace = sidePrefix + FootControlModule.class.getSimpleName();
      String namePrefix = sidePrefix + "Foot";
      String typeName = namePrefix + TrajectoryType.class.getSimpleName();
      String posName = namePrefix + "DesiredSolePositionInWorld";
      String oriName = namePrefix + "DesiredSoleOrientationInWorld";
      String linName = namePrefix + "DesiredSoleLinearVelocityInWorld";
      String angName = namePrefix + "DesiredSoleAngularVelocityInWorld";
      
      String currentWaypointVariable = namePrefix + "CurrentTrajectoryWaypoint";
      YoVariable<?> currentWaypointIndex = scs.getVariable(swingStateNamespace, currentWaypointVariable);
      
      double epsilon = 1.0E-6;
      
      currentWaypointIndex.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void variableChanged(YoVariable<?> v)
         {
            // this makes sure the asserts are not triggered when scs is in playback mode
            if (!scs.isSimulating())
            {
               return;
            }
            
            int waypointIndex = (int) v.getValueAsLongBits() - 1;
            
            if (waypointIndex < 0 || waypointIndex >= points)
               return;

            SE3TrajectoryPointMessage expectedDesired = waypoints[waypointIndex];
            
            @SuppressWarnings("unchecked")
            EnumYoVariable<TrajectoryType> currentTrajectoryType = (EnumYoVariable<TrajectoryType>) scs.getVariable(swingStateNamespace, typeName);
            Point3D currentDesiredPosition = EndToEndHandTrajectoryMessageTest.findPoint3d(swingStateNamespace, posName, scs);
            Quaternion currentDesiredOrientation = EndToEndHandTrajectoryMessageTest.findQuat4d(swingStateNamespace, oriName, scs);
            Vector3D currentDesiredLinearVelocity = EndToEndHandTrajectoryMessageTest.findVector3d(swingStateNamespace, linName, scs);
            Vector3D currentDesiredAngularVelocity = EndToEndHandTrajectoryMessageTest.findVector3d(swingStateNamespace, angName, scs);
            
            assertEquals("Unexpected Trajectory Type", TrajectoryType.WAYPOINTS, currentTrajectoryType.getEnumValue());
            EuclidCoreTestTools.assertTuple3DEquals("Position", expectedDesired.position, currentDesiredPosition, epsilon, format);
            EuclidCoreTestTools.assertQuaternionEqualsSmart("Orientation", expectedDesired.orientation, currentDesiredOrientation, epsilon, format);
            EuclidCoreTestTools.assertTuple3DEquals("LinearVelocity", expectedDesired.linearVelocity, currentDesiredLinearVelocity, epsilon, format);
            EuclidCoreTestTools.assertTuple3DEquals("AngularVelocity", expectedDesired.angularVelocity, currentDesiredAngularVelocity, epsilon, format);
         }
      });
      
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(initialTransferTime + swingTime + transferTime));
      assertEquals("Swing Trajectory did not execute.", points, currentWaypointIndex.getValueAsLongBits());
   }
   
   @Before
   public void showMemoryUsageBeforeTest()
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
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

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
      simulationTestingParameters = null;
   }
}
