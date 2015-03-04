package us.ihmc.darpaRoboticsChallenge.obstacleCourseTests;

import static org.junit.Assert.assertTrue;

import java.util.List;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import org.junit.After;
import org.junit.Before;
import org.junit.Ignore;
import org.junit.Test;

import us.ihmc.commonWalkingControlModules.controlModules.ChestOrientationControlModule;
import us.ihmc.communication.packets.walking.ChestOrientationPacket;
import us.ihmc.communication.packets.walking.FootstepData;
import us.ihmc.communication.packets.walking.FootstepDataList;
import us.ihmc.communication.packets.wholebody.WholeBodyTrajectoryPacket;
import us.ihmc.darpaRoboticsChallenge.DRCObstacleCourseStartingLocation;
import us.ihmc.darpaRoboticsChallenge.MultiRobotTestInterface;
import us.ihmc.darpaRoboticsChallenge.testTools.DRCSimulationTestHelper;
import us.ihmc.pathGeneration.footstepGenerator.TurnInPlaceFootstepGenerator;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.utilities.MemoryTools;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.code.agileTesting.BambooAnnotations.AverageDuration;
import us.ihmc.utilities.humanoidRobot.footstep.FootSpoof;
import us.ihmc.utilities.humanoidRobot.footstep.Footstep;
import us.ihmc.utilities.humanoidRobot.footstep.footsepGenerator.PathTypeStepParameters;
import us.ihmc.utilities.math.geometry.BoundingBox3d;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FrameOrientation2d;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.PoseReferenceFrame;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.ZUpFrame;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.robotSide.SideDependentList;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.yoUtilities.dataStructure.listener.VariableChangedListener;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.YoVariable;
import us.ihmc.yoUtilities.math.frames.YoFramePoint;
import us.ihmc.yoUtilities.math.frames.YoFrameQuaternion;
import us.ihmc.yoUtilities.math.frames.YoFrameVector;

public abstract class DRCObstacleCourseWholeBodyTrajectoryTests implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();
   
   private DRCSimulationTestHelper drcSimulationTestHelper;
   
   @Before
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test");
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

   @Ignore("Invoked manually to test memory & thread leaks")

   @AverageDuration(duration = 50.0)
   @Test(timeout=300000)
   public void testForMemoryLeaks() throws Exception
   {
      for (int i = 0; i < 10; i++)
      {
         showMemoryUsageBeforeTest();
         testStandingForACoupleSeconds();
         destroySimulationAndRecycleMemory();
      }
   }
   
   @AverageDuration(duration = 14.9)
   @Test(timeout = 54656)
   public void testStandingForACoupleSeconds() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;

      drcSimulationTestHelper = new DRCSimulationTestHelper("DRCStandingTest", "", selectedLocation, simulationTestingParameters, getRobotModel());

      setupCameraForWalkingUpToRamp();

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0);

      ThreadTools.sleep(2000);
      // drcSimulationTestHelper.createMovie(getSimpleRobotName(), simulationConstructionSet, 1);
      drcSimulationTestHelper.checkNothingChanged();

      assertTrue(success);

      Point3d center = new Point3d(-8.956281888358388E-4, -3.722237566790175E-7, 0.8882009563211146);
      Vector3d plusMinusVector = new Vector3d(0.2, 0.2, 0.5);
      BoundingBox3d boundingBox = BoundingBox3d.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);
      
      BambooTools.reportTestFinishedMessage();
   }
   
   private void setupCameraForWalkingUpToRamp()
   {
      Point3d cameraFix = new Point3d(1.8375, -0.16, 0.89);
      Point3d cameraPosition = new Point3d(1.10, 8.30, 1.37);

      drcSimulationTestHelper.setupCameraForUnitTest(cameraFix, cameraPosition);
   }
   
   public abstract WholeBodyTrajectoryPacket getRobotSpecificWholeBodyTrajectoryPacket(double trajectoryTime);
   
   @AverageDuration(duration = 38.0)
   @Test(timeout = 124050)
   public void testChestControlWithPackets() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;

      drcSimulationTestHelper = new DRCSimulationTestHelper("DRCStandingTest", "", selectedLocation, simulationTestingParameters, getRobotModel());

      setupCameraForWalkingUpToRamp();

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);
      
      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();

      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      final PoseReferenceFrame pelvisFrame = new PoseReferenceFrame("pelvisFrame", worldFrame);
      final ZUpFrame pelvisZUpFrame = new ZUpFrame(worldFrame, pelvisFrame, "pelvisZUpFrame");
      
      DoubleYoVariable q_x = (DoubleYoVariable) scs.getVariable("q_x");
      DoubleYoVariable q_y = (DoubleYoVariable) scs.getVariable("q_y");
      DoubleYoVariable q_z = (DoubleYoVariable) scs.getVariable("q_z");
      DoubleYoVariable q_qx = (DoubleYoVariable) scs.getVariable("q_qx");
      DoubleYoVariable q_qy = (DoubleYoVariable) scs.getVariable("q_qy");
      DoubleYoVariable q_qz = (DoubleYoVariable) scs.getVariable("q_qz");
      DoubleYoVariable q_qs = (DoubleYoVariable) scs.getVariable("q_qs");
      
      final YoFramePoint pelvisPosition = new YoFramePoint(q_x, q_y, q_z, worldFrame);
      final YoFrameQuaternion pelvisOrientation = new YoFrameQuaternion(q_qx, q_qy, q_qz, q_qs, worldFrame);
      
      VariableChangedListener pelvisFrameUpdater = new VariableChangedListener()
      {
         private final FramePoint localFramePoint = new FramePoint();
         private final FrameOrientation localFrameOrientation = new FrameOrientation();

         @Override
         public void variableChanged(YoVariable<?> v)
         {
            pelvisPosition.getFrameTupleIncludingFrame(localFramePoint);
            pelvisOrientation.getFrameOrientationIncludingFrame(localFrameOrientation);
            
            pelvisFrame.setPoseAndUpdate(localFramePoint, localFrameOrientation);
            pelvisZUpFrame.update();
         }
      };
      
      pelvisPosition.attachVariableChangedListener(pelvisFrameUpdater);
      pelvisOrientation.attachVariableChangedListener(pelvisFrameUpdater);
      
      FrameOrientation desiredChestFrameOrientation = new FrameOrientation();
      Quat4d desiredChestQuat = new Quat4d();


      DoubleYoVariable controllerDesiredInWorldQx = (DoubleYoVariable) scs.getVariable(ChestOrientationControlModule.class.getSimpleName(), "desiredChestInWorld" + "Qx");
      DoubleYoVariable controllerDesiredInWorldQy = (DoubleYoVariable) scs.getVariable(ChestOrientationControlModule.class.getSimpleName(), "desiredChestInWorld" + "Qy");
      DoubleYoVariable controllerDesiredInWorldQz = (DoubleYoVariable) scs.getVariable(ChestOrientationControlModule.class.getSimpleName(), "desiredChestInWorld" + "Qz");
      DoubleYoVariable controllerDesiredInWorldQs = (DoubleYoVariable) scs.getVariable(ChestOrientationControlModule.class.getSimpleName(), "desiredChestInWorld" + "Qs");
      YoFrameQuaternion controllerDesiredOrientationInWorld = new YoFrameQuaternion(controllerDesiredInWorldQx, controllerDesiredInWorldQy, controllerDesiredInWorldQz, controllerDesiredInWorldQs, worldFrame);
      FrameOrientation controllerChestDesiredFrameOrientation = new FrameOrientation();
      DoubleYoVariable chestAxisAngleErrorX = (DoubleYoVariable) scs.getVariable("chestElevatorAxisAngleOrientationController", "chestElevatorAxisAngleErrorInBody" + "X");
      DoubleYoVariable chestAxisAngleErrorY = (DoubleYoVariable) scs.getVariable("chestElevatorAxisAngleOrientationController", "chestElevatorAxisAngleErrorInBody" + "Y");
      DoubleYoVariable chestAxisAngleErrorZ = (DoubleYoVariable) scs.getVariable("chestElevatorAxisAngleOrientationController", "chestElevatorAxisAngleErrorInBody" + "Z");
      YoFrameVector chestAxisAngleError = new YoFrameVector(chestAxisAngleErrorX, chestAxisAngleErrorY, chestAxisAngleErrorZ, worldFrame);
      
      desiredChestFrameOrientation.setIncludingFrame(pelvisFrame, Math.toRadians(40.0), Math.toRadians(20.0), Math.toRadians(10.0));
      desiredChestFrameOrientation.changeFrame(worldFrame);
      desiredChestFrameOrientation.getQuaternion(desiredChestQuat);
      
      double trajectoryTime = 8;
//      ChestOrientationPacket packet = new ChestOrientationPacket(desiredChestQuat, false, trajectoryTime);
      
      
      WholeBodyTrajectoryPacket packet = getRobotSpecificWholeBodyTrajectoryPacket(trajectoryTime);
      
      //FIXME Davide currently has trajectory generation for orientations such that only the final destination is used. This might change later so fix it then.... -Will
      packet.chestWorldOrientation[packet.chestWorldOrientation.length - 1] = desiredChestQuat;
      
      drcSimulationTestHelper.sendWholeBodyTrajectoryPacketToListeners(packet);
      
      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0 + trajectoryTime );
      assertTrue(success);
      
      FootSpoof leftFootSpoof = new FootSpoof("leftFoot");
      FootSpoof rightFootSpoof = new FootSpoof("rightFoot");
      
      SideDependentList<RigidBody> feet = new SideDependentList<RigidBody>(leftFootSpoof.getRigidBody(), rightFootSpoof.getRigidBody());
      SideDependentList<ReferenceFrame> soleFrames = new SideDependentList<ReferenceFrame>(leftFootSpoof.getSoleFrame(), rightFootSpoof.getSoleFrame());
      
      FrameOrientation2d pathYaw = new FrameOrientation2d(worldFrame, Math.toRadians(-90.0));
      PathTypeStepParameters pathType = new PathTypeStepParameters()
      {
         @Override
         public double getTurningStepWidth()
         {
            return 0.3;
         }
         
         @Override
         public double getTurningOpenStepAngle()
         {
            return 0.4;
         }
         
         @Override
         public double getTurningCloseStepAngle()
         {
            return 0.4;
         }
         
         @Override
         public double getStepWidth()
         {
            return 0.3;
         }
         
         @Override
         public double getStepLength()
         {
            return 0.5;
         }
         
         @Override
         public double getAngle()
         {
            return 0.4;
         }
      };
      TurnInPlaceFootstepGenerator turnInPlaceFootstepGenerator = new TurnInPlaceFootstepGenerator(feet, soleFrames, pathYaw, pathType);
      List<Footstep> desiredFootstepList = turnInPlaceFootstepGenerator.generateDesiredFootstepList();
      double swingTime = 0.6;
      double transferTime = 0.25;
      FootstepDataList footstepDataList = new FootstepDataList(swingTime, transferTime);
      
      for (Footstep desiredFootstep : desiredFootstepList)
      {
         RobotSide robotSide = desiredFootstep.getRobotSide();
         Point3d location = new Point3d();
         Quat4d orientation = new Quat4d();
         desiredFootstep.getPose(location, orientation);
         location.setZ(0.0);
         FootstepData footstepData = new FootstepData(robotSide, location, orientation);
         footstepDataList.add(footstepData);
      }
      
      controllerDesiredOrientationInWorld.getFrameOrientationIncludingFrame(controllerChestDesiredFrameOrientation);

      assertTrue(
            "Desired chest orientation in controller does not match the desired chest orientation in the packet:\n Desired orientation from packet: "
                  + desiredChestFrameOrientation.toStringAsYawPitchRoll() + "\n Desired orientation from controller: "
                  + controllerChestDesiredFrameOrientation.toStringAsYawPitchRoll(),
            desiredChestFrameOrientation.epsilonEquals(controllerChestDesiredFrameOrientation, 1E-10));

      controllerChestDesiredFrameOrientation.changeFrame(pelvisZUpFrame);

      drcSimulationTestHelper.sendFootstepListToListeners(footstepDataList);

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(desiredFootstepList.size() * (swingTime + transferTime) + 5.0);
      assertTrue(success);

      assertTrue("The chest controller goes bad when turning", chestAxisAngleError.length() < 0.01);

      ThreadTools.sleep(2000);

      assertTrue(success);
      
      BambooTools.reportTestFinishedMessage();
   }

}
