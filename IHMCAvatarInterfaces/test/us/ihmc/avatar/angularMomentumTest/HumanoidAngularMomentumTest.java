package us.ihmc.avatar.angularMomentumTest;

import us.ihmc.avatar.DRCStartingLocation;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.initialSetup.OffsetAndYawRobotInitialSetup;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.robotController.SimpleRobotController;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.thread.ThreadTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public abstract class HumanoidAngularMomentumTest implements MultiRobotTestInterface
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();
   private OffsetAndYawRobotInitialSetup location = new OffsetAndYawRobotInitialSetup(new Vector3D(0.0, 0.0, 0.0), 0.0);
   private DRCRobotModel robotModel;
   private FullHumanoidRobotModel fullRobotModel;
   private DRCSimulationTestHelper drcSimulationTestHelper;

   public void walk() throws SimulationExceededMaximumTimeException
   {
      setupTest();
      AngularMomentumSpy angularMomentumSpy = new AngularMomentumSpy(drcSimulationTestHelper);
      double stepLength = getStepLength();
      double stepWidth = getStepWidth();
      RobotSide side = RobotSide.LEFT;
      FootstepDataListMessage footMessage = new FootstepDataListMessage();
      int numberOfFootstepsToTake = getNumberOfStepsToTake();
      Quaternion footOrientation = new Quaternion(0.0, 0.0, 0.0, 1.0);
      Point3D footLocation = new Point3D();
      double x = 0.0;
      for (int i = 0; i < numberOfFootstepsToTake; i++)
      {
         x += stepLength;
         footLocation.set(x, side.negateIfRightSide(stepWidth) / 2, 0.0);
         addFootstep(footLocation, footOrientation, side, footMessage);
         side = side.getOppositeSide();
      }
      footLocation.set(x, side.negateIfRightSide(stepWidth / 2), 0.0);
      addFootstep(footLocation, footOrientation, side, footMessage);
      double testTime = numberOfFootstepsToTake * 2.0 + 1.0;
      drcSimulationTestHelper.send(footMessage);
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(testTime);
      destroySimulationAndRecycleMemory();
   }

   private void setupTest()
   {
      robotModel = getRobotModel();
      FlatGroundEnvironment emptyEnvironment = new FlatGroundEnvironment();
      String className = getClass().getSimpleName();
      DRCStartingLocation startingLocation = new DRCStartingLocation()
      {
         @Override
         public OffsetAndYawRobotInitialSetup getStartingLocationOffset()
         {
            return location;
         }
      };
      simulationTestingParameters.setKeepSCSUp(true);
      drcSimulationTestHelper = new DRCSimulationTestHelper(emptyEnvironment, className, startingLocation, simulationTestingParameters, robotModel);
      Point3D cameraFix = new Point3D(0.0, 0.0, 1.0);
      Point3D cameraPosition = new Point3D(0.0, 10.0, 1.0);
      drcSimulationTestHelper.setupCameraForUnitTest(cameraFix, cameraPosition);
      fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();
      ThreadTools.sleep(1000);
   }

   public void destroySimulationAndRecycleMemory()
   {
      if (simulationTestingParameters.getKeepSCSUp())
      {
         ThreadTools.sleepForever();
      }

      if (drcSimulationTestHelper != null)
      {
         drcSimulationTestHelper.destroySimulation();
         drcSimulationTestHelper = null;
      }
      simulationTestingParameters = null;
   }

   private void addFootstep(Point3D stepLocation, Quaternion orient, RobotSide robotSide, FootstepDataListMessage message)
   {
      FootstepDataMessage footstepData = new FootstepDataMessage();
      footstepData.setLocation(stepLocation);
      footstepData.setOrientation(orient);
      footstepData.setRobotSide(robotSide);
      message.add(footstepData);
   }

   protected double getStepLength()
   {
      return 0.25;
   }

   protected double getStepWidth()
   {
      return 0.08;
   }

   protected int getNumberOfStepsToTake()
   {
      return 10;
   }

   private class AngularMomentumSpy extends SimpleRobotController
   {
      HumanoidFloatingRootJointRobot floatingRootJointModel;
      Vector3D comAngMom = new Vector3D();
      Vector3D comLinMom = new Vector3D();
      Point3D comPoint = new Point3D();
      YoFrameVector comAngularMomentum;
      FloatingJoint rootJoint;
      Quaternion rootOrientation = new Quaternion();
      RigidBodyTransform rootJointTransform = new RigidBodyTransform();
      
      public AngularMomentumSpy(DRCSimulationTestHelper simulationTestHelper)
      {
         YoVariableRegistry scsRegistry = drcSimulationTestHelper.getYovariableRegistry();
         drcSimulationTestHelper.addRobotControllerOnControllerThread(this);
         floatingRootJointModel = drcSimulationTestHelper.getRobot();
         rootJoint = floatingRootJointModel.getRootJoint();
         comAngularMomentum = new YoFrameVector("CoMAngularMomentum", worldFrame, scsRegistry);
      }

      @Override
      public void doControl()
      {
         floatingRootJointModel.computeCOMMomentum(comPoint, comLinMom, comAngMom);
         rootJoint.getRotationToWorld(rootOrientation);
         rootJointTransform.setRotationAndZeroTranslation(rootOrientation);
         rootJointTransform.inverseTransform(comAngMom);
         comAngularMomentum.set(comAngMom);
         PrintTools.info(rootOrientation.toString());
      }
   }
}