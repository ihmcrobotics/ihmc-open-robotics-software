package us.ihmc.avatar;

import static org.junit.Assert.*;

import java.util.ArrayList;
import java.util.Random;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.initialSetup.OffsetAndYawRobotInitialSetup;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.OneDoFJointTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.simulationConstructionSetTools.robotController.SimpleRobotController;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationconstructionset.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.thread.ThreadTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public abstract class AvatarFlatGroundForwardWalkingTest implements MultiRobotTestInterface
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();
   private OffsetAndYawRobotInitialSetup location = new OffsetAndYawRobotInitialSetup(new Vector3D(0.0, 0.0, 0.0), 0.0);
   private DRCSimulationTestHelper drcSimulationTestHelper;
   private DRCRobotModel robotModel;
   private FullHumanoidRobotModel fullRobotModel;
   Random random = new Random();

   protected int getNumberOfSteps()
   {
      return 10;
   }

   protected double getStepLength()
   {
      return 0.25;
   }

   protected double getStepWidth()
   {
      return 0.08;
   }

   @Test
   public void testCircleWalk() throws SimulationExceededMaximumTimeException
   {
      setupTest();
      setupCameraSideView();
      
      int numberOfSteps = getNumberOfSteps();
      double stepLength = getStepLength();
      double stepWidth = getStepWidth();

      RobotSide side = RobotSide.LEFT;
      
      FootstepDataListMessage footMessage = new FootstepDataListMessage();
      ArrayList<Point3D> rootLocations = new ArrayList<>();
      
      ControllerSpy controllerSpy = new ControllerSpy(drcSimulationTestHelper);
      
      for (int currentStep = 0; currentStep < numberOfSteps; currentStep++)
      {
         if (drcSimulationTestHelper.getQueuedControllerCommands().isEmpty())
         {
            Point3D footLocation = new Point3D(stepLength * currentStep, side.negateIfRightSide(stepWidth / 2), 0.0);
            rootLocations.add(new Point3D(stepLength * currentStep, 0.0, 0.0));
            Quaternion footOrientation = new Quaternion(0.0, 0.0, 0.0, 1.0);
            addFootstep(footLocation, footOrientation, side, footMessage);
            side = side.getOppositeSide();
         }
      }
      Point3D footLocation = new Point3D(stepLength * (numberOfSteps-1), side.negateIfRightSide(stepWidth / 2), 0.0);
      rootLocations.add(new Point3D(stepLength * (numberOfSteps-1), 0.0, 0.0));
      Quaternion footOrientation = new Quaternion(0.0, 0.0, 0.0, 1.0);
      addFootstep(footLocation, footOrientation, side, footMessage);

      controllerSpy.setFootStepCheckPoints(rootLocations, getStepLength(), getStepWidth());
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      drcSimulationTestHelper.send(footMessage);
      double simulationTime = 1 * footMessage.footstepDataList.size() + 1.0;

      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationTime));
      ArrayList<YoBoolean> footCheckPointFlags = controllerSpy.getFootCheckPointFlag();
      for (int i = 0; i < footCheckPointFlags.size(); i++)
      {
         assertTrue(footCheckPointFlags.get(i).getBooleanValue());
      }
   }

   private void addFootstep(Point3D stepLocation, Quaternion orient, RobotSide robotSide, FootstepDataListMessage message)
   {
      FootstepDataMessage footstepData = new FootstepDataMessage();
      footstepData.setLocation(stepLocation);
      footstepData.setOrientation(orient);
      footstepData.setRobotSide(robotSide);
      message.add(footstepData);
   }

   private void setupTest()
   {
//      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      // create simulation test helper
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
      robotModel = getRobotModel();
      fullRobotModel = robotModel.createFullRobotModel();
      simulationTestingParameters.setKeepSCSUp(keepSCSUp());
      drcSimulationTestHelper = new DRCSimulationTestHelper(emptyEnvironment, className, startingLocation, simulationTestingParameters, robotModel);
      ThreadTools.sleep(1000);
   }

   protected boolean keepSCSUp()
   {
      return false;
   }

   private void setupCameraBackView()
   {
      Point3D cameraFix = new Point3D(0.0, 0.0, 1.0);
      Point3D cameraPosition = new Point3D(-10.0, 0.0, 1.0);
      drcSimulationTestHelper.setupCameraForUnitTest(cameraFix, cameraPosition);
   }

   private void setupCameraSideView()
   {
      Point3D cameraFix = new Point3D(0.0, 0.0, 1.0);
      Point3D cameraPosition = new Point3D(0.0, 10.0, 1.0);
      drcSimulationTestHelper.setupCameraForUnitTest(cameraFix, cameraPosition);
   }

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

   private class ControllerSpy extends SimpleRobotController
   {
      private final HumanoidFloatingRootJointRobot humanoidRobotModel;
      private final DRCSimulationTestHelper drcSimTestHelper;

      private ArrayList<YoBoolean> footCheckPointFlag;
      private ArrayList<BoundingBox3D> footCheckPoint;
      private Point3D position;
      private int footStepCheckPointIndex;

      private ArrayList<YoBoolean> armCheckPointFlag;
      private ArrayList<OneDoFJointTrajectoryMessage> leftArmMessages;
      private ArrayList<OneDoFJointTrajectoryMessage> rightArmMessages;
      private int armJointCheckPointIndex;
      private YoVariableRegistry circleWalkRegistry;
      private final double EPSILON = 1e-2;

      public ControllerSpy(DRCSimulationTestHelper drcSimulationTestHelper)
      {
         humanoidRobotModel = drcSimulationTestHelper.getRobot();
         drcSimTestHelper = drcSimulationTestHelper;
         drcSimulationTestHelper.addRobotControllerOnControllerThread(this);
         position = new Point3D();
         footStepCheckPointIndex = 0;

         armCheckPointFlag = new ArrayList<>();
         armJointCheckPointIndex = 0;
         circleWalkRegistry = new YoVariableRegistry("CircleWalkTest");
      }

      @Override
      public void doControl()
      {
         checkFootCheckPoints();
      }

      public boolean isEqual(double d1, double d2, double EPSILON)
      {
         return Math.abs(d1 - d2) < EPSILON;
      }

      private void checkFootCheckPoints()
      {
         humanoidRobotModel.getRootJoint().getPosition(position);
         if (footStepCheckPointIndex < footCheckPoint.size() && footCheckPoint.get(footStepCheckPointIndex).isInsideInclusive(position))
         {
            footCheckPointFlag.get(footStepCheckPointIndex).set(true);
            if (footStepCheckPointIndex < footCheckPoint.size())
               footStepCheckPointIndex++;
         }
      }

      public void setFootStepCheckPoints(ArrayList<Point3D> locations, double xRange, double yRange)
      {
         footCheckPointFlag = new ArrayList<>(locations.size());
         footCheckPoint = new ArrayList<>(locations.size());

         for (int i = 0; i < locations.size(); i++)
         {
            Point3D minBound = new Point3D(locations.get(i));
            minBound.add(-xRange / 2.0, -yRange / 2.0, -10.0);
            Point3D maxBound = new Point3D(locations.get(i));
            maxBound.add(xRange / 2.0, yRange / 2.0, 10.0);
            footCheckPoint.add(new BoundingBox3D(minBound, maxBound));
            YoBoolean newFlag = new YoBoolean("FootstepCheckPointFlag" + Integer.toString(i), circleWalkRegistry);
            footCheckPointFlag.add(newFlag);
         }
      }

      public YoDouble getJointDesiredPosition(RobotSide side, OneDoFJoint joint)
      {
         String variable = "q_d_" + joint.getName();
         return (YoDouble) humanoidRobotModel.getVariable(variable);
      }

      public YoDouble getJointDesiredPosition(RobotSide side, ArmJointName jointName)
      {
         String variable = "q_d_" + fullRobotModel.getArmJoint(side, jointName).getName();
         return (YoDouble) humanoidRobotModel.getVariable(variable);
      }

      public boolean getFootCheckPointFlag(int index)
      {
         return footCheckPointFlag.get(index).getBooleanValue();
      }

      public ArrayList<YoBoolean> getFootCheckPointFlag()
      {
         return footCheckPointFlag;
      }
   }
}
