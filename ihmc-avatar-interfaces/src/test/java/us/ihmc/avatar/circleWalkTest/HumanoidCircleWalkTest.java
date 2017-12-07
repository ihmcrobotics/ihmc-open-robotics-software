package us.ihmc.avatar.circleWalkTest;

import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.Random;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.avatar.DRCStartingLocation;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.initialSetup.OffsetAndYawRobotInitialSetup;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.ArmTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.OneDoFJointTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.robotController.SimpleRobotController;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationconstructionset.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public abstract class HumanoidCircleWalkTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   private DRCSimulationTestHelper drcSimulationTestHelper;
   Random random = new Random(42);

   @After
   public void tearDown()
   {
      drcSimulationTestHelper = null;
   }

   protected double getRadiusForCircle()
   {
      return 1.0;
   }

   protected double getStepLength()
   {
      return 0.25;
   }

   protected double getStepWidth()
   {
      return 0.08;
   }

   protected double getValidElbowAngle()
   {
      return -2.2 + (0.2 + 2.2) * random.nextDouble();
   }

   protected int getArmDoF()
   {
      return 6;
   }

   protected int getArmTrajectoryPoints()
   {
      return 10;
   }

   protected double getRandomValidJointAngle(RobotSide side, ArmJointName armJointName, FullHumanoidRobotModel fullRobotModel)
   {
      OneDoFJoint armJoint = fullRobotModel.getArmJoint(side, armJointName);
      if (armJoint != null)
      {
         double jointAngle = armJoint.getJointLimitLower() + (armJoint.getJointLimitUpper() - armJoint.getJointLimitLower()) * random.nextDouble();
         return jointAngle;
      }
      else
      {
         return 0.0;
      }
   }

   protected ArmJointName[] getArmJointNames()
   {
      return null;
   }

   @Test(timeout = 30000)
   public void testCircleWalk() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      // create simulation test helper
      FlatGroundEnvironment emptyEnvironment = new FlatGroundEnvironment();
      String className = getClass().getSimpleName();

      DRCStartingLocation startingLocation = new DRCStartingLocation()
      {
         @Override
         public OffsetAndYawRobotInitialSetup getStartingLocationOffset()
         {
            return new OffsetAndYawRobotInitialSetup(new Vector3D(getRadiusForCircle(), 0.0, 0.0), Math.PI / 2);
         }
      };
      DRCRobotModel robotModel = getRobotModel();
      FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, robotModel);
      drcSimulationTestHelper.setStartingLocation(startingLocation);
      drcSimulationTestHelper.setTestEnvironment(emptyEnvironment);
      drcSimulationTestHelper.createSimulation(className);
      ThreadTools.sleep(1000);

      setupCameraSideView();
      double radius = getRadiusForCircle();
      double stepLength = getStepLength();
      double stepWidth = getStepWidth();
      double dTheta = stepLength / radius;
      double theta = 0;
      ArmTrajectoryMessage leftHandMessage = new ArmTrajectoryMessage(RobotSide.LEFT, getArmDoF());
      ArmTrajectoryMessage rightHandMessage = new ArmTrajectoryMessage(RobotSide.RIGHT, getArmDoF());
      ArmJointName[] armJoint = getArmJointNames();
      if (armJoint == null)
      {
         System.out.println("Arm joint information not available");
         assertTrue(false);
      }

      RobotSide side = RobotSide.LEFT;
      FootstepDataListMessage footMessage = new FootstepDataListMessage();
      ArrayList<Point3D> rootLocations = new ArrayList<>();
      ArrayList<OneDoFJointTrajectoryMessage> leftArmTrajectory = new ArrayList<>();
      ArrayList<OneDoFJointTrajectoryMessage> rightArmTrajectory = new ArrayList<>();
      ControllerSpy controllerSpy = new ControllerSpy(drcSimulationTestHelper, fullRobotModel);

      for (int armJointIndex = 0; armJointIndex < getArmDoF(); armJointIndex++)
      {
         OneDoFJointTrajectoryMessage leftJointTrajectory = new OneDoFJointTrajectoryMessage(getArmTrajectoryPoints());
         OneDoFJointTrajectoryMessage rightJointTrajectory = new OneDoFJointTrajectoryMessage(getArmTrajectoryPoints());
         for (int trajectoryPointIndex = 0; trajectoryPointIndex < getArmTrajectoryPoints(); trajectoryPointIndex++)
         {

            leftJointTrajectory.setTrajectoryPoint(trajectoryPointIndex, 2 * trajectoryPointIndex + 1,
                                                   getRandomValidJointAngle(RobotSide.LEFT, armJoint[armJointIndex], fullRobotModel), 0);
            rightJointTrajectory.setTrajectoryPoint(trajectoryPointIndex, 2 * trajectoryPointIndex + 1,
                                                    getRandomValidJointAngle(RobotSide.RIGHT, armJoint[armJointIndex], fullRobotModel), 0);
         }
         leftHandMessage.setTrajectory1DMessage(armJointIndex, leftJointTrajectory);
         rightHandMessage.setTrajectory1DMessage(armJointIndex, rightJointTrajectory);
         leftArmTrajectory.add(leftJointTrajectory);
         rightArmTrajectory.add(rightJointTrajectory);
      }
      controllerSpy.setArmJointCheckPoints(leftArmTrajectory, rightArmTrajectory);

      while (theta < 2 * Math.PI)
      {
         if (drcSimulationTestHelper.getQueuedControllerCommands().isEmpty())
         {
            Point3D footLocation = new Point3D((radius + side.negateIfLeftSide(stepWidth / 2)) * Math.cos(theta),
                                               (radius + side.negateIfLeftSide(stepWidth / 2)) * Math.sin(theta), 0.0);
            rootLocations.add(new Point3D(radius * Math.cos(theta), radius * Math.sin(theta), 0.0));
            Quaternion footOrientation = new Quaternion(0.0, 0.0, Math.sin(theta / 2 + Math.PI / 4), Math.cos(theta / 2 + Math.PI / 4));
            addFootstep(footLocation, footOrientation, side, footMessage);
            side = side.getOppositeSide();
            theta += dTheta;
         }
      }

      addFootstep(new Point3D((radius + side.negateIfLeftSide(stepWidth / 2)), 0.0, 0.0),
                  new Quaternion(0.0, 0.0, Math.sin(Math.PI / 4), Math.sin(Math.PI / 4)), side, footMessage);
      side = side.getOppositeSide();
      addFootstep(new Point3D((radius + side.negateIfLeftSide(stepWidth / 2)), 0.0, 0.0),
                  new Quaternion(0.0, 0.0, Math.sin(Math.PI / 4), Math.sin(Math.PI / 4)), side, footMessage);

      controllerSpy.setFootStepCheckPoints(rootLocations, getStepLength(), getStepWidth());
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      drcSimulationTestHelper.send(leftHandMessage);
      drcSimulationTestHelper.send(rightHandMessage);
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      drcSimulationTestHelper.send(footMessage);
      int numberOfFootsteps = footMessage.footstepDataList.size();
      double defaultSwingTime = robotModel.getWalkingControllerParameters().getDefaultSwingTime();
      double defaultTransferTime = robotModel.getWalkingControllerParameters().getDefaultTransferTime();

      double simulationTime = numberOfFootsteps * defaultSwingTime + numberOfFootsteps * defaultTransferTime + 2.0;

      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationTime));
      ArrayList<YoBoolean> footCheckPointFlags = controllerSpy.getFootCheckPointFlag();
      for (int i = 0; i < footCheckPointFlags.size(); i++)
      {
         assertTrue(footCheckPointFlags.get(i).getBooleanValue());
      }
      ArrayList<YoBoolean> armCheckPointFlags = controllerSpy.getArmCheckPointFlag();
      for (int i = 0; i < armCheckPointFlags.size(); i++)
      {
         assertTrue(armCheckPointFlags.get(i).getBooleanValue());
      }

      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 2);
   }

   private void addFootstep(Point3D stepLocation, Quaternion orient, RobotSide robotSide, FootstepDataListMessage message)
   {
      FootstepDataMessage footstepData = new FootstepDataMessage();
      footstepData.setLocation(stepLocation);
      footstepData.setOrientation(orient);
      footstepData.setRobotSide(robotSide);
      message.add(footstepData);
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
      private ArmJointName[] armJoint = getArmJointNames();
      private YoVariableRegistry circleWalkRegistry;
      private final double EPSILON = 1e-2;
      private FullHumanoidRobotModel fullRobotModel;

      public ControllerSpy(DRCSimulationTestHelper drcSimulationTestHelper, FullHumanoidRobotModel fullRobotModel)
      {
         this.fullRobotModel = fullRobotModel;
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
         checkArmCheckPoints();
      }

      private double[] getExpectedJointPositions()
      {
         double[] expectedJointPos = new double[2 * getArmDoF()];
         for (int i = 0; i < expectedJointPos.length / 2; i++)
         {
            expectedJointPos[i] = leftArmMessages.get(i).getTrajectoryPoint(armJointCheckPointIndex).getPosition();
         }
         for (int i = 0; i < expectedJointPos.length / 2; i++)
         {
            expectedJointPos[i + expectedJointPos.length / 2] = rightArmMessages.get(i).getTrajectoryPoint(armJointCheckPointIndex).getPosition();
         }
         return expectedJointPos;
      }

      private double[] getCurrentJointPositions()
      {
         double[] currentJointPos = new double[2 * getArmDoF()];
         for (int i = 0; i < currentJointPos.length / 2; i++)
         {
            currentJointPos[i] = getJointDesiredPosition(RobotSide.LEFT, armJoint[i]).getDoubleValue();
         }
         for (int i = 0; i < currentJointPos.length / 2; i++)
         {
            currentJointPos[i + currentJointPos.length / 2] = getJointDesiredPosition(RobotSide.RIGHT, armJoint[i]).getDoubleValue();
         }
         return currentJointPos;
      }

      public boolean isEqual(double d1, double d2, double EPSILON)
      {
         return Math.abs(d1 - d2) < EPSILON;
      }

      private void checkArmCheckPoints()
      {
         if (armJointCheckPointIndex < armCheckPointFlag.size())
         {
            double[] expectedPositions = getExpectedJointPositions();
            double[] currentPositions = getCurrentJointPositions();

            for (int i = 0; i < expectedPositions.length; i++)
            {
               if (!isEqual(currentPositions[i], expectedPositions[i], EPSILON))
                  return;
            }
            armCheckPointFlag.get(armJointCheckPointIndex).set(true);
            armJointCheckPointIndex++;
         }
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

      public void setArmJointCheckPoints(ArrayList<OneDoFJointTrajectoryMessage> leftArmList, ArrayList<OneDoFJointTrajectoryMessage> rightArmList)
      {
         leftArmMessages = new ArrayList<>(leftArmList.size());
         armCheckPointFlag = new ArrayList<>(leftArmList.size()); // Assumed here that left and right arm lists are the same size

         int counter = 0;
         for (OneDoFJointTrajectoryMessage msg : leftArmList)
         {
            leftArmMessages.add(msg);
            YoBoolean newFlag = new YoBoolean("ArmJointCheckPointFlag" + Integer.toString(counter++), circleWalkRegistry);
            armCheckPointFlag.add(newFlag);
         }
         counter = 0;
         rightArmMessages = new ArrayList<>(rightArmList.size());
         for (OneDoFJointTrajectoryMessage msg : rightArmList)
         {
            rightArmMessages.add(msg);
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

      public boolean getArmJointCheckPointFlag(int index)
      {
         return armCheckPointFlag.get(index).getBooleanValue();
      }

      public ArrayList<YoBoolean> getArmCheckPointFlag()
      {
         return armCheckPointFlag;
      }
   }

}
