package us.ihmc.avatar.behaviorTests;

import static us.ihmc.robotics.Assert.*;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.DRCNetworkModuleParameters;
import us.ihmc.avatar.testTools.DRCBehaviorTestHelper;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commons.thread.ThreadTools;
import org.junit.jupiter.api.Tag;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.humanoidBehaviors.behaviors.roughTerrain.CollaborativeBehavior;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.sensorProcessing.parameters.HumanoidRobotSensorInformation;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;

@Tag("humanoid-behaviors")
public abstract class AvatarCollaborativeBehaviorTest implements MultiRobotTestInterface
{

   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   DRCBehaviorTestHelper drcBehaviorTestHelper;

   @AfterEach
   public void tearDown()
   {
      ReferenceFrameTools.clearWorldFrameTree();
   }

   @Test
   public void testBehavior() throws SimulationExceededMaximumTimeException
   {
      DRCRobotModel robotModel = getRobotModel();
      HumanoidRobotSensorInformation robotSensorInfo = robotModel.getSensorInformation();
      FlatGroundEnvironment flatGround = new FlatGroundEnvironment();
      DRCNetworkModuleParameters networkModuleParameters = new DRCNetworkModuleParameters();
      networkModuleParameters.enableBehaviorModule(true);
      networkModuleParameters.enableSensorModule(true);

      System.out.println(networkModuleParameters.toString());
      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT_BUT_ALMOST_PI;
      drcBehaviorTestHelper = new DRCBehaviorTestHelper(flatGround, "DRCSimpleFlatGroundScriptTest", selectedLocation, simulationTestingParameters, robotModel,
                                                        networkModuleParameters, true);
      FullHumanoidRobotModel fullRobotModel = drcBehaviorTestHelper.getControllerFullRobotModel();
      SimulationConstructionSet scs = drcBehaviorTestHelper.getSimulationConstructionSet();

      HumanoidReferenceFrames referenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      WalkingControllerParameters walkingControllerParameters = robotModel.getWalkingControllerParameters();
      Ros2Node ros2Node = drcBehaviorTestHelper.getRos2Node();
      CollaborativeBehavior collaborativeBehavior = new CollaborativeBehavior(robotModel.getSimpleRobotName(), ros2Node, referenceFrames, fullRobotModel,
                                                                              robotSensorInfo, walkingControllerParameters, null);
      scs.addYoVariableRegistry(collaborativeBehavior.getYoVariableRegistry());

      drcBehaviorTestHelper.setupCameraForUnitTest(new Point3D(0.0, 0.0, 1.0), new Point3D(10.0, 10.0, 3.0));
      ThreadTools.sleep(1000);
      assertTrue(drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(0.5));
      drcBehaviorTestHelper.dispatchBehavior(collaborativeBehavior);
      assertTrue(drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(5));
   }
}
