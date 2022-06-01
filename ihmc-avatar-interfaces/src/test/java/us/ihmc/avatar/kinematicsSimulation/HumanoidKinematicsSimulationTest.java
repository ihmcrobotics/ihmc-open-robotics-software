package us.ihmc.avatar.kinematicsSimulation;

import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;

import controller_msgs.msg.dds.FootstepDataListMessage;
import us.ihmc.avatar.AvatarTestScripts;
import us.ihmc.avatar.AvatarTestYoVariables;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.scs2.SimulationConstructionSet2;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.util.simulationrunner.GoalOrientedTestConductor;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;

public abstract class HumanoidKinematicsSimulationTest
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   private AvatarTestYoVariables variables;
   private GoalOrientedTestConductor conductor;
   private DRCRobotModel robotModel;
   private IHMCROS2Publisher<FootstepDataListMessage> footstepDataListPublisher;
   private ROS2Node ros2Node;

   public void takeSomeSteps()
   {
      // under construction
   }

//   private FootstepPlanningToolboxOutputStatus setupForFootstepTest() throws IOException
//   {
//      new MultiStageFootstepPlanningModule(robotModel, null, false, PubSubImplementation.INTRAPROCESS);
//
//      ros2Node = new ROS2Node(PubSubImplementation.INTRAPROCESS, getClass().getSimpleName());
//
//      footstepDataListPublisher = ROS2Tools.createPublisherTypeNamed(ros2Node,
//                                                            ROS2Tools.newMessageInstance(FootstepDataListCommand.class).getMessageClass(),
//                                                            ControllerAPIDefinition.getInputTopic(robotModel.getSimpleRobotName()));
//
//   }

   public abstract AvatarTestYoVariables createTestYoVariables(SimulationConstructionSet2 scs);

   public abstract DRCRobotModel createRobotModel();

   public abstract SimulationConstructionSet2 createSCS();

   @AfterEach
   public void destroySimulationAndRecycleMemory()
   {
      conductor.concludeTesting();
      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   @BeforeEach
   public void setUp()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
      robotModel = createRobotModel();
      SimulationConstructionSet2 scs = createSCS();
      variables = createTestYoVariables(scs);
      conductor = new GoalOrientedTestConductor(scs, simulationTestingParameters);
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      AvatarTestScripts.awaitStandUp(conductor, variables);
   }

   @AfterAll
   public static void printMemoryUsageAfterClass()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(HumanoidKinematicsSimulationTest.class + " after class.");
   }
}
