package us.ihmc.avatar.kinematicsSimulation;

import controller_msgs.msg.dds.FootstepDataListMessage;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import us.ihmc.avatar.AvatarTestScripts;
import us.ihmc.avatar.AvatarTestYoVariables;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.util.simulationrunner.GoalOrientedTestConductor;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;

public abstract class HumanoidKinematicsSimulationTest
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   private AvatarTestYoVariables variables;
   private GoalOrientedTestConductor conductor;
   private DRCRobotModel robotModel;
   private IHMCROS2Publisher<FootstepDataListMessage> footstepDataListPublisher;
   private Ros2Node ros2Node;

   public void takeSomeSteps()
   {
      // under construction
   }

//   private FootstepPlanningToolboxOutputStatus setupForFootstepTest() throws IOException
//   {
//      new MultiStageFootstepPlanningModule(robotModel, null, false, PubSubImplementation.INTRAPROCESS);
//
//      ros2Node = new Ros2Node(PubSubImplementation.INTRAPROCESS, getClass().getSimpleName());
//
//      footstepDataListPublisher = ROS2Tools.createPublisher(ros2Node,
//                                                            ROS2Tools.newMessageInstance(FootstepDataListCommand.class).getMessageClass(),
//                                                            ControllerAPIDefinition.getSubscriberTopicNameGenerator(robotModel.getSimpleRobotName()));
//
//   }

   public abstract AvatarTestYoVariables createTestYoVariables(SimulationConstructionSet scs);

   public abstract DRCRobotModel createRobotModel();

   public abstract SimulationConstructionSet createSCS();

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
      SimulationConstructionSet scs = createSCS();
      variables = createTestYoVariables(scs);
      conductor = new GoalOrientedTestConductor(scs, simulationTestingParameters);
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      AvatarTestScripts.standUp(conductor, variables);
   }

   @AfterAll
   public static void printMemoryUsageAfterClass()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(HumanoidKinematicsSimulationTest.class + " after class.");
   }
}
