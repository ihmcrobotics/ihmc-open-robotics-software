package us.ihmc.avatar.networkProcessor.footstepPlanningToolboxModule;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import controller_msgs.msg.dds.FootstepPlanningRequestPacket;
import controller_msgs.msg.dds.FootstepPlanningToolboxOutputStatus;
import org.junit.After;
import org.junit.Before;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.footstepPlanning.SimpleFootstep;
import us.ihmc.footstepPlanning.frameworkTests.DataSetFrameworkTest;
import us.ihmc.footstepPlanning.tools.FootstepPlannerIOTools.FootstepPlannerUnitTestDataset;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.footstepPlanning.ui.FootstepPlannerUIRosNode;
import us.ihmc.footstepPlanning.ui.FootstepPlannerUserInterfaceAPI;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.RealtimeRos2Node;

import java.io.IOException;
import java.util.concurrent.atomic.AtomicReference;

import static us.ihmc.footstepPlanning.ui.FootstepPlannerUserInterfaceAPI.*;

public abstract class NetworkedFootstepPlannerFrameworkTest extends DataSetFrameworkTest
{

   protected FootstepPlannerUIRosNode uiNode;
   private IHMCRealtimeROS2Publisher<FootstepPlanningRequestPacket> footstepPlanningRequestPublisher;
   private RealtimeRos2Node ros2Node;
   private final AtomicReference<FootstepPlan> footstepPlanReference = new AtomicReference<>(null);
   private final AtomicReference<FootstepPlanningResult> footstepResultReference = new AtomicReference<>(null);


   @Before
   public void setup()
   {
      tryToStartModule(() -> setupFootstepPlanningToolboxModule());
      uiNode = new FootstepPlannerUIRosNode("");

      ros2Node = ROS2Tools.createRealtimeRos2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "ihmc_footstep_planner_test");

      footstepPlanningRequestPublisher = ROS2Tools.createPublisher(ros2Node, FootstepPlanningRequestPacket.class, ROS2Tools
            .getTopicNameGenerator("", ROS2Tools.FOOTSTEP_PLANNER_TOOLBOX, ROS2Tools.ROS2TopicQualifier.INPUT));

      ros2Node.spin();
   }

   @After
   public void tearDown() throws Exception
   {
      ros2Node.destroy();
      uiNode.destroy();

      ros2Node = null;
      footstepPlanningRequestPublisher = null;
      uiNode = null;
   }

   private void tryToStartModule(ModuleStarter runnable)
   {
      try
      {
         runnable.startModule();
      }
      catch (RuntimeException | IOException e)
      {
         PrintTools.error(this, "Failed to start a module in the network processor, stack trace:");
         e.printStackTrace();
      }
   }

   private interface ModuleStarter
   {
      void startModule() throws IOException;
   }

   private void setupFootstepPlanningToolboxModule() throws IOException
   {
      new FootstepPlanningToolboxModule(null, null, null, true);
   }


   @Override
   public void submitDataSet(FootstepPlannerUnitTestDataset dataset)
   {
      FootstepPlanningRequestPacket planningRequestPacket = new FootstepPlanningRequestPacket();
      planningRequestPacket.getStanceFootPositionInWorld().set(dataset.getStart());
      planningRequestPacket.getGoalPositionInWorld().set(dataset.getGoal());
      planningRequestPacket.setRequestedFootstepPlannerType(getPlannerType().toByte());
      planningRequestPacket.getPlanarRegionsListMessage().set(PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(dataset.getPlanarRegionsList()));

      footstepPlanningRequestPublisher.publish(planningRequestPacket);



      //      JavaFXMessager messager = uiNode.getMessager();
      //      messager.submitMessage(FootstepPlannerUserInterfaceAPI.PlannerTimeoutTopic, dataset.getTimeout(getPlannerType()));
//      messager.submitMessage(FootstepPlannerUserInterfaceAPI.PlannerHorizonLengthTopic, Double.MAX_VALUE);
   }

   @Override
   public String findPlanAndAssertGoodResult(FootstepPlannerUnitTestDataset dataset)
   {
      ROS2Tools.MessageTopicNameGenerator nameGenerator = ROS2Tools.getTopicNameGenerator("", ROS2Tools.FOOTSTEP_PLANNER_TOOLBOX, ROS2Tools.ROS2TopicQualifier.INPUT);
      ROS2Tools.createCallbackSubscription(ros2Node, FootstepPlanningToolboxOutputStatus.class, nameGenerator,
                                           s -> processFootstepPlanningOutputStatus(s.takeNextData()));

      FootstepPlanningResult result = footstepResultReference.getAndSet(null);
      FootstepPlan plan = footstepPlanReference.getAndSet(null);



      JavaFXMessager messager = uiNode.getUI().getMessager();

      AtomicReference<Boolean> receivedPlan = new AtomicReference<>(false);
      AtomicReference<Boolean> receivedResult = new AtomicReference<>(false);
      messager.registerTopicListener(FootstepPlanTopic, request -> receivedPlan.set(true));
      messager.registerTopicListener(PlanningResultTopic, request -> receivedResult.set(true));

      AtomicReference<FootstepPlan> footstepPlanReference = messager.createInput(FootstepPlanTopic);
      AtomicReference<FootstepPlanningResult> footstepPlanningResult = messager.createInput(PlanningResultTopic);

      messager.submitMessage(FootstepPlannerUserInterfaceAPI.ComputePathTopic, true);

      while (!receivedPlan.get() && !receivedResult.get())
      {
         ThreadTools.sleep(10);
      }
      String datasetName = dataset.getDatasetName();

      int ticksToWait = 100;
      int tick = 0;
      if (receivedResult.get() && footstepPlanningResult.get().validForExecution())
      { // we know there's a valid plan, so wait until we've received it
         while (!receivedPlan.get())
         {
            if (tick > ticksToWait)
               return "Supposedly found a solution, but never received a plan out.";
            ThreadTools.sleep(10);
            tick++;
         }
      }

      String errorMessage = "";


      errorMessage += assertTrue(datasetName, "Planning results for " + datasetName + " are not equal: " + result + " and " + footstepPlanningResult.get() + ".\n",
                                 result.equals(footstepPlanningResult.get()));
      errorMessage += assertTrue(datasetName, "Planning result for " + datasetName + " is invalid, result was " + footstepPlanningResult.get(),
                                 footstepPlanningResult.get().validForExecution());

      if (footstepPlanningResult.get().validForExecution())
      {
         errorMessage += areFootstepPlansEqual(plan, footstepPlanReference.get());


         errorMessage += assertTrue(datasetName, datasetName + " did not reach goal.",
                                    PlannerTools.isGoalNextToLastStep(dataset.getGoal(), footstepPlanReference.get()));
      }

      return errorMessage;
   }

   private void processFootstepPlanningOutputStatus(FootstepPlanningToolboxOutputStatus packet)
   {
      footstepResultReference.set(FootstepPlanningResult.fromByte(packet.getFootstepPlanningResult()));
      footstepPlanReference.set(convertToFootstepPlan(packet.getFootstepDataList()));

   }

   private static FootstepPlan convertToFootstepPlan(FootstepDataListMessage footstepDataListMessage)
   {
      FootstepPlan footstepPlan = new FootstepPlan();

      for (FootstepDataMessage footstepMessage : footstepDataListMessage.getFootstepDataList())
      {
         FramePose3D stepPose = new FramePose3D();
         stepPose.setPosition(footstepMessage.getLocation());
         stepPose.setOrientation(footstepMessage.getOrientation());
         footstepPlan.addFootstep(RobotSide.fromByte(footstepMessage.getRobotSide()), stepPose);
      }

      return footstepPlan;
   }

   private String assertTrue(String datasetName, String message, boolean condition)
   {
      if (VISUALIZE || DEBUG)
      {
         if (!condition)
            PrintTools.error(datasetName + ": " + message);
      }
      return !condition ? "\n" + message : "";
   }

   private String areFootstepPlansEqual(FootstepPlan footstepPlanA, FootstepPlan footstepPlanB)
   {
      String errorMessage = "";

      if (footstepPlanA.getNumberOfSteps() != footstepPlanB.getNumberOfSteps())
      {
         errorMessage += "Plan A has " + footstepPlanA.getNumberOfSteps() + ", while Plan B has " + footstepPlanB.getNumberOfSteps() + ".\n";
      }

      for (int i = 0; i < Math.min(footstepPlanA.getNumberOfSteps(), footstepPlanB.getNumberOfSteps()); i++)
      {
         errorMessage += areFootstepsEqual(i, footstepPlanA.getFootstep(i), footstepPlanB.getFootstep(i));
      }

      return errorMessage;
   }

   private String areFootstepsEqual(int footstepNumber, SimpleFootstep footstepA, SimpleFootstep footstepB)
   {
      String errorMessage = "";

      if (footstepA.getRobotSide().equals(footstepB.getRobotSide()))
      {
         errorMessage += "Footsteps " + footstepNumber + " are different robot sides: " + footstepA.getRobotSide() + " and " + footstepB.getRobotSide() + ".\n";
      }

      FramePose3D poseA = new FramePose3D();
      FramePose3D poseB = new FramePose3D();

      footstepA.getSoleFramePose(poseA);
      footstepB.getSoleFramePose(poseB);

      if (!poseA.epsilonEquals(poseB, 1e-5))
      {
         errorMessage += "Footsteps " + footstepNumber + " have different poses: \n \t" + poseA.toString() + "\n and \n\t " + poseB.toString() + ".\n";
      }

      if (!footstepA.epsilonEquals(footstepB, 1e-5))

      {
         errorMessage += "Footsteps " + footstepNumber + " are not equal: \n \t" + footstepA.toString() + "\n and \n\t " + footstepB.toString() + ".\n";
      }

      return errorMessage;
   }

}
