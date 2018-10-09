package us.ihmc.footstepPlanning.remoteStandaloneDataSet;

import controller_msgs.msg.dds.FootstepPlanningRequestPacket;
import controller_msgs.msg.dds.PlanarRegionsListMessage;
import org.junit.After;
import org.junit.Test;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlannerDataSetTest;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.footstepPlanning.SimpleFootstep;
import us.ihmc.footstepPlanning.tools.FootstepPlannerIOTools.FootstepPlannerUnitTestDataset;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.footstepPlanning.ui.ApplicationRunner;
import us.ihmc.footstepPlanning.ui.RemoteStandaloneFootstepPlannerUI;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.ros2.RealtimeRos2Node;

import java.util.concurrent.atomic.AtomicReference;

import static us.ihmc.footstepPlanning.ui.FootstepPlannerUserInterfaceAPI.FootstepPlanTopic;
import static us.ihmc.footstepPlanning.ui.FootstepPlannerUserInterfaceAPI.PlanningResultTopic;

public abstract class RemoteStandalonePlannerDataSetTest extends FootstepPlannerDataSetTest
{
   private static final boolean visualize = false;
   private static final String robotName = "testBot";

   protected RemoteStandaloneFootstepPlannerUI uiNode;
   private RealtimeRos2Node ros2Node;

   private IHMCRealtimeROS2Publisher<FootstepPlanningRequestPacket> footstepPlanningRequestPublisher;

   private final AtomicReference<FootstepPlan> publishedPlanReference = new AtomicReference<>(null);
   private final AtomicReference<FootstepPlanningResult> publishedResultReference = new AtomicReference<>(null);
   private AtomicReference<Boolean> publishedReceivedPlan = new AtomicReference<>(false);

   private AtomicReference<FootstepPlan> uiFootstepPlanReference;
   private AtomicReference<FootstepPlanningResult> uiPlanningResultReference;
   private AtomicReference<Boolean> uiReceivedPlan = new AtomicReference<>(false);
   private AtomicReference<Boolean> uiReceivedResult = new AtomicReference<>(false);

   private final AtomicReference<FootstepPlan> expectedPlan = new AtomicReference<>(null);
   private final AtomicReference<FootstepPlan> actualPlan = new AtomicReference<>(null);
   private final AtomicReference<FootstepPlanningResult> expectedResult = new AtomicReference<>(null);
   private final AtomicReference<FootstepPlanningResult> actualResult = new AtomicReference<>(null);

   protected DomainFactory.PubSubImplementation pubSubImplementation;

   public void setup()
   {
      ros2Node = ROS2Tools.createRealtimeRos2Node(pubSubImplementation, "ihmc_footstep_planner_test");
      uiNode = RemoteStandaloneFootstepPlannerUI.createUI(robotName, pubSubImplementation, visualize);
      ApplicationRunner.runApplication(uiNode);

      while (!uiNode.isRunning())
         ThreadTools.sleep(100);

      JavaFXMessager messager = uiNode.getMessager();

      footstepPlanningRequestPublisher = ROS2Tools.createPublisher(ros2Node, FootstepPlanningRequestPacket.class, ROS2Tools
            .getTopicNameGenerator(robotName, ROS2Tools.FOOTSTEP_PLANNER_TOOLBOX, ROS2Tools.ROS2TopicQualifier.INPUT));

      uiReceivedPlan = new AtomicReference<>(false);
      uiReceivedResult = new AtomicReference<>(false);

      publishedReceivedPlan = new AtomicReference<>(false);

      messager.registerTopicListener(FootstepPlanTopic, request -> uiReceivedPlan.set(true));
      messager.registerTopicListener(PlanningResultTopic, request -> uiReceivedResult.set(true));

      uiFootstepPlanReference = messager.createInput(FootstepPlanTopic);
      uiPlanningResultReference = messager.createInput(PlanningResultTopic);
   }


   @After
   public void tearDown() throws Exception
   {
      ros2Node.destroy();
      uiNode.stop();

      uiReceivedPlan = null;
      uiReceivedResult = null;

      publishedReceivedPlan = null;
      pubSubImplementation = null;

      uiFootstepPlanReference = null;
      uiPlanningResultReference = null;

      ros2Node = null;
      footstepPlanningRequestPublisher = null;
      uiNode = null;
   }

   @Override
   public void submitDataSet(FootstepPlannerUnitTestDataset dataset)
   {
      for (int i = 0; i < 10; i++)
         ThreadTools.sleep(100);

      byte plannerType = getPlannerType().toByte();
      PlanarRegionsListMessage planarRegions = PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(dataset.getPlanarRegionsList());

      FootstepPlanningRequestPacket planningRequestPacket = new FootstepPlanningRequestPacket();
      planningRequestPacket.getStanceFootPositionInWorld().set(dataset.getStart());
      planningRequestPacket.getGoalPositionInWorld().set(dataset.getGoal());
      planningRequestPacket.setRequestedFootstepPlannerType(plannerType);
      planningRequestPacket.getPlanarRegionsListMessage().set(planarRegions);
      planningRequestPacket.setTimeout(dataset.getTimeout(getPlannerType()));

      footstepPlanningRequestPublisher.publish(planningRequestPacket);
   }

   @Override
   public String findPlanAndAssertGoodResult(FootstepPlannerUnitTestDataset dataset)
   {
      double totalTimeWaiting = 0;
      double maxTimeToWait = dataset.getTimeout(getPlannerType()) + 5.0;
      long waitTime = 10;

      queryUIResults();
      queryPlannerResults();

      while (actualPlan.get() == null || actualResult.get() == null || expectedPlan.get() == null || expectedResult.get() == null)
      {
         if (totalTimeWaiting > maxTimeToWait)
            throw new RuntimeException("Overran our maximum wait time for a result");

         ThreadTools.sleep(waitTime);
         totalTimeWaiting += Conversions.millisecondsToSeconds(waitTime);
         queryUIResults();
         queryPlannerResults();
      }

      String datasetName = dataset.getDatasetName();

      String errorMessage = "";

      FootstepPlanningResult expectedResult = this.expectedResult.getAndSet(null);
      FootstepPlanningResult actualResult = this.actualResult.getAndSet(null);
      FootstepPlan expectedPlan = this.expectedPlan.getAndSet(null);
      FootstepPlan actualPlan = this.actualPlan.getAndSet(null);

      uiReceivedResult.set(false);
      uiReceivedPlan.set(false);
      publishedReceivedPlan.set(false);

      errorMessage += assertTrue(datasetName, "Planning result for " + datasetName + " is invalid, result was " + actualResult,
                                 actualResult.validForExecution());
      errorMessage += assertTrue(datasetName, "Planning results for " + datasetName + " are not equal: " + expectedResult + " and " + actualResult + ".\n",
                                 expectedResult.equals(actualResult));

      if (actualResult.validForExecution())
      {
         errorMessage += areFootstepPlansEqual(actualPlan, expectedPlan);
         errorMessage += assertTrue(datasetName, datasetName + " did not reach goal.", PlannerTools.isGoalNextToLastStep(dataset.getGoal(), actualPlan));
      }

      return errorMessage;
   }

   private void queryUIResults()
   {
      if (uiReceivedPlan.get() && uiFootstepPlanReference.get() != null && actualPlan.get() == null)
         actualPlan.set(uiFootstepPlanReference.get());

      if (uiReceivedResult.get() && uiPlanningResultReference.get() != null && actualResult.get() == null)
         actualResult.set(uiPlanningResultReference.get());
   }

   private void queryPlannerResults()
   {
      if (publishedReceivedPlan.get() && publishedPlanReference.get() != null && expectedPlan.get() == null)
         expectedPlan.set(publishedPlanReference.get());

      if (publishedReceivedPlan.get() && publishedResultReference.get() != null && expectedResult.get() == null)
         expectedResult.set(publishedResultReference.get());
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

      if (!footstepA.getRobotSide().equals(footstepB.getRobotSide()))
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
