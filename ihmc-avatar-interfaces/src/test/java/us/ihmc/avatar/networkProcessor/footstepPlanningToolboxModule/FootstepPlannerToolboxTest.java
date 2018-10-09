package us.ihmc.avatar.networkProcessor.footstepPlanningToolboxModule;

import com.jme3.math.Transform;
import controller_msgs.msg.dds.*;
import org.junit.After;
import org.junit.Before;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.handControl.packetsAndConsumers.HandModel;
import us.ihmc.avatar.initialSetup.DRCRobotInitialSetup;
import us.ihmc.avatar.ros.DRCROSPPSTimestampOffsetProvider;
import us.ihmc.avatar.sensors.DRCSensorSuiteManager;
import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.ICPWithTimeFreezingPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.communication.packets.ToolboxState;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.footstepPlanning.DefaultFootstepPlanningParameters;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.footstepPlanning.SimpleFootstep;
import us.ihmc.footstepPlanning.graphSearch.FootstepPlannerParameters;
import us.ihmc.footstepPlanning.sharedMemoryDataSet.FootstepPlannerDataSetTest;
import us.ihmc.footstepPlanning.tools.FootstepPlannerIOTools.FootstepPlannerUnitTestDataset;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.footstepPlanning.ui.ApplicationRunner;
import us.ihmc.footstepPlanning.ui.RemoteFootstepPlannerUI;
import us.ihmc.footstepPlanning.ui.FootstepPlannerUserInterfaceAPI;
import us.ihmc.ihmcPerception.depthData.CollisionBoxProvider;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.robotDataLogger.logger.LogSettings;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.partNames.*;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SegmentDependentList;
import us.ihmc.robotics.screwTheory.*;
import us.ihmc.robotics.sensors.ContactSensorDefinition;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.ros2.RealtimeRos2Node;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorInformation;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.simulationConstructionSetTools.robotController.MultiThreadedRobotControlElement;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.tools.thread.CloseableAndDisposableRegistry;
import us.ihmc.wholeBodyController.DRCRobotJointMap;
import us.ihmc.wholeBodyController.RobotContactPointParameters;
import us.ihmc.wholeBodyController.concurrent.ThreadDataSynchronizerInterface;

import java.io.IOException;
import java.io.InputStream;
import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicReference;

import static us.ihmc.footstepPlanning.ui.FootstepPlannerUserInterfaceAPI.*;

public abstract class FootstepPlannerToolboxTest extends FootstepPlannerDataSetTest
{

   protected RemoteFootstepPlannerUI uiNode;
   private FootstepPlanningToolboxModule toolboxModule;
   private IHMCRealtimeROS2Publisher<FootstepPlanningRequestPacket> footstepPlanningRequestPublisher;
   private IHMCRealtimeROS2Publisher<ToolboxStateMessage> toolboxStatePublisher;

   private RealtimeRos2Node ros2Node;
   private final AtomicReference<FootstepPlan> footstepPlanReference = new AtomicReference<>(null);
   private final AtomicReference<FootstepPlanningResult> footstepResultReference = new AtomicReference<>(null);



   @Before
   public void setup()
   {
      tryToStartModule(() -> setupFootstepPlanningToolboxModule());
      uiNode = new RemoteFootstepPlannerUI("");
      ApplicationRunner.runApplication(uiNode);

      while (!uiNode.isRunning())
         ThreadTools.sleep(100);

      ros2Node = ROS2Tools.createRealtimeRos2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "ihmc_footstep_planner_test");

      String robotName = getRobotModel().getSimpleRobotName();
      footstepPlanningRequestPublisher = ROS2Tools.createPublisher(ros2Node, FootstepPlanningRequestPacket.class, ROS2Tools
            .getTopicNameGenerator(robotName, ROS2Tools.FOOTSTEP_PLANNER_TOOLBOX, ROS2Tools.ROS2TopicQualifier.INPUT));

      toolboxStatePublisher = ROS2Tools.createPublisher(ros2Node, ToolboxStateMessage.class, ROS2Tools
            .getTopicNameGenerator(robotName, ROS2Tools.FOOTSTEP_PLANNER_TOOLBOX, ROS2Tools.ROS2TopicQualifier.INPUT));

      ros2Node.spin();
   }

   @After
   public void tearDown() throws Exception
   {
      ros2Node.destroy();
      uiNode.stop();
      toolboxModule.destroy();

      ros2Node = null;
      footstepPlanningRequestPublisher = null;
      uiNode = null;
      toolboxModule = null;
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
      toolboxModule = new FootstepPlanningToolboxModule(getRobotModel(), null, true);
   }

   private DRCRobotModel getRobotModel()
   {
      return new TestRobotModel();
   }

   @Override
   public void submitDataSet(FootstepPlannerUnitTestDataset dataset)
   {
      toolboxStatePublisher.publish(MessageTools.createToolboxStateMessage(ToolboxState.WAKE_UP));

      FootstepPlanningRequestPacket planningRequestPacket = new FootstepPlanningRequestPacket();
      planningRequestPacket.getStanceFootPositionInWorld().set(dataset.getStart());
      planningRequestPacket.getGoalPositionInWorld().set(dataset.getGoal());
      planningRequestPacket.setRequestedFootstepPlannerType(getPlannerType().toByte());
      planningRequestPacket.getPlanarRegionsListMessage().set(PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(dataset.getPlanarRegionsList()));
      planningRequestPacket.setTimeout(dataset.getTimeout(getPlannerType()));

      footstepPlanningRequestPublisher.publish(planningRequestPacket);
   }

   @Override
   public String findPlanAndAssertGoodResult(FootstepPlannerUnitTestDataset dataset)
   {
      ROS2Tools.MessageTopicNameGenerator nameGenerator = ROS2Tools.getTopicNameGenerator("", ROS2Tools.FOOTSTEP_PLANNER_TOOLBOX, ROS2Tools.ROS2TopicQualifier.INPUT);
      ROS2Tools.createCallbackSubscription(ros2Node, FootstepPlanningToolboxOutputStatus.class, nameGenerator,
                                           s -> processFootstepPlanningOutputStatus(s.takeNextData()));

      FootstepPlanningResult result = footstepResultReference.getAndSet(null);
      FootstepPlan plan = footstepPlanReference.getAndSet(null);



      JavaFXMessager messager = uiNode.getMessager();

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

   private class TestRobotModel implements DRCRobotModel
   {
      @Override
      public DRCRobotJointMap getJointMap()
      {
         return null;
      }

      @Override
      public DRCRobotInitialSetup<HumanoidFloatingRootJointRobot> getDefaultRobotInitialSetup(double groundHeight, double initialYaw)
      {
         return null;
      }

      @Override
      public HandModel getHandModel()
      {
         return null;
      }

      @Override
      public Transform getJmeTransformWristToHand(RobotSide side)
      {
         return null;
      }

      @Override
      public double getSimulateDT()
      {
         return 0;
      }

      @Override
      public double getEstimatorDT()
      {
         return 0;
      }

      @Override
      public double getStandPrepAngle(String jointName)
      {
         return 0;
      }

      @Override
      public DRCROSPPSTimestampOffsetProvider getPPSTimestampOffsetProvider()
      {
         return null;
      }

      @Override
      public DRCSensorSuiteManager getSensorSuiteManager()
      {
         return null;
      }

      @Override
      public MultiThreadedRobotControlElement createSimulatedHandController(FloatingRootJointRobot simulatedRobot,
                                                                            ThreadDataSynchronizerInterface threadDataSynchronizer,
                                                                            RealtimeRos2Node realtimeRos2Node,
                                                                            CloseableAndDisposableRegistry closeableAndDisposableRegistry)
      {
         return null;
      }

      @Override
      public LogSettings getLogSettings()
      {
         return null;
      }

      @Override
      public LogModelProvider getLogModelProvider()
      {
         return null;
      }

      @Override
      public String getSimpleRobotName()
      {
         return "TestBot";
      }

      @Override
      public CollisionBoxProvider getCollisionBoxProvider()
      {
         return null;
      }

      @Override
      public HighLevelControllerParameters getHighLevelControllerParameters()
      {
         return null;
      }

      @Override
      public HumanoidFloatingRootJointRobot createHumanoidFloatingRootJointRobot(boolean createCollisionMeshes, boolean enableJointDamping)
      {
         return null;
      }

      @Override
      public RobotDescription getRobotDescription()
      {
         return null;
      }

      @Override
      public FullHumanoidRobotModel createFullRobotModel()
      {
         return new TestFullRobotModel();
      }

      @Override
      public double getControllerDT()
      {
         return 0;
      }

      @Override
      public StateEstimatorParameters getStateEstimatorParameters()
      {
         return null;
      }

      @Override
      public ICPWithTimeFreezingPlannerParameters getCapturePointPlannerParameters()
      {
         return null;
      }

      @Override
      public WalkingControllerParameters getWalkingControllerParameters()
      {
         return null;
      }

      @Override
      public RobotContactPointParameters<RobotSide> getContactPointParameters()
      {
         return null;
      }

      @Override
      public DRCRobotSensorInformation getSensorInformation()
      {
         return null;
      }

      @Override
      public InputStream getWholeBodyControllerParametersFile()
      {
         return null;
      }

      @Override
      public FootstepPlannerParameters getFootstepPlannerParameters()
      {
         return new DefaultFootstepPlanningParameters();
      }
   }

   public class TestFullRobotModel implements FullHumanoidRobotModel
   {

      @Override
      public RobotSpecificJointNames getRobotSpecificJointNames()
      {
         return null;
      }

      @Override
      public void updateFrames()
      {

      }

      @Override
      public MovingReferenceFrame getElevatorFrame()
      {
         return null;
      }

      @Override
      public FloatingInverseDynamicsJoint getRootJoint()
      {
         return null;
      }

      @Override
      public RigidBody getElevator()
      {
         return null;
      }

      @Override
      public OneDoFJoint getSpineJoint(SpineJointName spineJointName)
      {
         return null;
      }

      @Override
      public RigidBody getEndEffector(Enum<?> segmentEnum)
      {
         return null;
      }

      @Override
      public OneDoFJoint getNeckJoint(NeckJointName neckJointName)
      {
         return null;
      }

      @Override
      public InverseDynamicsJoint getLidarJoint(String lidarName)
      {
         return null;
      }

      @Override
      public ReferenceFrame getLidarBaseFrame(String name)
      {
         return null;
      }

      @Override
      public RigidBodyTransform getLidarBaseToSensorTransform(String name)
      {
         return null;
      }

      @Override
      public ReferenceFrame getCameraFrame(String name)
      {
         return null;
      }

      @Override
      public RigidBody getRootBody()
      {
         return null;
      }

      @Override
      public RigidBody getHead()
      {
         return null;
      }

      @Override
      public ReferenceFrame getHeadBaseFrame()
      {
         return null;
      }

      @Override
      public OneDoFJoint[] getOneDoFJoints()
      {
         return new OneDoFJoint[0];
      }

      @Override
      public Map<String, OneDoFJoint> getOneDoFJointsAsMap()
      {
         return null;
      }

      @Override
      public void getOneDoFJointsFromRootToHere(OneDoFJoint oneDoFJointAtEndOfChain, List<OneDoFJoint> oneDoFJointsToPack)
      {

      }

      @Override
      public OneDoFJoint[] getControllableOneDoFJoints()
      {
         return new OneDoFJoint[0];
      }

      @Override
      public void getOneDoFJoints(List<OneDoFJoint> oneDoFJointsToPack)
      {

      }

      @Override
      public OneDoFJoint getOneDoFJointByName(String name)
      {
         return null;
      }

      @Override
      public void getControllableOneDoFJoints(List<OneDoFJoint> oneDoFJointsToPack)
      {

      }

      @Override
      public IMUDefinition[] getIMUDefinitions()
      {
         return new IMUDefinition[0];
      }

      @Override
      public ForceSensorDefinition[] getForceSensorDefinitions()
      {
         return new ForceSensorDefinition[0];
      }

      @Override
      public ContactSensorDefinition[] getContactSensorDefinitions()
      {
         return new ContactSensorDefinition[0];
      }

      @Override
      public double getTotalMass()
      {
         return 0;
      }

      @Override
      public RigidBody getChest()
      {
         return null;
      }

      @Override
      public RigidBody getPelvis()
      {
         return null;
      }

      @Override
      public OneDoFJoint getArmJoint(RobotSide robotSide, ArmJointName armJointName)
      {
         return null;
      }

      @Override
      public RigidBody getHand(RobotSide robotSide)
      {
         return null;
      }

      @Override
      public MovingReferenceFrame getHandControlFrame(RobotSide robotSide)
      {
         return null;
      }

      @Override
      public void setJointAngles(RobotSide side, LimbName limb, double[] q)
      {

      }

      @Override
      public MovingReferenceFrame getFrameAfterLegJoint(RobotSide robotSegment, LegJointName legJointName)
      {
         return null;
      }

      @Override
      public OneDoFJoint getLegJoint(RobotSide robotSegment, LegJointName legJointName)
      {
         return null;
      }

      @Override
      public RigidBody getFoot(RobotSide robotSegment)
      {
         return null;
      }

      @Override
      public RigidBody getEndEffector(RobotSide robotSegment, LimbName limbName)
      {
         return null;
      }

      @Override
      public MovingReferenceFrame getEndEffectorFrame(RobotSide robotSegment, LimbName limbName)
      {
         return null;
      }

      @Override
      public MovingReferenceFrame getSoleFrame(RobotSide robotSegment)
      {
         return null;
      }

      @Override
      public SegmentDependentList<RobotSide, MovingReferenceFrame> getSoleFrames()
      {
         return null;
      }
   }
}
