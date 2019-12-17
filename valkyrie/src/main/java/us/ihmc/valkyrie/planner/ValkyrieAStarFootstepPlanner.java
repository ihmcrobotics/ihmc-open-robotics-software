package us.ihmc.valkyrie.planner;

import controller_msgs.msg.dds.*;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.footstepPlanning.AdaptiveSwingTrajectoryCalculator;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ROS2Tools.MessageTopicNameGenerator;
import us.ihmc.communication.ROS2Tools.ROS2TopicQualifier;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapAndWiggler;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapData;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapperReadOnly;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.SimplePlanarRegionFootstepNodeSnapper;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.heuristics.DistanceAndYawBasedHeuristics;
import us.ihmc.footstepPlanning.graphSearch.nodeExpansion.ParameterBasedNodeExpansion;
import us.ihmc.footstepPlanning.graphSearch.stepCost.*;
import us.ihmc.footstepPlanning.ui.ApplicationRunner;
import us.ihmc.idl.IDLSequence.Object;
import us.ihmc.log.LogTools;
import us.ihmc.pathPlanning.graph.search.AStarIterationData;
import us.ihmc.pathPlanning.graph.search.AStarPathPlanner;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.trajectories.TrajectoryType;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.configuration.ValkyrieRobotVersion;
import us.ihmc.valkyrie.parameters.ValkyrieAdaptiveSwingParameters;
import us.ihmc.valkyrie.planner.ui.ValkyrieFootstepPlannerUI;
import us.ihmc.valkyrieRosControl.ValkyrieRosControlController;
import us.ihmc.wholeBodyController.RobotContactPointParameters;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Consumer;

public class ValkyrieAStarFootstepPlanner
{
   private static final boolean sendCroppedFootholds = false;
   private static final boolean setSwingParameters = true;
   private static final double statusPublishPeriod = 1.0;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final ValkyrieRobotModel robotModel;
   private final ValkyrieAStarFootstepPlannerParameters parameters = new ValkyrieAStarFootstepPlannerParameters(registry);
   private final AStarPathPlanner<FootstepNode> planner;
   private final SimplePlanarRegionFootstepNodeSnapper snapper;
   private final FootstepNodeSnapAndWiggler snapAndWiggler;
   private final ValkyrieFootstepValidityChecker stepValidityChecker;
   private final DistanceAndYawBasedHeuristics heuristics;
   private final AdaptiveSwingTrajectoryCalculator swingParameterCalculator;

   private Status status = null;
   private FootstepNode endNode = null;
   private double endNodeCost;

   private Consumer<ValkyrieFootstepPlanningRequestPacket> requestCallback = request -> {};
   private Consumer<AStarIterationData<FootstepNode>> iterationCallback = iterationData -> {};
   private Consumer<ValkyrieFootstepPlanningStatus> statusCallback = result -> {};

   private final AtomicBoolean isPlanning = new AtomicBoolean();
   private final AtomicBoolean haltRequested = new AtomicBoolean();
   private final ValkyrieFootstepPlanningRequestPacket requestPacket = new ValkyrieFootstepPlanningRequestPacket();
   private final Stopwatch stopwatch = new Stopwatch();

   public static final String MODULE_NAME = "valkyrie_footstep_planner";
   private Ros2Node ros2Node;

   public ValkyrieAStarFootstepPlanner(ValkyrieRobotModel robotModel)
   {
      this.robotModel = robotModel;
      SideDependentList<ConvexPolygon2D> footPolygons = createFootPolygons(robotModel);
      snapper = new SimplePlanarRegionFootstepNodeSnapper(footPolygons);
      snapAndWiggler  = new FootstepNodeSnapAndWiggler(footPolygons, () -> true, parameters::getWiggleInsideDelta, parameters::getMaximumXYWiggle, parameters::getMaximumYawWiggle, () -> Double.POSITIVE_INFINITY);
      swingParameterCalculator = new AdaptiveSwingTrajectoryCalculator(new ValkyrieAdaptiveSwingParameters(), robotModel.getWalkingControllerParameters());

      ParameterBasedNodeExpansion nodeExpansion = new ParameterBasedNodeExpansion(parameters::getMinimumFootstepLength,
                                                                                  parameters::getMaximumStepReach,
                                                                                  parameters::getStepYawReductionFactorAtMaxReach,
                                                                                  parameters::getMinimumStepYaw,
                                                                                  parameters::getMaximumStepYaw,
                                                                                  parameters::getMinimumStepWidth,
                                                                                  parameters::getMaximumStepWidth,
                                                                                  parameters::getIdealFootstepWidth,
                                                                                  parameters::getMinimumXClearanceFromStance,
                                                                                  parameters::getMinimumYClearanceFromStance);

      CompositeFootstepCost stepCost = new CompositeFootstepCost();
      stepCost.addFootstepCost(new FootholdAreaCost(parameters::getFootholdAreaWeight, footPolygons, snapper));
      stepCost.addFootstepCost(new HeightCost(() -> true, parameters.getTranslationWeight()::getZ, parameters.getTranslationWeight()::getZ, snapper));
      stepCost.addFootstepCost(new PitchAndRollBasedCost(parameters.getOrientationWeight()::getPitch, parameters.getOrientationWeight()::getRoll, snapper));
      stepCost.addFootstepCost(new QuadraticDistanceAndYawCost(parameters::getIdealFootstepWidth,
                                                               parameters::getIdealFootstepLength,
                                                               parameters.getTranslationWeight()::getX,
                                                               parameters.getTranslationWeight()::getX,
                                                               parameters.getTranslationWeight()::getY,
                                                               parameters.getOrientationWeight()::getYaw,
                                                               parameters::getCostPerStep));

      stepValidityChecker = new ValkyrieFootstepValidityChecker(parameters, footPolygons, snapper);

      heuristics = new DistanceAndYawBasedHeuristics(parameters.getTranslationWeight()::getZ,
                                                     parameters.getTranslationWeight()::getZ,
                                                     parameters::getMaximumStepReach,
                                                     parameters::getMaximumStepYaw,
                                                     parameters.getOrientationWeight()::getYaw,
                                                     parameters::getCostPerStep,
                                                     parameters::getFinalTurnProximity,
                                                     () -> 0.25,
                                                     parameters::getIdealFootstepWidth,
                                                     parameters::getAstarHeuristicsWeight,
                                                     snapper);

      planner = new AStarPathPlanner<>(nodeExpansion::expandNode, stepValidityChecker::checkFootstep, stepCost::compute, heuristics::compute);
   }

   public void handleRequestPacket(ValkyrieFootstepPlanningRequestPacket requestPacket)
   {
      if (isPlanning.get())
      {
         LogTools.info("Received planning request packet but planner is currently running");
         return;
      }

      this.requestPacket.set(requestPacket);
      isPlanning.set(true);
      haltRequested.set(false);
      status = Status.PLANNING;

      FramePose3D goalMidFootPose = new FramePose3D();
      goalMidFootPose.interpolate(requestPacket.getGoalLeftFootPose(), requestPacket.getGoalRightFootPose(), 0.5);
      heuristics.setGoalPose(goalMidFootPose);

      requestCallback.accept(requestPacket);

      // update parameters
      parameters.setFromPacket(requestPacket.getParameters());

      // update planar regions
      if (requestPacket.getAssumeFlatGround() || requestPacket.getPlanarRegionsListMessage().getRegionId().isEmpty())
      {
         snapper.setPlanarRegions(null);
         snapAndWiggler.setPlanarRegions(null);
         stepValidityChecker.setPlanarRegionsList(null);
         swingParameterCalculator.setPlanarRegionsList(null);
      }
      else
      {
         PlanarRegionsList planarRegionsList = PlanarRegionMessageConverter.convertToPlanarRegionsList(requestPacket.getPlanarRegionsListMessage());
         snapper.setPlanarRegions(planarRegionsList);
         snapAndWiggler.setPlanarRegions(planarRegionsList);
         stepValidityChecker.setPlanarRegionsList(planarRegionsList);
         swingParameterCalculator.setPlanarRegionsList(planarRegionsList);
      }

      // Set up planner
      FootstepNode startNode = createStartNode(requestPacket);
      endNode = startNode;
      endNodeCost = heuristics.compute(endNode);
      SideDependentList<FootstepNode> goalNodes = createGoalNodes(requestPacket);
      planner.initialize(startNode);
      stepValidityChecker.setParentNodeSupplier(planner.getGraph()::getParentNode);

      // Check valid goal
      if (!validGoal(requestPacket))
      {
         status = Status.INVALID_GOAL;
         isPlanning.set(false);
         reportStatus();
         return;
      }

      // Do planning
      stopwatch.start();
      reportStatus();
      while (true)
      {
         if(stopwatch.totalElapsed() >= requestPacket.getTimeout() || haltRequested.get())
         {
            status = Status.TIMED_OUT;
            break;
         }

         AStarIterationData<FootstepNode> iterationData = planner.doPlanningIteration();
         iterationCallback.accept(iterationData);

         if (iterationData.getParentNode() == null)
         {
            status = Status.NO_SOLUTION_EXISTS;
            break;
         }
         if (checkIfGoalIsReached(goalNodes, iterationData))
         {
            status = Status.FOUND_SOLUTION;
            break;
         }
         if (stopwatch.lapElapsed() > statusPublishPeriod)
         {
            reportStatus();
            stopwatch.lap();
         }
      }

      reportStatus();
      isPlanning.set(false);
   }

   private void reportStatus()
   {
      ValkyrieFootstepPlanningStatus planningStatus = new ValkyrieFootstepPlanningStatus();
      planningStatus.setPlanId(requestPacket.getPlannerRequestId());
      planningStatus.setPlannerStatus(status.toByte());

      // Pack solution path
      FootstepDataListMessage footstepDataList = planningStatus.getFootstepDataList();
      List<FootstepNode> path = planner.getGraph().getPathFromStart(endNode);
      for (int i = 1; i < path.size(); i++)
      {
         FootstepDataMessage footstepDataMessage = footstepDataList.getFootstepDataList().add();

         footstepDataMessage.setRobotSide(path.get(i).getRobotSide().toByte());

         RigidBodyTransform footstepPose = new RigidBodyTransform();
         footstepPose.setRotationYawAndZeroTranslation(path.get(i).getYaw());
         footstepPose.setTranslationX(path.get(i).getX());
         footstepPose.setTranslationY(path.get(i).getY());

         FootstepNodeSnapData snapData = snapAndWiggler.snapFootstepNode(path.get(i));
         RigidBodyTransform snapTransform = snapData.getSnapTransform();
         snapTransform.transform(footstepPose);
         footstepDataMessage.getLocation().set(footstepPose.getTranslation());
         footstepDataMessage.getOrientation().set(footstepPose.getRotation());
         
         if (requestPacket.getAssumeFlatGround() || requestPacket.getPlanarRegionsListMessage().getRegionId().isEmpty())
         {
            double flatGroundHeight = requestPacket.getStartLeftFootPose().getPosition().getZ();
            footstepDataMessage.getLocation().setZ(flatGroundHeight);            
         }
         
         if (sendCroppedFootholds)
         {
            ConvexPolygon2D foothold = snapData.getCroppedFoothold();
            if (!foothold.isEmpty())
            {
               for (int j = 0; j < foothold.getNumberOfVertices(); j++)
               {
                  footstepDataMessage.getPredictedContactPoints2d().add().set(foothold.getVertex(i));
               }
            }
         }
      }
      
      if (setSwingParameters)
      {
         setSwingParameters(footstepDataList);
      }

      statusCallback.accept(planningStatus);
   }
   
   private void setSwingParameters(FootstepDataListMessage footstepDataListMessage)
   {
      Object<FootstepDataMessage> footstepDataList = footstepDataListMessage.getFootstepDataList();
      for (int i = 0; i < footstepDataList.size(); i++)
      {
         FootstepDataMessage footstepDataMessage = footstepDataList.get(i);
         Pose3D startPose = new Pose3D();
         Pose3D endPose = new Pose3D();

         endPose.set(footstepDataMessage.getLocation(), footstepDataMessage.getOrientation());

         if(i < 2)
         {
            startPose.set(footstepDataMessage.getRobotSide() == RobotSide.LEFT.toByte() ? requestPacket.getStartLeftFootPose() : requestPacket.getStartRightFootPose());
         }
         else
         {
            FootstepDataMessage previousStep = footstepDataList.get(i - 2);
            startPose.set(previousStep.getLocation(), previousStep.getOrientation());
         }

         if(!swingParameterCalculator.checkForFootCollision(startPose, footstepDataMessage))
         {
            footstepDataMessage.setSwingHeight(swingParameterCalculator.calculateSwingHeight(startPose.getPosition(), endPose.getPosition()));            
         }
         
         footstepDataMessage.setSwingDuration(swingParameterCalculator.calculateSwingTime(startPose.getPosition(), endPose.getPosition()));
      }
   }

   private boolean checkIfGoalIsReached(SideDependentList<FootstepNode> goalNodes, AStarIterationData<FootstepNode> iterationData)
   {
      for (int i = 0; i < iterationData.getValidChildNodes().size(); i++)
      {
         FootstepNode childNode = iterationData.getValidChildNodes().get(i);
         if (childNode.equals(goalNodes.get(childNode.getRobotSide())))
         {
            endNode = goalNodes.get(childNode.getRobotSide().getOppositeSide());
            planner.getGraph().checkAndSetEdge(childNode, endNode, 0.0);
            return true;
         }

         double cost = planner.getGraph().getCostFromStart(childNode) + heuristics.compute(childNode);
         if(cost < endNodeCost)
         {
            endNode = childNode;
            endNodeCost = cost;
         }
      }
      return false;
   }

   private boolean validGoal(ValkyrieFootstepPlanningRequestPacket requestPacket)
   {
      boolean validLeftGoalStep = stepValidityChecker.checkFootstep(new FootstepNode(requestPacket.getGoalLeftFootPose().getX(),
                                                                                     requestPacket.getGoalLeftFootPose().getY(),
                                                                                     requestPacket.getGoalLeftFootPose().getYaw(),
                                                                                     RobotSide.LEFT), null);
      boolean validRightGoalStep = stepValidityChecker.checkFootstep(new FootstepNode(requestPacket.getGoalRightFootPose().getX(),
                                                                                      requestPacket.getGoalRightFootPose().getY(),
                                                                                      requestPacket.getGoalRightFootPose().getYaw(),
                                                                                      RobotSide.RIGHT), null);
      return validLeftGoalStep && validRightGoalStep;
   }

   private FootstepNode createStartNode(ValkyrieFootstepPlanningRequestPacket requestPacket)
   {
      Pose3D leftFootPose = requestPacket.getStartLeftFootPose();
      return new FootstepNode(leftFootPose.getPosition().getX(), leftFootPose.getPosition().getY(), leftFootPose.getOrientation().getYaw(), RobotSide.LEFT);
   }

   private SideDependentList<FootstepNode> createGoalNodes(ValkyrieFootstepPlanningRequestPacket requestPacket)
   {
      return new SideDependentList<>(side ->
                                     {
                                        Pose3D goalPose = new Pose3D(side.equals(RobotSide.LEFT) ? requestPacket.getGoalLeftFootPose() : requestPacket.getGoalRightFootPose());
                                        return new FootstepNode(goalPose.getX(), goalPose.getY(), goalPose.getYaw(), side);
                                     });
   }

   public static SideDependentList<ConvexPolygon2D> createFootPolygons(ValkyrieRobotModel robotModel)
   {
      RobotContactPointParameters<RobotSide> contactPointParameters = robotModel.getContactPointParameters();
      return new SideDependentList<>(side ->
                                     {
                                        ArrayList<Point2D> footPoints = contactPointParameters.getFootContactPoints().get(side);
                                        return new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(footPoints));
                                     });
   }

   public void halt()
   {
      haltRequested.set(true);
   }

   public ValkyrieAStarFootstepPlannerParameters getParameters()
   {
      return parameters;
   }

   public FootstepNodeSnapperReadOnly getSnapper()
   {
      return snapper;
   }

   public ValkyrieRobotModel getRobotModel()
   {
      return robotModel;
   }

   public void addRequestCallback(Consumer<ValkyrieFootstepPlanningRequestPacket> callback)
   {
      requestCallback = requestCallback.andThen(callback);
   }

   public void addIterationCallback(Consumer<AStarIterationData<FootstepNode>> callback)
   {
      iterationCallback = iterationCallback.andThen(callback);
   }

   public void addStatusCallback(Consumer<ValkyrieFootstepPlanningStatus> callback)
   {
      statusCallback = statusCallback.andThen(callback);
   }

   public void launchVisualizer()
   {
      ApplicationRunner.runApplication(new ValkyrieFootstepPlannerUI(this));
   }

   public void setupWithRos(PubSubImplementation pubSubImplementation)
   {
      if(ros2Node != null)
         return;

      ros2Node = ROS2Tools.createRos2Node(pubSubImplementation, MODULE_NAME);
      MessageTopicNameGenerator inputTopicNameGenerator = ROS2Tools.getTopicNameGenerator(robotModel.getSimpleRobotName(), MODULE_NAME, ROS2TopicQualifier.INPUT);
      MessageTopicNameGenerator outputTopicNameGenerator = ROS2Tools.getTopicNameGenerator(robotModel.getSimpleRobotName(), MODULE_NAME, ROS2TopicQualifier.OUTPUT);

      IHMCROS2Publisher<ValkyrieFootstepPlanningStatus> statusPublisher = ROS2Tools.createPublisher(ros2Node,
                                                                                                    ValkyrieFootstepPlanningStatus.class,
                                                                                                    outputTopicNameGenerator);
      IHMCROS2Publisher<ValkyrieFootstepPlannerParametersPacket> parametersPublisher = ROS2Tools.createPublisher(ros2Node,
                                                                                                                 ValkyrieFootstepPlannerParametersPacket.class,
                                                                                                                 outputTopicNameGenerator);
      addStatusCallback(statusPublisher::publish);

      ROS2Tools.createCallbackSubscription(ros2Node, ValkyrieFootstepPlanningRequestPacket.class, inputTopicNameGenerator, s ->
      {
         ValkyrieFootstepPlanningRequestPacket requestPacket = s.takeNextData();
         new Thread(() -> handleRequestPacket(requestPacket)).start();
      });

      ROS2Tools.createCallbackSubscription(ros2Node, ValkyrieFootstepPlanningActionPacket.class, inputTopicNameGenerator, s ->
      {
         ValkyrieFootstepPlanningActionPacket packet = s.takeNextData();
         switch(Action.fromByte(packet.getPlannerAction()))
         {
            case HALT:
               halt();
               break;
            case REQUEST_PARAMETERS:
               ValkyrieFootstepPlannerParametersPacket parametersPacket = new ValkyrieFootstepPlannerParametersPacket();
               parameters.setPacket(parametersPacket);
               parametersPublisher.publish(parametersPacket);
               break;
         }
      });
   }

   public void closeAndDispose()
   {
      if(ros2Node != null)
      {
         ros2Node.destroy();
         ros2Node = null;
      }
   }

   public enum Status
   {
      PLANNING, FOUND_SOLUTION, TIMED_OUT, NO_SOLUTION_EXISTS, INVALID_GOAL;

      public static Status fromByte(byte value)
      {
         return values()[(int) value];
      }

      public byte toByte()
      {
         return (byte) ordinal();
      }
   }

   public enum Action
   {
      HALT, REQUEST_PARAMETERS;

      public static Action fromByte(byte value)
      {
         return values()[(int) value];
      }

      public byte toByte()
      {
         return (byte) ordinal();
      }
   }

   public static void main(String[] args)
   {
      // match controller version by default
      ValkyrieRobotVersion version = ValkyrieRosControlController.VERSION;

      ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.REAL_ROBOT, version);
      new ValkyrieAStarFootstepPlanner(robotModel);
   }
}
