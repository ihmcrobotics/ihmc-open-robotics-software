package us.ihmc.valkyrie.planner;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.Consumer;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import controller_msgs.msg.dds.ValkyrieFootstepPlannerParametersPacket;
import controller_msgs.msg.dds.ValkyrieFootstepPlanningActionPacket;
import controller_msgs.msg.dds.ValkyrieFootstepPlanningRequestPacket;
import controller_msgs.msg.dds.ValkyrieFootstepPlanningStatus;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.footstepPlanning.AdaptiveSwingTrajectoryCalculator;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ROS2Tools.MessageTopicNameGenerator;
import us.ihmc.communication.ROS2Tools.ROS2TopicQualifier;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapAndWiggler;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapData;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapper;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnappingTools;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.SimplePlanarRegionFootstepNodeSnapper;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.nodeExpansion.ParameterBasedNodeExpansion;
import us.ihmc.idl.IDLSequence.Object;
import us.ihmc.javaFXToolkit.starter.ApplicationRunner;
import us.ihmc.log.LogTools;
import us.ihmc.pathPlanning.graph.search.AStarIterationData;
import us.ihmc.pathPlanning.graph.search.AStarPathPlanner;
import us.ihmc.pathPlanning.graph.structure.GraphEdge;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.configuration.ValkyrieRobotVersion;
import us.ihmc.valkyrie.parameters.ValkyrieAdaptiveSwingParameters;
import us.ihmc.valkyrie.planner.log.ValkyriePlannerEdgeData;
import us.ihmc.valkyrie.planner.log.ValkyriePlannerIterationData;
import us.ihmc.valkyrie.planner.ui.ValkyrieFootstepPlannerUI;
import us.ihmc.valkyrieRosControl.ValkyrieRosControlController;
import us.ihmc.wholeBodyController.RobotContactPointParameters;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

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
   private final ValkyrieFootstepPlannerHeuristics heuristics;
   private final AdaptiveSwingTrajectoryCalculator swingParameterCalculator;
   private final ValkyrieStepCost stepCost;
   private final BodyPathHelper bodyPathHelper = new BodyPathHelper(parameters);
   private final ValkyrieIdealStepCalculator idealStepCalculator;

   private final ValkyriePlannerEdgeData edgeData = new ValkyriePlannerEdgeData();
   private final HashMap<GraphEdge<FootstepNode>, ValkyriePlannerEdgeData> edgeDataMap = new HashMap<>();
   private final List<ValkyriePlannerIterationData> iterationData = new ArrayList<>();

   private Status status = null;
   private FootstepNode endNode = null;
   private double endNodeCost;

   private Consumer<ValkyrieFootstepPlanningRequestPacket> requestCallback = request -> {};
   private Consumer<AStarIterationData<FootstepNode>> iterationCallback = iterationData -> {};
   private Consumer<ValkyrieFootstepPlanningStatus> statusCallback = result -> {};

   private final AtomicBoolean isPlanning = new AtomicBoolean();
   private final AtomicBoolean haltRequested = new AtomicBoolean();
   private final ValkyrieFootstepPlanningRequestPacket requestPacket = new ValkyrieFootstepPlanningRequestPacket();
   private final ValkyrieFootstepPlanningStatus planningStatus = new ValkyrieFootstepPlanningStatus();
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

      stepValidityChecker = new ValkyrieFootstepValidityChecker(parameters, footPolygons, snapper, edgeData);
      idealStepCalculator = new ValkyrieIdealStepCalculator(parameters, bodyPathHelper, stepValidityChecker);
      heuristics = new ValkyrieFootstepPlannerHeuristics(parameters, bodyPathHelper, edgeData);
      stepCost = new ValkyrieStepCost(parameters, snapper, heuristics, idealStepCalculator::computeIdealStep, footPolygons, edgeData);

      planner = new AStarPathPlanner<>(nodeExpansion::expandNode, stepValidityChecker::checkFootstep, stepCost::compute, heuristics::compute);
      planner.getGraph().setGraphExpansionCallback(edge ->
                                                   {
                                                      edgeData.setCostFromStart(planner.getGraph().getCostFromStart(edge.getEndNode()));
                                                      edgeDataMap.put(edge, edgeData.getCopyAndClear());
                                                   });
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

      requestCallback.accept(requestPacket);
      bodyPathHelper.initialize(requestPacket);
      idealStepCalculator.initialize();
      edgeData.clear();
      edgeDataMap.clear();
      iterationData.clear();

      // update parameters
      parameters.setFromPacket(requestPacket.getParameters());

      // update planar regions
      PlanarRegionsList planarRegionsList = null;
      if (!requestPacket.getAssumeFlatGround() && !requestPacket.getPlanarRegionsListMessage().getRegionId().isEmpty())
      {
         planarRegionsList = PlanarRegionMessageConverter.convertToPlanarRegionsList(requestPacket.getPlanarRegionsListMessage());
      }

      snapper.setPlanarRegions(planarRegionsList);
      snapAndWiggler.setPlanarRegions(planarRegionsList);
      stepValidityChecker.setPlanarRegionsList(planarRegionsList);
      swingParameterCalculator.setPlanarRegionsList(planarRegionsList);

      // Set up planner
      FootstepNode startNode = createStartNode(requestPacket);
      addStartPosesToSnapper(requestPacket);
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
         recordIterationData(iterationData);

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
         if (stopwatch.lapElapsed() > statusPublishPeriod && !MathTools.epsilonEquals(stopwatch.totalElapsed(), requestPacket.getTimeout(), 0.1))
         {
            reportStatus();
            stopwatch.lap();
         }
      }

      reportStatus();
      markSolutionEdges();
      isPlanning.set(false);
   }

   private void recordIterationData(AStarIterationData<FootstepNode> iterationData)
   {
      ValkyriePlannerIterationData loggedData = new ValkyriePlannerIterationData();
      loggedData.setStanceNode(iterationData.getParentNode());
      iterationData.getValidChildNodes().forEach(loggedData::addChildNode);
      iterationData.getInvalidChildNodes().forEach(loggedData::addChildNode);
      loggedData.setIdealStep(idealStepCalculator.computeIdealStep(iterationData.getParentNode()));
      loggedData.setStanceNodeSnapData(snapper.getSnapData(iterationData.getParentNode()));
      this.iterationData.add(loggedData);
   }

   private void reportStatus()
   {
      planningStatus.setPlanId(requestPacket.getPlannerRequestId());
      planningStatus.setPlannerStatus(status.toByte());

      // Pack solution path
      FootstepDataListMessage footstepDataList = planningStatus.getFootstepDataList();
      footstepDataList.getFootstepDataList().clear();
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

   private void markSolutionEdges()
   {
      List<FootstepNode> path = planner.getGraph().getPathFromStart(endNode);
      for (int i = 1; i < path.size(); i++)
      {
         edgeDataMap.get(new GraphEdge<>(path.get(i - 1), path.get(i))).setSolutionEdge(true);
      }
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

   private void addStartPosesToSnapper(ValkyrieFootstepPlanningRequestPacket requestPacket)
   {
      Pose3D leftFootPose = requestPacket.getStartLeftFootPose();
      Pose3D rightFootPose = requestPacket.getStartRightFootPose();

      FootstepNode leftFootNode = new FootstepNode(leftFootPose.getX(), leftFootPose.getY(), leftFootPose.getYaw(), RobotSide.LEFT);
      FootstepNode rightFootNode = new FootstepNode(rightFootPose.getX(), rightFootPose.getY(), rightFootPose.getYaw(), RobotSide.RIGHT);

      RigidBodyTransform leftFootSnapTransform = FootstepNodeSnappingTools.computeSnapTransform(leftFootNode, leftFootPose);
      RigidBodyTransform rightFootSnapTransform = FootstepNodeSnappingTools.computeSnapTransform(rightFootNode, rightFootPose);

      snapper.addSnapData(leftFootNode, new FootstepNodeSnapData(leftFootSnapTransform));
      snapper.addSnapData(rightFootNode, new FootstepNodeSnapData(rightFootSnapTransform));
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

   public FootstepNodeSnapper getSnapper()
   {
      return snapper;
   }

   public ValkyrieRobotModel getRobotModel()
   {
      return robotModel;
   }

   public AStarPathPlanner<FootstepNode> getInternalPlanner()
   {
      return planner;
   }

   public ValkyrieFootstepPlannerHeuristics getHeuristics()
   {
      return heuristics;
   }

   public FootstepNode getEndNode()
   {
      return endNode;
   }

   public HashMap<GraphEdge<FootstepNode>, ValkyriePlannerEdgeData> getEdgeDataMap()
   {
      return edgeDataMap;
   }

   public List<ValkyriePlannerIterationData> getIterationData()
   {
      return iterationData;
   }

   public ValkyrieFootstepPlanningRequestPacket getRequestPacket()
   {
      return requestPacket;
   }

   public ValkyrieFootstepPlanningStatus getPlanningStatus()
   {
      return planningStatus;
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
