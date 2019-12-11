package us.ihmc.valkyrie.planner;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import controller_msgs.msg.dds.ValkyrieFootstepPlanningRequestPacket;
import controller_msgs.msg.dds.ValkyrieFootstepPlanningResult;
import us.ihmc.avatar.drcRobot.RobotTarget;
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
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapAndWiggler;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapData;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.SimplePlanarRegionFootstepNodeSnapper;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.heuristics.DistanceAndYawBasedHeuristics;
import us.ihmc.footstepPlanning.graphSearch.nodeExpansion.ParameterBasedNodeExpansion;
import us.ihmc.footstepPlanning.graphSearch.stepCost.*;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPolygon;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.log.LogTools;
import us.ihmc.pathPlanning.graph.search.AStarIterationData;
import us.ihmc.pathPlanning.graph.search.AStarPathPlanner;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.configuration.ValkyrieRobotVersion;
import us.ihmc.valkyrieRosControl.ValkyrieRosControlController;
import us.ihmc.wholeBodyController.RobotContactPointParameters;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFramePose3D;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;

public class ValkyrieAStarFootstepPlanner
{
   public static final String MODULE_NAME = "valkyrie_footstep_planner";
   public static final boolean sendCroppedFootholds = false;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final YoFramePose3D footPose = new YoFramePose3D("footPose", ReferenceFrame.getWorldFrame(), registry);
   private final YoGraphicPolygon footPolygon = new YoGraphicPolygon("footPolygon", footPose, 4, registry, 1.0, YoAppearance.Green());
   private final YoGraphicPosition[] candidateSteps = new YoGraphicPosition[300];
   private final YoGraphicPolygon[] solutionSteps = new YoGraphicPolygon[100];
   private final SideDependentList<YoGraphicPolygon> startSteps = new SideDependentList<>(side -> new YoGraphicPolygon(side.getLowerCaseName() + "StartStep", 4, registry, true, 1.0, YoAppearance.ForestGreen()));
   private final SideDependentList<YoGraphicPolygon> goalSteps = new SideDependentList<>(side -> new YoGraphicPolygon(side.getLowerCaseName() + "GoalStep", 4, registry, true, 1.0, YoAppearance.DarkRed()));

   private final Ros2Node rosNode = ROS2Tools.createRos2Node(PubSubImplementation.FAST_RTPS, MODULE_NAME);
   private final MessageTopicNameGenerator inputTopicNameGenerator;
   private final MessageTopicNameGenerator outputTopicNameGenerator;
   private final IHMCROS2Publisher<ValkyrieFootstepPlanningResult> resultPublisher;

   private final ValkyrieAStarFootstepPlannerParameters parameters = new ValkyrieAStarFootstepPlannerParameters(registry);
   private final AStarPathPlanner<FootstepNode> planner;
   private final SimplePlanarRegionFootstepNodeSnapper snapper;
   private final FootstepNodeSnapAndWiggler snapAndWiggler;
   private final ValkyrieFootstepValidityChecker stepValidityChecker;
   private final DistanceAndYawBasedHeuristics heuristics;
   private FootstepNode endNode = null;

   private final AtomicBoolean isPlanning = new AtomicBoolean();
   private final Stopwatch stopwatch = new Stopwatch();

   public ValkyrieAStarFootstepPlanner(ValkyrieRobotModel robotModel, YoGraphicsListRegistry graphicsListRegistry)
   {
      this.inputTopicNameGenerator = ROS2Tools.getTopicNameGenerator(robotModel.getSimpleRobotName(), MODULE_NAME, ROS2TopicQualifier.INPUT);
      this.outputTopicNameGenerator = ROS2Tools.getTopicNameGenerator(robotModel.getSimpleRobotName(), MODULE_NAME, ROS2TopicQualifier.OUTPUT);

      SideDependentList<ConvexPolygon2D> footPolygons = createFootPolygons(robotModel);
      snapper = new SimplePlanarRegionFootstepNodeSnapper(footPolygons);
      snapAndWiggler  = new FootstepNodeSnapAndWiggler(footPolygons, () -> true, parameters::getWiggleInsideDelta, parameters::getMaximumXYWiggle, parameters::getMaximumYawWiggle, () -> Double.POSITIVE_INFINITY);

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

      ROS2Tools.createCallbackSubscription(rosNode,
                                           ValkyrieFootstepPlanningRequestPacket.class,
                                           inputTopicNameGenerator,
                                           s -> handleRequestPacket(s.takeNextData()));
      resultPublisher = ROS2Tools.createPublisher(rosNode, ValkyrieFootstepPlanningResult.class, outputTopicNameGenerator);

      if (graphicsListRegistry != null)
      {
         footPolygon.updateConvexPolygon2d(footPolygons.get(RobotSide.LEFT));
         YoGraphicsList graphicsList = new YoGraphicsList(MODULE_NAME);
         graphicsList.add(footPolygon);

         for (int i = 0; i < candidateSteps.length; i++)
         {
            candidateSteps[i] = new YoGraphicPosition("candidateStep" + i, "", registry, 0.01, YoAppearance.White());
            graphicsList.add(candidateSteps[i]);
         }

         for (int i = 0; i < solutionSteps.length; i++)
         {
            solutionSteps[i] = new YoGraphicPolygon("solutionStep" + i, 4, registry, true, 1.0, YoAppearance.Green());
            solutionSteps[i].updateConvexPolygon2d(footPolygons.get(RobotSide.LEFT));
            solutionSteps[i].setPoseToNaN();
            graphicsList.add(solutionSteps[i]);
         }

         startSteps.forEach((side, step) ->
                            {
                               graphicsList.add(step);
                               step.updateConvexPolygon2d(footPolygons.get(side));
                               step.setPoseToNaN();
                            });

         goalSteps.forEach((side, step) ->
                            {
                               graphicsList.add(step);
                               step.updateConvexPolygon2d(footPolygons.get(side));
                               step.setPoseToNaN();
                            });

         graphicsListRegistry.registerYoGraphicsList(graphicsList);
      }
   }

   public ValkyrieFootstepPlanningResult handleRequestPacket(ValkyrieFootstepPlanningRequestPacket requestPacket)
   {
      if (isPlanning.get())
      {
         LogTools.info("Received planning request packet but planner is currently running");
         return null;
      }

      isPlanning.set(true);
      endNode = null;

      initializeGraphics(requestPacket);
      heuristics.setGoalPose(new FramePose3D(requestPacket.getGoalPoses().get(0)));

      // update parameters
      parameters.setFromPacket(requestPacket.getParameters());

      // update planar regions
      if (requestPacket.getAssumeFlatGround() || requestPacket.getPlanarRegionsListMessage().getRegionId().isEmpty())
      {
         snapper.setPlanarRegions(null);
         stepValidityChecker.setPlanarRegionsList(null);
      }
      else
      {
         PlanarRegionsList planarRegionsList = PlanarRegionMessageConverter.convertToPlanarRegionsList(requestPacket.getPlanarRegionsListMessage());
         snapper.setPlanarRegions(planarRegionsList);
         snapAndWiggler.setPlanarRegions(planarRegionsList);
         stepValidityChecker.setPlanarRegionsList(planarRegionsList);
      }

      // Set up planner
      FootstepNode startNode = createStartNode(requestPacket);
      SideDependentList<FootstepNode> goalNodes = createGoalNodes(requestPacket);
      planner.initialize(startNode);
      stepValidityChecker.setParentNodeSupplier(planner.getGraph()::getParentNode);

      // Do planning
      stopwatch.start();
      while (stopwatch.totalElapsed() < requestPacket.getTimeout())
      {
         AStarIterationData<FootstepNode> iterationData = planner.doPlanningIteration();

         if (iterationData.getParentNode() == null)
            break;
         if (checkIfGoalIsReached(goalNodes, iterationData))
            break;

         updateGraphics(iterationData);
      }

      // Send result
      ValkyrieFootstepPlanningResult planningResult = new ValkyrieFootstepPlanningResult();
      planningResult.setPlanId(requestPacket.getPlannerRequestId());
      planningResult.getPlanarRegionsList().set(requestPacket.getPlanarRegionsListMessage());

      if (endNode != null)
      {
         FootstepDataListMessage footstepDataList = planningResult.getFootstepDataList();
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

            if(sendCroppedFootholds)
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
      }

      setSolutionGraphics(planningResult);
      resultPublisher.publish(planningResult);
      return planningResult;
   }

   private void initializeGraphics(ValkyrieFootstepPlanningRequestPacket requestPacket)
   {
      startSteps.get(RobotSide.LEFT).setPose(new FramePose3D(requestPacket.getLeftFootPose()));
      startSteps.get(RobotSide.RIGHT).setPose(new FramePose3D(requestPacket.getRightFootPose()));
      FramePose3D goalStep = new FramePose3D(requestPacket.getGoalPoses().get(0));
      goalStep.appendTranslation(0.0, 0.5 * parameters.getIdealFootstepWidth(), 0.0);
      goalSteps.get(RobotSide.LEFT).setPose(goalStep);
      goalStep.appendTranslation(0.0, -parameters.getIdealFootstepWidth(), 0.0);
      goalSteps.get(RobotSide.RIGHT).setPose(goalStep);
   }

   private void updateGraphics(AStarIterationData<FootstepNode> iterationData)
   {
      footPose.set(iterationData.getParentNode().getX(), iterationData.getParentNode().getY(), 0.0, iterationData.getParentNode().getYaw(), 0.0, 0.0);

      for (int i = 0; i < candidateSteps.length; i++)
      {
         if (i < iterationData.getValidChildNodes().size())
            candidateSteps[i].setPosition(iterationData.getValidChildNodes().get(i).getX(), iterationData.getValidChildNodes().get(i).getY(), 0.0);
         else
            candidateSteps[i].setPositionToNaN();
      }
   }

   private void setSolutionGraphics(ValkyrieFootstepPlanningResult planningResult)
   {
      for (int i = 0; i < planningResult.getFootstepDataList().getFootstepDataList().size(); i++)
      {
         FootstepDataMessage footstepDataMessage = planningResult.getFootstepDataList().getFootstepDataList().get(i);
         solutionSteps[i].setPose(new FramePose3D(ReferenceFrame.getWorldFrame(), footstepDataMessage.getLocation(), footstepDataMessage.getOrientation()));
      }

      for (int i = 0; i < candidateSteps.length; i++)
      {
         candidateSteps[i].setPositionToNaN();
      }

      footPose.setToNaN();
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
      }
      return false;
   }

   private FootstepNode createStartNode(ValkyrieFootstepPlanningRequestPacket requestPacket)
   {
      Pose3D leftFootPose = requestPacket.getLeftFootPose();
      return new FootstepNode(leftFootPose.getPosition().getX(), leftFootPose.getPosition().getY(), leftFootPose.getOrientation().getYaw(), RobotSide.LEFT);
   }

   private SideDependentList<FootstepNode> createGoalNodes(ValkyrieFootstepPlanningRequestPacket requestPacket)
   {
      return new SideDependentList<>(side ->
                                     {
                                        Pose3D goalPose = new Pose3D(requestPacket.getGoalPoses().get(0));
                                        goalPose.appendTranslation(0.0, side.negateIfRightSide(0.5 * parameters.getIdealFootstepWidth()), 0.0);
                                        return new FootstepNode(goalPose.getX(), goalPose.getY(), goalPose.getYaw(), side);
                                     });
   }

   private static SideDependentList<ConvexPolygon2D> createFootPolygons(ValkyrieRobotModel robotModel)
   {
      RobotContactPointParameters<RobotSide> contactPointParameters = robotModel.getContactPointParameters();
      return new SideDependentList<>(side ->
                                     {
                                        ArrayList<Point2D> footPoints = contactPointParameters.getFootContactPoints().get(side);
                                        return new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(footPoints));
                                     });
   }

   public static void main(String[] args)
   {
      // match controller version by default
      ValkyrieRobotVersion version = ValkyrieRosControlController.VERSION;

      ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.REAL_ROBOT, version);
      new ValkyrieAStarFootstepPlanner(robotModel, null);
   }
}
