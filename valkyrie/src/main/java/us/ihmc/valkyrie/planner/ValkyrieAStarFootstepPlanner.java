package us.ihmc.valkyrie.planner;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import controller_msgs.msg.dds.ValkyrieFootstepPlanningStatus;
import controller_msgs.msg.dds.ValkyrieFootstepPlanningRequestPacket;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.commons.time.Stopwatch;
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
import us.ihmc.log.LogTools;
import us.ihmc.pathPlanning.graph.search.AStarIterationData;
import us.ihmc.pathPlanning.graph.search.AStarPathPlanner;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.configuration.ValkyrieRobotVersion;
import us.ihmc.valkyrieRosControl.ValkyrieRosControlController;
import us.ihmc.wholeBodyController.RobotContactPointParameters;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.Consumer;

public class ValkyrieAStarFootstepPlanner
{
   private static final boolean sendCroppedFootholds = false;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final ValkyrieRobotModel robotModel;
   private final ValkyrieAStarFootstepPlannerParameters parameters = new ValkyrieAStarFootstepPlannerParameters(registry);
   private final AStarPathPlanner<FootstepNode> planner;
   private final SimplePlanarRegionFootstepNodeSnapper snapper;
   private final FootstepNodeSnapAndWiggler snapAndWiggler;
   private final ValkyrieFootstepValidityChecker stepValidityChecker;
   private final DistanceAndYawBasedHeuristics heuristics;

   private Status status = null;
   private FootstepNode endNode = null;

   private Consumer<ValkyrieFootstepPlanningRequestPacket> requestCallback = request -> {};
   private Consumer<AStarIterationData<FootstepNode>> iterationCallback = iterationData -> {};
   private Consumer<ValkyrieFootstepPlanningStatus> resultCallback = result -> {};

   private final AtomicBoolean isPlanning = new AtomicBoolean();
   private final AtomicBoolean haltRequested = new AtomicBoolean();
   private final Stopwatch stopwatch = new Stopwatch();

   public ValkyrieAStarFootstepPlanner(ValkyrieRobotModel robotModel)
   {
      this.robotModel = robotModel;
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
   }

   public void handleRequestPacket(ValkyrieFootstepPlanningRequestPacket requestPacket)
   {
      if (isPlanning.get())
      {
         LogTools.info("Received planning request packet but planner is currently running");
         return;
      }

      isPlanning.set(true);
      haltRequested.set(false);
      endNode = null;
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
      reportStatus(requestPacket);
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
         if (stopwatch.lapElapsed() > 2.0)
         {
            reportStatus(requestPacket);
            stopwatch.lap();
         }
      }

      reportStatus(requestPacket);
      isPlanning.set(false);
   }

   private void reportStatus(ValkyrieFootstepPlanningRequestPacket requestPacket)
   {
      ValkyrieFootstepPlanningStatus planningStatus = new ValkyrieFootstepPlanningStatus();
      planningStatus.setPlanId(requestPacket.getPlannerRequestId());
      planningStatus.setPlannerStatus(status.toByte());

      if (endNode != null)
      {
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

      resultCallback.accept(planningStatus);
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

   public void addResultCallback(Consumer<ValkyrieFootstepPlanningStatus> callback)
   {
      resultCallback = resultCallback.andThen(callback);
   }

   /* package private */
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

   public static void main(String[] args)
   {
      // match controller version by default
      ValkyrieRobotVersion version = ValkyrieRosControlController.VERSION;

      ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.REAL_ROBOT, version);
      new ValkyrieAStarFootstepPlanner(robotModel);
   }
}
