package us.ihmc.humanoidBehaviors.behaviors.roughTerrain;

import us.ihmc.commons.PrintTools;
import us.ihmc.communication.packets.*;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.footstepPlanning.*;
import us.ihmc.footstepPlanning.graphSearch.YoFootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.planners.VisibilityGraphWithAStarPlanner;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridge;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridgeInterface;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.WalkOverTerrainGoalPacket;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessageConverter;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepPlanningRequestPacket;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepPlanningToolboxOutputStatus;
import us.ihmc.robotEnvironmentAwareness.communication.converters.REAPlanarRegionsConverter;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.MovingReferenceFrame;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.State;
import us.ihmc.wholeBodyController.RobotContactPointParameters;
import us.ihmc.wholeBodyController.WholeBodyControllerParameters;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicReference;

class PlanFootstepsState extends State<WalkOverTerrainStateMachineBehavior.WalkOverTerrainState>
{
   private static final boolean debug = true;
   private static final boolean useToolbox = true;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final YoInteger planId = new YoInteger("PlanId", registry);
   private final SideDependentList<MovingReferenceFrame> soleFrames;

   private final AtomicReference<FramePose3D> goalPose = new AtomicReference<>();
   private final AtomicReference<PlanarRegionsListMessage> planarRegionsListMessage = new AtomicReference<>();
   private final AtomicReference<FootstepPlanningToolboxOutputStatus> plannerOutputStatus = new AtomicReference<>();

   private final YoEnum<RobotSide> nextSideToSwing = new YoEnum<>("nextSideToSwing", registry, RobotSide.class, false);
   private final YoBoolean plannerRequestHasBeenSent = new YoBoolean("plannerRequestHasBeenSent", registry);
   private final DoubleProvider swingTime;

   private final VisibilityGraphWithAStarPlanner planner;

   private final CommunicationBridgeInterface communicationBridge;

   PlanFootstepsState(CommunicationBridgeInterface communicationBridge, SideDependentList<MovingReferenceFrame> soleFrames, WholeBodyControllerParameters wholeBodyControllerParameters, DoubleProvider swingTime, YoVariableRegistry parentRegistry)
   {
      super(WalkOverTerrainStateMachineBehavior.WalkOverTerrainState.PLAN_FOOTSTEPS);
      this.soleFrames = soleFrames;
      this.communicationBridge = communicationBridge;
      this.swingTime = swingTime;

      communicationBridge.attachListener(WalkOverTerrainGoalPacket.class, (packet) ->
      {
         goalPose.set(new FramePose3D(ReferenceFrame.getWorldFrame(), packet.position, packet.orientation));
         PrintTools.info("received goal pose");
      });

      communicationBridge.attachListener(PlanarRegionsListMessage.class, planarRegionsListMessage::set);
      communicationBridge.attachListener(FootstepPlanningToolboxOutputStatus.class, plannerOutputStatus::set);

      nextSideToSwing.set(RobotSide.LEFT);

      SideDependentList<ConvexPolygon2D> contactPoints = createFootPolygonsFromContactPoints(wholeBodyControllerParameters.getContactPointParameters());
      planner = new VisibilityGraphWithAStarPlanner(new YoFootstepPlannerParameters(registry, new DefaultFootstepPlanningParameters()), contactPoints, null, registry);

      parentRegistry.addChild(registry);
   }

   @Override
   public void doTransitionIntoAction()
   {
      if(debug)
         PrintTools.info("Entering plan state");

      plannerRequestHasBeenSent.set(false);
      plannerOutputStatus.set(null);
      requestPlanarRegionsList();
   }

   private void requestPlanarRegionsList()
   {
      RequestPlanarRegionsListMessage requestPlanarRegionsListMessage = new RequestPlanarRegionsListMessage(RequestPlanarRegionsListMessage.RequestType.SINGLE_UPDATE);
      requestPlanarRegionsListMessage.setDestination(PacketDestination.REA_MODULE);
      communicationBridge.sendPacket(requestPlanarRegionsListMessage);
   }

   @Override
   public void doAction()
   {
      if (goalPose.get() == null)
      {
         return;
      }

      if (planarRegionsListMessage.get() == null)
      {
         requestPlanarRegionsList();
         return;
      }

      if (useToolbox)
      {
         if(!plannerRequestHasBeenSent.getBooleanValue())
         {
            ToolboxStateMessage wakeUp = new ToolboxStateMessage(ToolboxStateMessage.ToolboxState.WAKE_UP);
            wakeUp.setDestination(PacketDestination.FOOTSTEP_PLANNING_TOOLBOX_MODULE);
            communicationBridge.sendPacket(wakeUp);

            FootstepPlanningRequestPacket planningRequestPacket = createPlanningRequestPacket();
            planningRequestPacket.setDestination(PacketDestination.FOOTSTEP_PLANNING_TOOLBOX_MODULE);
            communicationBridge.sendPacket(planningRequestPacket);

            plannerRequestHasBeenSent.set(true);
         }
      }
      else
      {
         PrintTools.info("setting up planner...");
         planner.setPlanarRegions(PlanarRegionMessageConverter.convertToPlanarRegionsList(planarRegionsListMessage.get()));
         PrintTools.info("setting swing time...");
         planner.setTimeout(swingTime.getValue() - 0.25);

         FootstepPlannerGoal goal = new FootstepPlannerGoal();
         FramePose3D goalPose = this.goalPose.get();
         goalPose.changeFrame(ReferenceFrame.getWorldFrame());
         goal.setGoalPoseBetweenFeet(goalPose);
         goal.setFootstepPlannerGoalType(FootstepPlannerGoalType.POSE_BETWEEN_FEET);
         PrintTools.info("setting goal...");
         planner.setGoal(goal);

         RobotSide stanceSide = nextSideToSwing.getValue().getOppositeSide();
         FramePose3D stanceFootPose = new FramePose3D(soleFrames.get(stanceSide));
         stanceFootPose.changeFrame(ReferenceFrame.getWorldFrame());
         PrintTools.info("setting initial stance foot...");
         planner.setInitialStanceFoot(stanceFootPose, stanceSide);

         PrintTools.info("starting to plan...");
         FootstepPlanningResult result = planner.plan();
         PrintTools.info("finished planning. result: " + result);
         FootstepPlan plan = planner.getPlan();

         FootstepPlanningToolboxOutputStatus outputStatus = new FootstepPlanningToolboxOutputStatus();
         outputStatus.planningResult = result;
         if(result.validForExecution())
         {
            nextSideToSwing.set(stanceSide);
            outputStatus.footstepDataList = FootstepDataMessageConverter.createFootstepDataListFromPlan(plan, 0.0, 0.0, ExecutionMode.OVERRIDE);
         }

         this.plannerOutputStatus.set(outputStatus);
      }
   }

   @Override
   public void doTransitionOutOfAction()
   {
   }

   FootstepPlanningToolboxOutputStatus getPlannerOutput()
   {
      return plannerOutputStatus.get();
   }

   private FootstepPlanningRequestPacket createPlanningRequestPacket()
   {
      planId.increment();
      FramePose3D goalPose = this.goalPose.get();
      goalPose.changeFrame(ReferenceFrame.getWorldFrame());
      RobotSide stanceSide = nextSideToSwing.getValue().getOppositeSide();

      FramePose3D stanceFootPose = new FramePose3D(soleFrames.get(stanceSide));
      stanceFootPose.changeFrame(ReferenceFrame.getWorldFrame());

      FootstepPlanningRequestPacket planningRequestPacket = new FootstepPlanningRequestPacket(stanceFootPose, stanceSide, goalPose, FootstepPlannerType.VIS_GRAPH_WITH_A_STAR);
      planningRequestPacket.planarRegionsListMessage = planarRegionsListMessage.get();
      planningRequestPacket.timeout = swingTime.getValue() - 0.25;
      planningRequestPacket.planId = planId.getIntegerValue();

      nextSideToSwing.set(stanceSide);

      return planningRequestPacket;
   }

   private static SideDependentList<ConvexPolygon2D> createFootPolygonsFromContactPoints(RobotContactPointParameters contactPointParameters)
   {
      SideDependentList<ConvexPolygon2D> footPolygons = new SideDependentList<>();
      for (RobotSide side : RobotSide.values)
      {
         ArrayList<Point2D> footPoints = contactPointParameters.getFootContactPoints().get(side);
         ConvexPolygon2D scaledFoot = new ConvexPolygon2D(footPoints);
         footPolygons.set(side, scaledFoot);
      }

      return footPolygons;
   }
}
