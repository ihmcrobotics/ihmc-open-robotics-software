package us.ihmc.humanoidBehaviors.behaviors.roughTerrain;

import us.ihmc.commons.PrintTools;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.PlanarRegionsListMessage;
import us.ihmc.communication.packets.ToolboxStateMessage;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.footstepPlanning.FootstepPlannerType;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridge;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.WalkOverTerrainGoalPacket;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepPlanningRequestPacket;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepPlanningToolboxOutputStatus;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.MovingReferenceFrame;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.State;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.concurrent.atomic.AtomicReference;

class PlanFootstepsState extends State<WalkOverTerrainStateMachineBehavior.WalkOverTerrainState>
{
   private static final boolean debug = true;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final YoInteger planId = new YoInteger("PlanId", registry);
   private final SideDependentList<MovingReferenceFrame> soleFrames;

   private final AtomicReference<FramePose3D> goalPose = new AtomicReference<>();
   private final AtomicReference<PlanarRegionsListMessage> planarRegionsListMessage = new AtomicReference<>();
   private final AtomicReference<FootstepPlanningToolboxOutputStatus> plannerOutputStatus = new AtomicReference<>();

   private final YoEnum<RobotSide> nextSideToSwing = new YoEnum<>("nextSideToSwing", registry, RobotSide.class, false);
   private final YoBoolean plannerRequestHasBeenSent = new YoBoolean("plannerRequestHasBeenSent", registry);
   private final DoubleProvider swingTime;

   private final CommunicationBridge communicationBridge;

   PlanFootstepsState(CommunicationBridge communicationBridge, SideDependentList<MovingReferenceFrame> soleFrames, DoubleProvider swingTime, YoVariableRegistry parentRegistry)
   {
      super(WalkOverTerrainStateMachineBehavior.WalkOverTerrainState.PLAN_FOOTSTEPS);
      this.soleFrames = soleFrames;
      this.communicationBridge = communicationBridge;
      this.swingTime = swingTime;

      communicationBridge.attachListener(WalkOverTerrainGoalPacket.class, (packet) ->
      {
         goalPose.set(new FramePose3D(ReferenceFrame.getWorldFrame(), packet.position, packet.orientation));
      });
      communicationBridge.attachListener(PlanarRegionsListMessage.class, planarRegionsListMessage::set);
      communicationBridge.attachListener(FootstepPlanningToolboxOutputStatus.class, plannerOutputStatus::set);

      nextSideToSwing.set(RobotSide.LEFT);

      parentRegistry.addChild(registry);
   }

   @Override
   public void doTransitionIntoAction()
   {
      if(debug)
         PrintTools.info("Entering plan state");

      plannerRequestHasBeenSent.set(false);
      plannerOutputStatus.set(null);
   }

   @Override
   public void doAction()
   {
      if(goalPose.get() == null)
      {
         return;
      }

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
}
