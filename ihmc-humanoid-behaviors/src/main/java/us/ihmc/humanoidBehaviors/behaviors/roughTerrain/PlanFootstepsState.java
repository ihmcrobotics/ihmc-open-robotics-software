package us.ihmc.humanoidBehaviors.behaviors.roughTerrain;

import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.PlanarRegionsListMessage;
import us.ihmc.communication.packets.ToolboxStateMessage;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.footstepPlanning.FootstepPlannerType;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridge;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepPlanningRequestPacket;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepPlanningToolboxOutputStatus;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.MovingReferenceFrame;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.State;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.concurrent.atomic.AtomicReference;

class PlanFootstepsState extends State<WalkOverTerrainStateMachineBehavior.WalkOverTerrainState>
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final YoInteger planId = new YoInteger("PlanId", registry);
   private final SideDependentList<MovingReferenceFrame> soleFrames;

   private final AtomicReference<FramePose3D> goalPose = new AtomicReference<>();
   private final AtomicReference<PlanarRegionsListMessage> planarRegionsListMessage = new AtomicReference<>();
   private final AtomicReference<FootstepPlanningToolboxOutputStatus> plannerOutputStatus = new AtomicReference<>();

   private final YoEnum<RobotSide> nextSideToSwing = new YoEnum<RobotSide>("nextSideToSwing", registry, RobotSide.class, false);
   private final YoDouble swingTime = new YoDouble("swingTime", registry);
   private final CommunicationBridge communicationBridge;

   public PlanFootstepsState(CommunicationBridge communicationBridge, SideDependentList<MovingReferenceFrame> soleFrames, YoVariableRegistry parentRegistry)
   {
      super(WalkOverTerrainStateMachineBehavior.WalkOverTerrainState.PLAN_FOOTSTEPS);
      this.soleFrames = soleFrames;
      this.communicationBridge = communicationBridge;

      communicationBridge.attachListener(WalkOverTerrainGoalPacket.class, (packet) -> goalPose.set(packet.goalPose));
      communicationBridge.attachListener(PlanarRegionsListMessage.class, planarRegionsListMessage::set);
      communicationBridge.attachListener(FootstepPlanningToolboxOutputStatus.class, plannerOutputStatus::set);

      swingTime.set(Double.NaN);

      parentRegistry.addChild(registry);
   }

   public void setNextSideToSwing(RobotSide nextSideToSwing)
   {
      this.nextSideToSwing.set(nextSideToSwing);
   }

   public void setSwingTime(double swingTime)
   {
      this.swingTime.set(swingTime);
   }

   @Override
   public void doTransitionIntoAction()
   {
      if(swingTime.getDoubleValue() == Double.NaN)
      {
         throw new RuntimeException("Swing time hasn't been set, call setSwingTime before executing state");
      }

      ToolboxStateMessage wakeUp = new ToolboxStateMessage(ToolboxStateMessage.ToolboxState.WAKE_UP);
      wakeUp.setDestination(PacketDestination.FOOTSTEP_PLANNING_TOOLBOX_MODULE);
      communicationBridge.sendPacket(wakeUp);

      FootstepPlanningRequestPacket planningRequestPacket = createPlanningRequestPacket();
      planningRequestPacket.setDestination(PacketDestination.FOOTSTEP_PLANNING_TOOLBOX_MODULE);
      communicationBridge.sendPacket(planningRequestPacket);
   }

   @Override
   public void doAction()
   {
   }

   @Override
   public void doTransitionOutOfAction()
   {
      FootstepDataListMessage footstepDataMessage = plannerOutputStatus.get().footstepDataList;
      footstepDataMessage.setDestination(PacketDestination.CONTROLLER);
      communicationBridge.sendPacket(footstepDataMessage);
   }

   public boolean planHasBeenReceived()
   {
      return plannerOutputStatus.get() != null;
   }

   public boolean planIsValidForExecution()
   {
      return plannerOutputStatus.get().planningResult.validForExecution();
   }

   public FootstepPlanningToolboxOutputStatus getToolboxOutputStatus()
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
