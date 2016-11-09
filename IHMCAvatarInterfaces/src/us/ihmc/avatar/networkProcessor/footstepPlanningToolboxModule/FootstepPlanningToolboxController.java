package us.ihmc.avatar.networkProcessor.footstepPlanningToolboxModule;

import java.util.concurrent.atomic.AtomicReference;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import us.ihmc.avatar.networkProcessor.modules.ToolboxController;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlanner;
import us.ihmc.footstepPlanning.FootstepPlannerGoal;
import us.ihmc.footstepPlanning.FootstepPlannerGoalType;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.footstepPlanning.SimpleFootstep;
import us.ihmc.footstepPlanning.simplePlanners.PlanThenSnapPlanner;
import us.ihmc.footstepPlanning.simplePlanners.TurnWalkTurnPlanner;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage.FootstepOrigin;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepPlanningRequestPacket;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepPlanningToolboxOutputStatus;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.wholeBodyController.RobotContactPointParameters;

public class FootstepPlanningToolboxController extends ToolboxController<FootstepPlanningToolboxOutputStatus>
{
   private final AtomicReference<FootstepPlanningRequestPacket> latestRequestReference = new AtomicReference<FootstepPlanningRequestPacket>(null);
   private final BooleanYoVariable isDone = new BooleanYoVariable("isDone", registry);

   private final FootstepPlanner planner;

   public FootstepPlanningToolboxController(RobotContactPointParameters contactPointParameters, StatusMessageOutputManager statusOutputManager, YoVariableRegistry parentRegistry)
   {
      super(statusOutputManager, parentRegistry);

      SideDependentList<ConvexPolygon2d> footPolygons = new SideDependentList<>();
      for (RobotSide side : RobotSide.values)
         footPolygons.set(side, new ConvexPolygon2d(contactPointParameters.getFootContactPoints().get(side)));
      planner = new PlanThenSnapPlanner(new TurnWalkTurnPlanner(), footPolygons);
   }

   @Override
   protected void updateInternal()
   {
      FootstepPlanningResult status = planner.plan();
      FootstepPlan footstepPlan = planner.getPlan();

      reportMessage(packResult(footstepPlan, status));
      isDone.set(true);
   }

   @Override
   protected boolean initialize()
   {
      isDone.set(false);
      FootstepPlanningRequestPacket request = latestRequestReference.getAndSet(null);
      if (request == null)
         return false;

      FramePose initialStancePose = new FramePose(ReferenceFrame.getWorldFrame());
      initialStancePose.setPosition(new Point3d(request.stanceFootPositionInWorld));
      initialStancePose.setOrientation(new Quat4d(request.stanceFootOrientationInWorld));

      FramePose goalPose = new FramePose(ReferenceFrame.getWorldFrame());
      goalPose.setPosition(new Point3d(request.goalPositionInWorld));
      goalPose.setOrientation(new Quat4d(request.goalOrientationInWorld));

      planner.setInitialStanceFoot(initialStancePose, request.initialStanceSide);

      FootstepPlannerGoal goal = new FootstepPlannerGoal();
      goal.setFootstepPlannerGoalType(FootstepPlannerGoalType.POSE_BETWEEN_FEET);
      goal.setGoalPoseBetweenFeet(goalPose);
      planner.setGoal(goal);

      return true;
   }

   public PacketConsumer<FootstepPlanningRequestPacket> createRequestConsumer()
   {
      return new PacketConsumer<FootstepPlanningRequestPacket>()
      {
         @Override
         public void receivedPacket(FootstepPlanningRequestPacket packet)
         {
            if (packet == null)
               return;
            latestRequestReference.set(packet);
         }
      };
   }

   @Override
   protected boolean isDone()
   {
      return isDone.getBooleanValue();
   }

   private FootstepPlanningToolboxOutputStatus packResult(FootstepPlan footstepPlan, FootstepPlanningResult status)
   {
      FootstepPlanningToolboxOutputStatus result = new FootstepPlanningToolboxOutputStatus();
      result.footstepDataList = new FootstepDataListMessage();
      result.planningResult = status;

      int numberOfSteps = footstepPlan.getNumberOfSteps();

      for (int i = 0; i < numberOfSteps; i++)
      {
         SimpleFootstep footstep = footstepPlan.getFootstep(i);

         FramePose footstepPose = new FramePose();
         footstep.getSoleFramePose(footstepPose);

         Point3d location = new Point3d();
         Quat4d orientation = new Quat4d();
         footstepPose.getPosition(location);
         footstepPose.getOrientation(orientation);
         FootstepDataMessage footstepData = new FootstepDataMessage(footstep.getRobotSide(), location, orientation);
         footstepData.setOrigin(FootstepOrigin.AT_SOLE_FRAME);
         result.footstepDataList.add(footstepData);
      }

      return result;
   }

}
