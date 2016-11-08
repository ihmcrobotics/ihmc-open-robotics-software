package us.ihmc.avatar.networkProcessor.footstepPlanningToolboxModule;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import us.ihmc.avatar.networkProcessor.modules.ToolboxController;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.footstepPlanning.FootstepPlanner;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.footstepPlanning.simplePlanners.TurnWalkTurnPlanner;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage.FootstepOrigin;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepPlanningRequestPacket;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepPlanningToolboxOutputStatus;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;

public class FootstepPlanningToolboxController extends ToolboxController<FootstepPlanningToolboxOutputStatus>
{
   private final AtomicReference<FootstepPlanningRequestPacket> latestRequestReference = new AtomicReference<FootstepPlanningRequestPacket>(null);
   private final BooleanYoVariable isDone = new BooleanYoVariable("isDone", registry);

   private final FootstepPlanner planner = new TurnWalkTurnPlanner();
   private RobotSide stepSide;

   public FootstepPlanningToolboxController(StatusMessageOutputManager statusOutputManager, YoVariableRegistry parentRegistry)
   {
      super(statusOutputManager, parentRegistry);
   }

   @Override
   protected void updateInternal()
   {
      List<FramePose> footstepPlan = new ArrayList<>();
      FootstepPlanningResult status = planner.plan(footstepPlan);

      reportMessage(packResult(footstepPlan, status, stepSide));
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
      planner.setGoalPose(goalPose);
      stepSide = request.initialStanceSide.getOppositeSide();

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

   private FootstepPlanningToolboxOutputStatus packResult(List<FramePose> footstepPlan, FootstepPlanningResult status, RobotSide firstStepSide)
   {
      FootstepPlanningToolboxOutputStatus result = new FootstepPlanningToolboxOutputStatus();
      result.footstepDataList = new FootstepDataListMessage();
      result.planningResult = status;

      for (FramePose footstepPose : footstepPlan)
      {
         Point3d location = new Point3d();
         Quat4d orientation = new Quat4d();
         footstepPose.getPosition(location);
         footstepPose.getOrientation(orientation);
         FootstepDataMessage footstepData = new FootstepDataMessage(stepSide, location, orientation);
         footstepData.setOrigin(FootstepOrigin.AT_SOLE_FRAME);
         result.footstepDataList.add(footstepData);
         stepSide = stepSide.getOppositeSide();
      }

      return result;
   }

}
