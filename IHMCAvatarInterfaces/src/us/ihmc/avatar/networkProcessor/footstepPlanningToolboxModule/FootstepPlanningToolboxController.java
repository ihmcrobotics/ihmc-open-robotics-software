package us.ihmc.avatar.networkProcessor.footstepPlanningToolboxModule;

import java.util.concurrent.atomic.AtomicReference;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import us.ihmc.avatar.networkProcessor.modules.ToolboxController;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packets.FootstepPlanningToolboxOutputStatus;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepPlanningRequestPacket;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class FootstepPlanningToolboxController extends ToolboxController<FootstepPlanningToolboxOutputStatus>
{
   private final AtomicReference<FootstepPlanningRequestPacket> latestRequestReference = new AtomicReference<FootstepPlanningRequestPacket>(null);
   private final FootstepPlanningToolboxOutputStatus result = new FootstepPlanningToolboxOutputStatus();
   private final BooleanYoVariable isDone = new BooleanYoVariable("isDone", registry);

   public FootstepPlanningToolboxController(StatusMessageOutputManager statusOutputManager, YoVariableRegistry parentRegistry)
   {
      super(statusOutputManager, parentRegistry);
   }

   @Override
   protected void updateInternal()
   {
      // TODO

      reportMessage(result);
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

      // TODO

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

}
