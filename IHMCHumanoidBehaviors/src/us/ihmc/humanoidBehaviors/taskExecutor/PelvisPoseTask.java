package us.ihmc.humanoidBehaviors.taskExecutor;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import us.ihmc.communication.packets.walking.PelvisPosePacket;
import us.ihmc.communication.util.PacketControllerTools;
import us.ihmc.humanoidBehaviors.behaviors.primitives.PelvisPoseBehavior;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;

public class PelvisPoseTask extends BehaviorTask
{
   private final PelvisPosePacket pelvisPosePacket;
   private final PelvisPoseBehavior pelvisPoseBehavior;

   public PelvisPoseTask(FrameOrientation desiredPelvisOrientation, DoubleYoVariable yoTime, PelvisPoseBehavior pelvisPoseBehavior, double trajectoryTime)
   {
      this(desiredPelvisOrientation, yoTime, pelvisPoseBehavior, trajectoryTime, 0.0);
   }

   public PelvisPoseTask(FrameOrientation desiredPelvisOrientation, DoubleYoVariable yoTime, PelvisPoseBehavior pelvisPoseBehavior, double trajectoryTime, double sleepTime)
   {
      super(pelvisPoseBehavior, yoTime, sleepTime);
      this.pelvisPoseBehavior = pelvisPoseBehavior;
      Quat4d pelvisOrientation = new Quat4d();
      desiredPelvisOrientation.getQuaternion(pelvisOrientation);
      pelvisPosePacket = PacketControllerTools.createPelvisPosePacketForOrientationOnly(pelvisOrientation, trajectoryTime);
   }

   public PelvisPoseTask(FramePoint desiredPelvisPosition, DoubleYoVariable yoTime, PelvisPoseBehavior pelvisPoseBehavior, double trajectoryTime, double sleepTime)
   {
      super(pelvisPoseBehavior, yoTime, sleepTime);
      this.pelvisPoseBehavior = pelvisPoseBehavior;
      Point3d pelvisPosition = new Point3d();
      desiredPelvisPosition.get(pelvisPosition);
      pelvisPosePacket = PacketControllerTools.createPelvisPosePacketForPositionOnly(pelvisPosition, trajectoryTime);
   }

   public PelvisPoseTask(FramePose desiredPelvisPose, DoubleYoVariable yoTime, PelvisPoseBehavior pelvisPoseBehavior, double trajectoryTime)
   {
      this(desiredPelvisPose, yoTime, pelvisPoseBehavior, trajectoryTime, 0.0);
   }

   public PelvisPoseTask(FramePose desiredPelvisPose, DoubleYoVariable yoTime, PelvisPoseBehavior pelvisPoseBehavior, double trajectoryTime, double sleepTime)
   {
      super(pelvisPoseBehavior, yoTime, sleepTime);
      this.pelvisPoseBehavior = pelvisPoseBehavior;
      Point3d pelvisPosition = new Point3d();
      Quat4d pelvisOrientation = new Quat4d();
      desiredPelvisPose.getOrientation(pelvisOrientation);
      desiredPelvisPose.getPosition(pelvisPosition);
      pelvisPosePacket = PacketControllerTools.createPelvisPosePacketForPositionAndOrientation(pelvisPosition, pelvisOrientation, trajectoryTime);
   }

   public PelvisPoseTask(PelvisPosePacket pelvisPosePacket, DoubleYoVariable yoTime, PelvisPoseBehavior pelvisPoseBehavior)
   {
      this(pelvisPosePacket, yoTime, pelvisPoseBehavior, 0.0);
   }

   public PelvisPoseTask(PelvisPosePacket pelvisPosePacket, DoubleYoVariable yoTime, PelvisPoseBehavior pelvisPoseBehavior, double sleepTime)
   {
      super(pelvisPoseBehavior, yoTime, sleepTime);
      this.pelvisPoseBehavior = pelvisPoseBehavior;
      this.pelvisPosePacket = pelvisPosePacket;
   }

   @Override
   protected void setBehaviorInput()
   {
      pelvisPoseBehavior.setInput(pelvisPosePacket); 
   }
}
