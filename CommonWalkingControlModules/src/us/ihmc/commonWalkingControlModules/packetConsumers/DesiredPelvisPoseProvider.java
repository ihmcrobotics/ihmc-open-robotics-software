package us.ihmc.commonWalkingControlModules.packetConsumers;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packets.walking.PelvisPosePacket;
import us.ihmc.communication.packets.wholebody.WholeBodyTrajectoryPacket;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.yoUtilities.math.trajectories.WaypointOrientationTrajectoryData;
import us.ihmc.yoUtilities.math.trajectories.WaypointPositionTrajectoryData;

/**
 * User: Matt
 * Date: 2/18/13
 */
public class DesiredPelvisPoseProvider implements PacketConsumer<PelvisPosePacket>, PelvisPoseProvider
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final PacketConsumer<WholeBodyTrajectoryPacket> wholeBodyTrajectoryPacketConsumer;

   private final AtomicBoolean goToHomePosition = new AtomicBoolean(false);
   private final AtomicBoolean goToHomeOrientation = new AtomicBoolean(false);
   private final AtomicReference<FramePoint> desiredPelvisPosition = new AtomicReference<FramePoint>(new FramePoint());
   private final AtomicReference<FrameOrientation> desiredPelvisOrientation = new AtomicReference<FrameOrientation>(new FrameOrientation());
   private final AtomicReference<WaypointPositionTrajectoryData> desiredPelvisPositionWithWaypoints = new AtomicReference<>(null);
   private final AtomicReference<WaypointOrientationTrajectoryData> desiredPelvisOrientationWithWaypoints = new AtomicReference<>(null);
   private double trajectoryTime = Double.NaN;

   public DesiredPelvisPoseProvider()
   {
      wholeBodyTrajectoryPacketConsumer = new PacketConsumer<WholeBodyTrajectoryPacket>()
            {
         @Override
         public void receivedPacket(WholeBodyTrajectoryPacket packet)
         {
            if (packet != null && packet.hasPelvisTrajectory() )
            {
               double[] timeAtWaypoints = packet.timeSincePrevious;
               Point3d[] positions = packet.pelvisWorldPosition;
               Vector3d[] velocities = packet.pelvisLinearVelocity;
               WaypointPositionTrajectoryData positionTrajectoryData = new WaypointPositionTrajectoryData(worldFrame, timeAtWaypoints, positions, velocities);
               desiredPelvisPositionWithWaypoints.set(positionTrajectoryData);

               Quat4d[] orientations = packet.pelvisWorldOrientation;
               // TODO: angular velocity is not used yet
               WaypointOrientationTrajectoryData orientationTrajectoryData = new WaypointOrientationTrajectoryData(worldFrame, timeAtWaypoints, orientations, null);
               desiredPelvisOrientationWithWaypoints.set(orientationTrajectoryData);
            }
         }
            };
   }

   public PacketConsumer<WholeBodyTrajectoryPacket> getWholeBodyTrajectoryPacketConsumer()
   {
      return wholeBodyTrajectoryPacketConsumer;
   }

   @Override
   public boolean checkForNewPosition()
   {
      return desiredPelvisPosition.get() != null;
   }

   @Override
   public boolean checkForNewOrientation()
   {
      return desiredPelvisOrientation.get() != null;
   }

   @Override
   public boolean checkForNewOrientationWithWaypoints()
   {
      return desiredPelvisOrientationWithWaypoints.get() != null;
   }

   @Override
   public boolean checkForHomePosition()
   {
      return goToHomePosition.getAndSet(false);
   }

   @Override
   public boolean checkForHomeOrientation()
   {
      return goToHomeOrientation.getAndSet(false);
   }

   @Override
   public boolean checkForNewPositionWithWaypoints()
   {
      return desiredPelvisPositionWithWaypoints.get() != null;
   }

   @Override
   public FramePoint getDesiredPelvisPosition(ReferenceFrame supportFrame)
   {
      return desiredPelvisPosition.getAndSet(null);
   }

   @Override
   public WaypointPositionTrajectoryData getDesiredPelvisPositionWithWaypoints()
   {
      return desiredPelvisPositionWithWaypoints.getAndSet(null);
   }

   @Override
   public FrameOrientation getDesiredPelvisOrientation(ReferenceFrame desiredPelvisFrame)
   {
      FrameOrientation ret = desiredPelvisOrientation.getAndSet(null);

      if (ret == null) return null;

      ret.changeFrame(desiredPelvisFrame);
      return ret;
   }

   @Override
   public WaypointOrientationTrajectoryData getDesiredPelvisOrientationWithWaypoints()
   {
      return desiredPelvisOrientationWithWaypoints.getAndSet(null);
   }

   @Override
   public double getTrajectoryTime()
   {
      return trajectoryTime;
   }

   @Override
   public void receivedPacket(PelvisPosePacket object)
   {
      if (object == null)
         return;

      trajectoryTime = object.getTrajectoryTime();

      // If go to home position requested, ignore the other commands.
      if (object.isToHomePosition())
      {
         goToHomePosition.set(true);
         goToHomeOrientation.set(true);
         return;
      }

      if (object.getPosition() != null)
         desiredPelvisPosition.set(new FramePoint(worldFrame, object.getPosition()));
      else
         desiredPelvisPosition.set(null);

      if (object.getOrientation() != null)
         desiredPelvisOrientation.set(new FrameOrientation(worldFrame, object.getOrientation()));
      else
         desiredPelvisOrientation.set(null);
   }
}
