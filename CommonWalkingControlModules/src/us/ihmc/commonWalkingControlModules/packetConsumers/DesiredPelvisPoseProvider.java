package us.ihmc.commonWalkingControlModules.packetConsumers;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packets.manipulation.StopMotionPacket;
import us.ihmc.communication.packets.walking.PelvisPosePacket;
import us.ihmc.communication.packets.wholebody.WholeBodyTrajectoryPacket;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.math.trajectories.WaypointOrientationTrajectoryData;
import us.ihmc.robotics.math.trajectories.WaypointPositionTrajectoryData;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

/**
 * User: Matt
 * Date: 2/18/13
 */
public class DesiredPelvisPoseProvider implements PelvisPoseProvider
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final PacketConsumer<WholeBodyTrajectoryPacket> wholeBodyTrajectoryConsumer;
   private final PacketConsumer<StopMotionPacket>    stopMotionConsumer;
   private final PacketConsumer<PelvisPosePacket>    pelvisPoseConsumer;

   private final AtomicBoolean goToHomePosition = new AtomicBoolean(false);
   private final AtomicBoolean goToHomeOrientation = new AtomicBoolean(false);
   private final AtomicReference<FramePoint> desiredPelvisPosition = new AtomicReference<FramePoint>(null);
   private final AtomicReference<FrameOrientation> desiredPelvisOrientation = new AtomicReference<FrameOrientation>(null);
   private final AtomicReference<WaypointPositionTrajectoryData> desiredPelvisPositionWithWaypoints = new AtomicReference<>(null);
   private final AtomicReference<WaypointOrientationTrajectoryData> desiredPelvisOrientationWithWaypoints = new AtomicReference<>(null);
   private final AtomicReference<StopMotionPacket> stopMotion = new  AtomicReference<>(null);
   private double trajectoryTime = Double.NaN;

   public DesiredPelvisPoseProvider()
   {
      pelvisPoseConsumer = new PacketConsumer<PelvisPosePacket>()
      {
         @Override
         public void receivedPacket(PelvisPosePacket packet)
         {
            receivedPacketImpl(packet);
         }
      };
      
      stopMotionConsumer = new PacketConsumer<StopMotionPacket>()
      {
         @Override
         public void receivedPacket(StopMotionPacket packet)
         {
            receivedPacketImpl(packet);
         }
      };
      
      wholeBodyTrajectoryConsumer = new PacketConsumer<WholeBodyTrajectoryPacket>()
      {
         @Override
         public void receivedPacket(WholeBodyTrajectoryPacket packet)
         {
            //System.out.println("DesiredPelvisPoseProvider: PACKET received");

            if (packet != null)
            {
               double[] timeAtWaypoints = packet.timeAtWaypoint;
               Point3d[] positions = packet.pelvisWorldPosition;
               Vector3d[] velocities = packet.pelvisLinearVelocity;
               if (positions != null)
               {
                  if (velocities == null)
                  {
                     velocities = new Vector3d[packet.numWaypoints];
                     for (int i = 0; i < packet.numWaypoints; ++i)
                     {
                        velocities[i] = new Vector3d();
                     }
                  }
                  WaypointPositionTrajectoryData positionTrajectoryData = new WaypointPositionTrajectoryData(worldFrame, timeAtWaypoints, positions, velocities);
                  desiredPelvisPositionWithWaypoints.set(positionTrajectoryData);
               }

               Quat4d[] orientations = packet.pelvisWorldOrientation;
               // TODO: angular velocity is not used yet
               if (orientations != null)
               {
                  WaypointOrientationTrajectoryData orientationTrajectoryData = new WaypointOrientationTrajectoryData(worldFrame, timeAtWaypoints, orientations, null);
                  desiredPelvisOrientationWithWaypoints.set(orientationTrajectoryData);
               }
            }
         }
      };
   }

   public PacketConsumer<WholeBodyTrajectoryPacket> getWholeBodyTrajectoryPacketConsumer()
   {
      return wholeBodyTrajectoryConsumer;
   }
   
   public PacketConsumer<StopMotionPacket> getStopMotionPacketConsumer()
   {
      return stopMotionConsumer;
   }
   
   public PacketConsumer<PelvisPosePacket> getPelvisPosePacketConsumer()
   {
      return pelvisPoseConsumer;
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
   public void clearOrientation()
   {
      desiredPelvisOrientation.set(null);
      desiredPelvisOrientationWithWaypoints.set(null);
      goToHomeOrientation.set(false);
   }

   @Override
   public void clearPosition()
   {
      desiredPelvisPosition.set(null);
      desiredPelvisPositionWithWaypoints.set(null);
      goToHomePosition.set(false);
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


   private void receivedPacketImpl(PelvisPosePacket object)
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
   
   private void receivedPacketImpl(StopMotionPacket object)
   {
      if (object == null)
         return;

      trajectoryTime = 0.1;
      stopMotion.set( object );
   }

   @Override
   public boolean checkAndResetStopCommand()
   {
      return stopMotion.getAndSet( null ) != null;
   }
}
