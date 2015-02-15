package us.ihmc.commonWalkingControlModules.packetConsumers;

import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packets.walking.PelvisPosePacket;
import us.ihmc.communication.packets.wholebody.WholeBodyTrajectoryDevelopmentPacket;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.Twist;

/**
 * User: Matt
 * Date: 2/18/13
 */
public class DesiredPelvisPoseProvider implements PacketConsumer<PelvisPosePacket>, PelvisPoseProvider
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final AtomicBoolean goToHomePosition = new AtomicBoolean(false);
   private final AtomicBoolean goToHomeOrientation = new AtomicBoolean(false);
   private final AtomicReference<FramePoint> desiredPelvisPosition = new AtomicReference<FramePoint>(new FramePoint(ReferenceFrame.getWorldFrame()));
   private final AtomicReference<FrameOrientation> desiredPelvisOrientation = new AtomicReference<FrameOrientation>(new FrameOrientation(ReferenceFrame.getWorldFrame()));
   private double trajectoryTime = Double.NaN;

   private final PacketConsumer<WholeBodyTrajectoryDevelopmentPacket> wholeBodyTrajectoryPacketConsumer;  
   AtomicReference<FramePoint[]> pelvisPositions = new AtomicReference<FramePoint[]>();
   AtomicReference<FrameOrientation[]> pelvisOrientations = new AtomicReference<FrameOrientation[]>();
   
   AtomicReference<double[]> pelvisWaypointsTime = new AtomicReference<double[]>();
   AtomicReference<FrameVector[]> pelvisVelocity = new AtomicReference<FrameVector[]>();
   AtomicReference<Twist[]> pelvisTwist = new AtomicReference<Twist[]>();
   
   WholeBodyTrajectoryDevelopmentPacket lastWholeBodyPacket;

   public DesiredPelvisPoseProvider()
   {
      wholeBodyTrajectoryPacketConsumer = new PacketConsumer<WholeBodyTrajectoryDevelopmentPacket>()
      {
         @Override
         public void receivedPacket(WholeBodyTrajectoryDevelopmentPacket packet)
         {
            if (packet != null)
            {
               lastWholeBodyPacket = packet;
               System.out.println(" PACKET RECEIVED ");
            }
         }
      };
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
   public FramePoint getDesiredPelvisPosition(ReferenceFrame supportFrame)
   {
      return desiredPelvisPosition.getAndSet(null);
   }

   @Override
   public FrameOrientation getDesiredPelvisOrientation(ReferenceFrame desiredPelvisFrame)
   {
      FrameOrientation ret = desiredPelvisOrientation.getAndSet(null);

      if (ret == null)
         return null;

      ret.changeFrame(desiredPelvisFrame);
      return ret;
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

   @Override
   public boolean checkForNewTrajectory()
   {
      return (lastWholeBodyPacket != null);
   }

   @Override
   public ReferenceFrame getDesiredPelvisPositionTrajectory(
         ArrayList<Double> time,
         ArrayList<Point3d> position, 
         ArrayList<Vector3d> velocity ) 
   {     
      int N = lastWholeBodyPacket.getNumberOfWaypoints();
        
      for(int i=0; i<N; i++)
      {
         time.add( lastWholeBodyPacket.absTime[i] );
         position.add( lastWholeBodyPacket.pelvisWorldPosition[i] );
         velocity.add( lastWholeBodyPacket.pelvisWorldVelocity[i] );
      }
      return ReferenceFrame.getWorldFrame();
   }

   @Override
   public ReferenceFrame getDesiredPelvisOrientationTrajectory(double[] time, Point3d[] position, Vector3d[] velocity)
   {
      // TODO
      return null;
   }
   
   public PacketConsumer<WholeBodyTrajectoryDevelopmentPacket> getWholeBodyTrajectoryConsumer()
   {
      return wholeBodyTrajectoryPacketConsumer;
   }

   @Override
   public void removeLastTrajectory()
   {
      lastWholeBodyPacket = null;
   }
   
}
