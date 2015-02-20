package us.ihmc.commonWalkingControlModules.packetConsumers;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

import javax.vecmath.Vector3d;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packets.walking.PelvisPosePacket;
import us.ihmc.communication.packets.wholebody.WholeBodyTrajectoryDevelopmentPacket;
import us.ihmc.utilities.math.geometry.FrameOrientationWaypoint;
import us.ihmc.utilities.math.geometry.FramePointWaypoint;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.google.common.util.concurrent.AtomicDouble;

/**
 * Responsible: Davide
 * Date: February 2015
 *
 */

public class DesiredPelvisPoseProvider extends  PelvisPoseProvider
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final AtomicBoolean goToHomePosition = new AtomicBoolean(false);
   private final AtomicBoolean goToHomeOrientation = new AtomicBoolean(false);
   private final AtomicDouble  trajectoryTimeToHomePosition = new AtomicDouble( -1 );

   private final PacketConsumer<WholeBodyTrajectoryDevelopmentPacket> wholeBodyTrajectoryPacketConsumer;
   private final PacketConsumer<PelvisPosePacket> pelvisPacketConsumer; 

   private AtomicReference<FramePointWaypoint[]>  pelvisPositions = new AtomicReference<FramePointWaypoint[]>();
   private AtomicReference<FrameOrientationWaypoint[]> pelvisOrientations = new AtomicReference<FrameOrientationWaypoint[]>();


   public DesiredPelvisPoseProvider()
   {
      wholeBodyTrajectoryPacketConsumer = new PacketConsumer<WholeBodyTrajectoryDevelopmentPacket>()
            {
         @Override public void receivedPacket(WholeBodyTrajectoryDevelopmentPacket packet)
         {
            if (packet != null){ receivedPacketImplementation(packet);  }
         }};

         pelvisPacketConsumer = new PacketConsumer<PelvisPosePacket>()
               {
            @Override      public void receivedPacket(PelvisPosePacket packet) 
            {
               if (packet != null){  receivedPacketImplementation(packet);  }
            } };   
   }

   @Override
   public boolean checkForNewPosition()
   {
      return pelvisPositions.get() != null;
   }

   @Override
   public Double getTrajectoryTimeToHome()
   {
      double out = trajectoryTimeToHomePosition.getAndSet( -1 );
      if( out >=0 )
         return new Double( out );
      else
         return null;
   }

   @Override
   public boolean checkForNewOrientation()
   {
      return pelvisOrientations.get() != null;
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


   private void receivedPacketImplementation(WholeBodyTrajectoryDevelopmentPacket object)
   {
      int N = object.timeSincePrevious.length;

      FramePointWaypoint[] position = new FramePointWaypoint[N];
      FrameOrientationWaypoint[] orientation = new FrameOrientationWaypoint[N];

      for (int i=0; i<N; i++)
      {
         position[i] = new FramePointWaypoint(worldFrame,
               object.timeSincePrevious[i],
               object.pelvisWorldPosition[i],
               object.pelvisWorldVelocity[i] );

         orientation[i] = new FrameOrientationWaypoint(worldFrame,
               object.timeSincePrevious[i],
               object.pelvisOrientation[i],
               null ); 

      }
      pelvisPositions.set( position );
      pelvisOrientations.set( orientation );
   }

   private void receivedPacketImplementation(PelvisPosePacket object)
   {
      // the trajectory will have only one point.

      // If go to home position requested, ignore the other commands.
      if (object.isToHomePosition())
      {
         goToHomePosition.set(true);
         goToHomeOrientation.set(true);
         trajectoryTimeToHomePosition.set( object.trajectoryTime );
         return;
      }

      if( object.trajectoryTime > 0)
      {
         if (object.getPosition() != null)
         {
            FramePointWaypoint[] position = new FramePointWaypoint[1];
            position[0] = new FramePointWaypoint(worldFrame, 
                  object.trajectoryTime, 
                  object.getPosition(),
                  new Vector3d()
                  );
            pelvisPositions.set( position );
         }
         else{
            pelvisPositions.set(null);
         }

         if (object.getOrientation() != null)
         {
            FrameOrientationWaypoint[] orientation = new FrameOrientationWaypoint[1];
            orientation[0] = new FrameOrientationWaypoint(worldFrame, 
                  object.trajectoryTime, 
                  object.getOrientation(),
                  new Vector3d()
                  );
            pelvisOrientations.set(orientation);
         }
         else{
            pelvisOrientations.set(null);
         }
      }
   }


   public PacketConsumer<WholeBodyTrajectoryDevelopmentPacket> getTrajectoryConsumer()
   {
      return wholeBodyTrajectoryPacketConsumer;
   }

   public PacketConsumer<PelvisPosePacket> getSingleTargetConsumer()
   {
      return pelvisPacketConsumer;
   }

//   @Override
//   public void getDesiredPelvisPositionTrajectory(
//         ArrayList<Double> time,
//         ArrayList<FramePoint> position, 
//         ArrayList<FrameVector> velocity ) 
//   {     
//      int N = lastWholeBodyPacket.getNumberOfWaypoints();
//        
//      for(int i=0; i<N; i++)
//      {
//         time.add( lastWholeBodyPacket.absTime[i] );
//         position.add( new FramePoint(ReferenceFrame.getWorldFrame(), lastWholeBodyPacket.pelvisWorldPosition[i]));
//         velocity.add( new FrameVector(ReferenceFrame.getWorldFrame(), lastWholeBodyPacket.pelvisWorldVelocity[i]));
//      }
//   }

   @Override
   public FramePointWaypoint[] getDesiredPelvisPosition(ReferenceFrame desiredFrame)
   {
      FramePointWaypoint[] output = pelvisPositions.getAndSet(null);
      if( output == null )
      {
         return null;
      }
      for(int i=0; i < output.length; i++ )
      {
         if( output[i].point != null)
            output[i].point.changeFrame(desiredFrame);

         if( output[i].linearVelocity != null)
            output[i].linearVelocity.changeFrame(desiredFrame);
      }
      return output;
   }
   
   @Override
   public FrameOrientationWaypoint[] getDesiredPelvisOrientation(ReferenceFrame desiredFrame )
   {
      FrameOrientationWaypoint[] output = pelvisOrientations.getAndSet(null);
      if( output == null )
      {
         return null;
      }
      for(int i=0; i<output.length; i++ )
      {
         if(output[i] != null )
         {
            if( output[i].orientation != null)
               output[i].orientation.changeFrame(desiredFrame);

            if( output[i].angularVelocity != null)
               output[i].angularVelocity.changeFrame(desiredFrame);
         }
      }
      return output;
   }

}
