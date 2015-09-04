package us.ihmc.commonWalkingControlModules.packetProducers;

import java.util.concurrent.TimeUnit;

import us.ihmc.communication.packets.walking.CapturabilityBasedStatus;
import us.ihmc.communication.streamingData.GlobalDataProducer;
import us.ihmc.concurrent.Builder;
import us.ihmc.concurrent.ConcurrentRingBuffer;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.thread.CloseableAndDisposable;
import us.ihmc.tools.thread.CloseableAndDisposableRegistry;
import us.ihmc.util.PeriodicThreadScheduler;

public class CapturabilityBasedStatusProducer implements Runnable, CloseableAndDisposable
{
   private final GlobalDataProducer objectCommunicator;
   private final PeriodicThreadScheduler scheduler;
   private final ConcurrentRingBuffer<CapturabilityBasedStatus> capturabilityBuffer;

   public CapturabilityBasedStatusProducer(CloseableAndDisposableRegistry closeAndDisposeRegistry, PeriodicThreadScheduler scheduler, GlobalDataProducer objectCommunicator)
   {
      this.scheduler = scheduler;
      this.objectCommunicator = objectCommunicator;
      this.capturabilityBuffer = new ConcurrentRingBuffer<>(new CapturabilityBasedStatusBuilder(), 16);
      scheduler.schedule(this, 1, TimeUnit.MILLISECONDS);
      
      closeAndDisposeRegistry.registerCloseable(this);
   }

   public void sendStatus(FramePoint2d capturePoint2d, FramePoint2d desiredCapturePoint2d, FramePoint centerOfMass, SideDependentList<FrameConvexPolygon2d> footSupportPolygons)
   {
      capturePoint2d.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());
      desiredCapturePoint2d.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());

      centerOfMass.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());
      
      CapturabilityBasedStatus nextStatus = capturabilityBuffer.next();
      
      if(nextStatus != null)
      {
         capturePoint2d.get(nextStatus.capturePoint);
         desiredCapturePoint2d.get(nextStatus.desiredCapturePoint);
         centerOfMass.get(nextStatus.centerOfMass);
         for(RobotSide robotSide : RobotSide.values)
         {
            nextStatus.setSupportPolygon(robotSide, footSupportPolygons.get(robotSide));
         }         
         capturabilityBuffer.commit();
      }

   }
   
   public void closeAndDispose()
   {
      scheduler.shutdown();
   }

   @Override
   public void run()
   {
      if(capturabilityBuffer.poll())
      {
         CapturabilityBasedStatus status;
         while((status = capturabilityBuffer.read()) != null)
         {
            objectCommunicator.send(status);
         }
         capturabilityBuffer.flush();
      }
   }

   private static class CapturabilityBasedStatusBuilder implements Builder<CapturabilityBasedStatus>
   {

      @Override
      public CapturabilityBasedStatus newInstance()
      {
         return new CapturabilityBasedStatus();
      }

   }
}
