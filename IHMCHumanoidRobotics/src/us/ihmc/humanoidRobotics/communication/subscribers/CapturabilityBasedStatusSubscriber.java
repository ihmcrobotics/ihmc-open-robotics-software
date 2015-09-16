package us.ihmc.humanoidRobotics.communication.subscribers;

import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.humanoidRobotics.communication.packets.walking.CapturabilityBasedStatus;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class CapturabilityBasedStatusSubscriber implements PacketConsumer<CapturabilityBasedStatus>
{
   private final AtomicReference<FramePoint2d> capturePointReference = new AtomicReference<FramePoint2d>(null);
   private final AtomicReference<FramePoint2d> desiredCapturePointReference = new AtomicReference<FramePoint2d>(null);
   private final SideDependentList<AtomicReference<FrameConvexPolygon2d>> footSupportPolygonReferences = new SideDependentList<>();
   private final AtomicReference<Boolean> doubleSupportReference = new AtomicReference<Boolean>(null);

   private ArrayList<CapturabilityBasedStatusListener> listOfListener = new ArrayList<>();

   public CapturabilityBasedStatusSubscriber()
   {
      for (RobotSide robotSide : RobotSide.values)
         footSupportPolygonReferences.put(robotSide, new AtomicReference<FrameConvexPolygon2d>(null));
   }

   public void registerListener(CapturabilityBasedStatusListener listener)
   {
      listOfListener.add(listener);
   }

   public FramePoint2d getCapturePoint()
   {
      return capturePointReference.getAndSet(null);
   }

   public FramePoint2d getDesiredCapturePoint()
   {
      return desiredCapturePointReference.getAndSet(null);
   }

   public FrameConvexPolygon2d getFootSupportPolygon(RobotSide robotSide)
   {
      return footSupportPolygonReferences.get(robotSide).getAndSet(null);
   }

   public Boolean isInDoubleSupport()
   {
      return doubleSupportReference.getAndSet(null);
   }

   @Override
   public void receivedPacket(CapturabilityBasedStatus object)
   {
      if (object == null)
         return;

      capturePointReference.set(object.getCapturePoint());
      desiredCapturePointReference.set(object.getDesiredCapturePoint());
      for (RobotSide robotSide : RobotSide.values)
         footSupportPolygonReferences.get(robotSide).set(object.getFootSupportPolygon(robotSide));
      doubleSupportReference.set(object.isInDoubleSupport());

      for (CapturabilityBasedStatusListener listener : listOfListener)
      {
         listener.updateStatusPacket(object);
      }
   }
}
