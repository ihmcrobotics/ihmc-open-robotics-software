package us.ihmc.humanoidRobotics.communication.subscribers;

import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.walking.CapturabilityBasedStatus;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class CapturabilityBasedStatusSubscriber implements PacketConsumer<CapturabilityBasedStatus>
{
   private final AtomicReference<FramePoint2D> capturePointReference = new AtomicReference<FramePoint2D>(null);
   private final AtomicReference<FramePoint2D> desiredCapturePointReference = new AtomicReference<FramePoint2D>(null);
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

   public FramePoint2D getCapturePoint()
   {
      return capturePointReference.getAndSet(null);
   }

   public FramePoint2D getDesiredCapturePoint()
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

      capturePointReference.set(new FramePoint2D(ReferenceFrame.getWorldFrame(), object.capturePoint));
      desiredCapturePointReference.set(new FramePoint2D(ReferenceFrame.getWorldFrame(), object.desiredCapturePoint));
      for (RobotSide robotSide : RobotSide.values)
         footSupportPolygonReferences.get(robotSide).set(HumanoidMessageTools.unpackFootSupportPolygon(object, robotSide));
      doubleSupportReference.set(HumanoidMessageTools.unpackIsInDoubleSupport(object));

      for (CapturabilityBasedStatusListener listener : listOfListener)
      {
         listener.updateStatusPacket(object);
      }
   }
}
