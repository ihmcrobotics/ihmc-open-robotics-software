package us.ihmc.commonWalkingControlModules.desiredFootStep;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;

import us.ihmc.commonWalkingControlModules.controlModules.HeadOrientationControlModule;
import us.ihmc.utilities.io.streamingData.AbstractStreamingDataConsumer;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

/**
 * User: Matt
 * Date: 1/28/13
 */
public class DesiredHeadOrientationProvider
{
   private static final boolean DEBUG = false;
   private HeadOrientationControlModule headOrientationControlModule;

   private AbstractStreamingDataConsumer<Point3d> absoluteHeadOrientationConsumer;
   private AbstractStreamingDataConsumer<Point2d> relativeHeadOrientationConsumer;
   private final ReferenceFrame elevatorFrame;

   public DesiredHeadOrientationProvider(long absoluteHeadOrientationDataIdentifier, long relativeHeadOrientationDataIdentifier, ReferenceFrame elevatorFrame)
   {
      absoluteHeadOrientationConsumer = new AbsoluteHeadOrientationConsumer(absoluteHeadOrientationDataIdentifier);
      relativeHeadOrientationConsumer = new RelativeHeadOrientationConsumer(relativeHeadOrientationDataIdentifier);
      this.elevatorFrame = elevatorFrame;
   }

   public AbstractStreamingDataConsumer<Point3d> getAbsoluteHeadOrientationConsumer()
   {
      return absoluteHeadOrientationConsumer;
   }

   public AbstractStreamingDataConsumer<Point2d> getRelativeHeadOrientationConsumer()
   {
      return relativeHeadOrientationConsumer;
   }

   public void setHeadOrientationControlModule(HeadOrientationControlModule headOrientationControlModule)
   {
      this.headOrientationControlModule = headOrientationControlModule;
   }

   private class AbsoluteHeadOrientationConsumer extends AbstractStreamingDataConsumer<Point3d>
   {
      public AbsoluteHeadOrientationConsumer(long objectIdentifier)
      {
         super(objectIdentifier, Point3d.class);
      }

      protected void processPacket(Point3d point3d)
      {
         if (DEBUG)
         {
            System.out.println("DesiredHeadOrientationProvider: absolute orientation: " + point3d);
         }

         if (headOrientationControlModule != null)
         {
            FramePoint pointToTrack = new FramePoint(ReferenceFrame.getWorldFrame(), point3d);
            headOrientationControlModule.setPointToTrack(pointToTrack, elevatorFrame);
         }
      }
   }


   private class RelativeHeadOrientationConsumer extends AbstractStreamingDataConsumer<Point2d>
   {
      public RelativeHeadOrientationConsumer(long objectIdentifier)
      {
         super(objectIdentifier, Point2d.class);
      }

      protected void processPacket(Point2d point2d)
      {
         if (DEBUG)
         {
            System.out.println("DesiredHeadOrientationProvider: relative orientation: " + point2d);
         }

         if (headOrientationControlModule != null)
         {
            double yaw = point2d.getX();
            double pitch = point2d.getY();
            double roll = 0.0;
            FrameOrientation frameOrientation = new FrameOrientation(ReferenceFrame.getWorldFrame(), yaw, pitch, roll);
            headOrientationControlModule.setOrientationToTrack(frameOrientation, elevatorFrame);
         }
      }
   }
}
