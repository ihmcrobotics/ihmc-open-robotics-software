package us.ihmc.commonWalkingControlModules.desiredFootStep;

import us.ihmc.commonWalkingControlModules.controlModules.HeadOrientationControlModule;
import us.ihmc.utilities.io.streamingData.AbstractStreamingDataConsumer;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import javax.vecmath.Point3d;

/**
 * User: Matt
 * Date: 1/28/13
 */
public class DesiredHeadOrientationProvider extends AbstractStreamingDataConsumer<Point3d>
{
   private static final boolean DEBUG = false;
   private HeadOrientationControlModule headOrientationControlModule;

   public DesiredHeadOrientationProvider(long dataIdentifier)
   {
      super(dataIdentifier, Point3d.class);
   }

   protected void processPacket(Point3d point3d)
   {
      if (DEBUG)
         System.out.println("DesiredHeadOrientationProvider: processPacket: " + point3d);

      if(headOrientationControlModule != null)
      {
         FramePoint pointToTrack = new FramePoint(ReferenceFrame.getWorldFrame(), point3d);
         headOrientationControlModule.setPointToTrack(pointToTrack, headOrientationControlModule.getElevatorFrame());
      }
   }

   public void setHeadOrientationControlModule(HeadOrientationControlModule headOrientationControlModule)
   {
      this.headOrientationControlModule = headOrientationControlModule;
   }
}

