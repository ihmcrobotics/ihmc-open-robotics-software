package us.ihmc.quadrupedPlanning.footstepChooser;

import java.util.concurrent.atomic.AtomicReference;

import ihmc_common_msgs.msg.dds.GroundPlaneMessage;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.quadrupedBasics.referenceFrames.QuadrupedReferenceFrames;

public class PlanarGroundPointFootSnapper implements PointFootSnapper
{
   private final AtomicReference<GroundPlaneMessage> groundPlaneMessage = new AtomicReference<>();
   private final Plane3D plane = new Plane3D();
   private final Point3D snappedPoint = new Point3D();

   private final ReferenceFrame centroidFrame;

   public PlanarGroundPointFootSnapper(QuadrupedReferenceFrames referenceFrames)
   {
      this.centroidFrame = referenceFrames.getCenterOfFeetZUpFrameAveragingLowestZHeightsAcrossEnds();
   }


   public void submitGroundPlane(GroundPlaneMessage groundPlaneMessage)
   {
      this.groundPlaneMessage.set(groundPlaneMessage);
   }

   @Override
   public Point3DReadOnly snapStep(double xPosition, double yPosition, double minimumZPosition)
   {
      GroundPlaneMessage groundPlaneMessage = this.groundPlaneMessage.get();

      if(groundPlaneMessage == null)
      {
         // fall back on support polygon height
         snappedPoint.set(xPosition, yPosition, centroidFrame.getTransformToWorldFrame().getTranslationZ());
      }
      else
      {
         // snap to ground plane estimate if provided
         plane.set(groundPlaneMessage.region_origin_, groundPlaneMessage.region_normal_);
         snappedPoint.set(xPosition, yPosition, plane.getZOnPlane(xPosition, yPosition));
      }

      return snappedPoint;
   }
}
