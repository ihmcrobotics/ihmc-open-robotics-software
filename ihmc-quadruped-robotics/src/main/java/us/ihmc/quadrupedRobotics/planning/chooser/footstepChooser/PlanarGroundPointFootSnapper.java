package us.ihmc.quadrupedRobotics.planning.chooser.footstepChooser;

import controller_msgs.msg.dds.QuadrupedGroundPlaneMessage;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.quadrupedRobotics.estimator.referenceFrames.QuadrupedReferenceFrames;

import java.util.concurrent.atomic.AtomicReference;

public class PlanarGroundPointFootSnapper implements PointFootSnapper
{
   private final AtomicReference<QuadrupedGroundPlaneMessage> groundPlaneMessage = new AtomicReference<>();
   private final Plane3D plane = new Plane3D();
   private final Point3D snappedPoint = new Point3D();

   private final ReferenceFrame centroidFrame;

   public PlanarGroundPointFootSnapper(QuadrupedReferenceFrames referenceFrames, PacketCommunicator packetCommunicator)
   {
      this.centroidFrame = referenceFrames.getCenterOfFeetZUpFrameAveragingLowestZHeightsAcrossEnds();
      packetCommunicator.attachListener(QuadrupedGroundPlaneMessage.class, groundPlaneMessage::set);
   }

   @Override
   public Point3DReadOnly snapStep(double xPosition, double yPosition)
   {
      QuadrupedGroundPlaneMessage groundPlaneMessage = this.groundPlaneMessage.get();

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
