package us.ihmc.rdx.ui.collidables;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.interaction.Axis3DRotations;
import us.ihmc.robotics.interaction.DiscreteArrowRayIntersection;

import java.util.EnumMap;

public class RDXCoordinateFrameIntersection
{
   private final EnumMap<Axis3D, DiscreteArrowRayIntersection> arrows = new EnumMap<>(Axis3D.class);
   private final Axis3DRotations axis3DRotations = new Axis3DRotations();
   private final FramePose3D tempFramePose = new FramePose3D();
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();
   private final double length;

   public RDXCoordinateFrameIntersection(double length)
   {
      this.length = length;
      for (Axis3D axis : Axis3D.values)
      {
         arrows.put(axis, new DiscreteArrowRayIntersection());
      }
   }

   public double intersect(ReferenceFrame coordinateFrame, Line3DReadOnly pickRay)
   {
      double closestCollisionDistance = Double.NaN;
      for (Axis3D axis : Axis3D.values)
      {
         tempFramePose.setToZero(coordinateFrame);
         tempFramePose.getOrientation().set(axis3DRotations.get(axis));
         tempFramePose.changeFrame(ReferenceFrame.getWorldFrame());
         tempFramePose.get(tempTransform);

         double arrowBodyLength = length;
         double arrowBodyRadius = 0.02 * length;
         double arrowHeadRadius = 0.05 * length;
         double arrowHeadLength = 0.10 * length;
         double zOffset = 0.5 * arrowBodyLength;
         DiscreteArrowRayIntersection arrowIntersection = arrows.get(axis);
         arrowIntersection.update(arrowBodyLength, arrowBodyRadius, arrowHeadRadius, arrowHeadLength, zOffset, tempTransform);
         double distance = arrowIntersection.intersect(pickRay, 100, true);

         if (!Double.isNaN(distance) && Double.isNaN(closestCollisionDistance))
         {
            closestCollisionDistance = distance;
         }
         else if (!Double.isNaN(distance) && !Double.isNaN(closestCollisionDistance) && distance < closestCollisionDistance)
         {
            closestCollisionDistance = distance;
         }
      }
      return closestCollisionDistance;
   }

   // TODO: Add intersect(Point3D) for VR
}
