package us.ihmc.gdx.ui.collidables;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.gdx.ui.gizmo.Axis3DRotations;
import us.ihmc.gdx.ui.gizmo.DiscreteArrowRayIntersection;

import java.util.EnumMap;

public class GDXCollidableCoordinateFrame
{
   private final EnumMap<Axis3D, DiscreteArrowRayIntersection> arrows = new EnumMap<>(Axis3D.class);
   private final Axis3DRotations axis3DRotations = new Axis3DRotations();
   private final double length;

   public GDXCollidableCoordinateFrame(double length)
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
         RigidBodyTransform transform = new RigidBodyTransform();
         double arrowBodyLength = length;
         double arrowBodyRadius = 0.02 * length;
         double arrowHeadRadius = 0.05 * length;
         double arrowHeadLength = 0.10 * length;
         double zOffset = 0.0;
         DiscreteArrowRayIntersection arrowIntersection = arrows.get(axis);
         arrowIntersection.setupShapes(arrowBodyLength, arrowBodyRadius, arrowHeadRadius, arrowHeadLength, zOffset, transform);
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
}
