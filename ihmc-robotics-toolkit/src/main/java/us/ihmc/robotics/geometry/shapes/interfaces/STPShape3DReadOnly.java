package us.ihmc.robotics.geometry.shapes.interfaces;

import us.ihmc.euclid.shape.primitives.interfaces.Shape3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * Read-only interface for shapes that implement the sphere-torus-patches (STP) method to
 * make shapes strictly convex.
 * <p>
 * This data structure was implemented initially to improve continuity when performing collision
 * detection continuously over time. The overall data structure is pretty experimental still and
 * lack testing.
 * </p>
 * <p>
 * The approach is from <a href=
 * "http://sylvain.miossec.free.fr/work/publis/IEEE_TRO_Escande_Miossec_Kheddar.pdf">Strictly convex
 * hulls for computing continuous gradient proximity distances</a>
 * </p>
 * <p>
 * The general idea is to smooth out vertices, edges, and faces. For instance, for a box, a small
 * sphere is used for its vertices, a (very) large sphere is used for each face and meets the small
 * spheres such that the connection is continuous, the edges use properly sized torus such that the
 * small and large spheres are connecting in a continuous manner. The geometry used behind the scene
 * can be somewhat confusing but visualization of such shape is pretty self-explanatory, see the
 * examples in the <i>simulation-construction-set-visualizers</i> repository.
 * </p>
 * <p>
 * Instead of defining the radius for the small and large spheres, the user configure the smoothness
 * of an STP shape by defining a minimum margin and a maximum margin. The method used grows the
 * original shape, the more the shape is grown the smoother it is, until it pretty much look like a
 * sphere. The minimum margin represents the minimum distance from the original shape to the STP
 * version of the shape. When it is zero, vertices are not expanded into small spheres for instance.
 * The maximum margin in turn is the maximum distance from the original shape to the STP version of
 * the shape. This is main parameter for controlling the smoothness of the shape, for a high value
 * we obtain high smoothness but also the STP shape represents the original shape less accurately.
 * </p>
 * 
 * @author Sylvain Bertrand
 */
public interface STPShape3DReadOnly extends Shape3DReadOnly
{
   /**
    * Gets the current value for the minimum distance between the original shape and the STP shape.
    * <p>
    * The minimum margin is equivalent to the small radius.
    * </p>
    * 
    * @return the value for the minimum margin.
    */
   double getMinimumMargin();

   /**
    * Gets the current value for the maximum distance between the original shape and the STP shape.
    * <p>
    * The maximum margin is typically used to compute the large radius. It directly relates to the
    * smoothness of the STP shape, i.e. a smaller value relates to a sharper shape (less smooth), a
    * greater value relates to a more spherical shape (more smooth).
    * </p>
    * 
    * @return the value for the maximum margin.
    */
   double getMaximumMargin();

   /**
    * Gets the value used for the small spheres.
    * <p>
    * Small spheres are used for instance to enlarge the hull at vertices.
    * </p>
    * 
    * @return the value for the small radius.
    */
   double getSmallRadius();

   /**
    * Gets the value used for the large spheres.
    * <p>
    * Large spheres are used for instance to enlarge the hull at faces.
    * </p>
    * 
    * @return the value for the large radius.
    */
   double getLargeRadius();

   // This is to ensure that the default method is being overridden.
   @Override
   boolean getSupportingVertex(Vector3DReadOnly supportDirection, Point3DBasics supportingVertexToPack);

   // The following part of the API has not been implemented for STP shapes yet, let's prevent their use for now.

   @Override
   default boolean evaluatePoint3DCollision(Point3DReadOnly pointToCheck, Point3DBasics closestPointOnSurfaceToPack, Vector3DBasics normalAtClosestPointToPack)
   {
      throw new UnsupportedOperationException("Not supported for STP shape 3D");
   }

   @Override
   default double signedDistance(Point3DReadOnly point)
   {
      throw new UnsupportedOperationException("Not supported for STP shape 3D");
   }

   @Override
   default boolean isPointInside(Point3DReadOnly query, double epsilon)
   {
      throw new UnsupportedOperationException("Not supported for STP shape 3D");
   }

   @Override
   default boolean orthogonalProjection(Point3DReadOnly pointToProject, Point3DBasics projectionToPack)
   {
      throw new UnsupportedOperationException("Not supported for STP shape 3D");
   }
}
