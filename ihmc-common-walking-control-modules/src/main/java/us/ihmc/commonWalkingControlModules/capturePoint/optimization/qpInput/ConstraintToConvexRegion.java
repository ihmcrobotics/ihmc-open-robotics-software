package us.ihmc.commonWalkingControlModules.capturePoint.optimization.qpInput;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

import us.ihmc.commonWalkingControlModules.polygonWiggling.PolygonWiggler;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.robotics.geometry.ConvexPolygonScaler;
import us.ihmc.robotics.geometry.PlanarRegion;

/**
 * Class that represents constraining a variable to lie in a convex region. This constraint has the form<br>
 *    A<sub>in</sub>x <= b<sub>in</sub><br>
 * where x is the variable to be constrained. This class also allows the formulation<br>
 *    A<sub>in</sub>(x - a) <= b<sub>in</sub>,<br>
 * allowing the constrained variable to be offset inside the polygon. <br>
 * Note that this representation
 * relies on the ability to represent lying in a convex polygon as a linear inequality constraint.
 * Higher complexity convex regions can be represented, but may require more complex constraints. As
 * an example, the variable can be constrained to a circle, but this requires a quadratic constraint.
 */
public class ConstraintToConvexRegion extends ICPInequalityInput
{
   /** position offset of the constrained variable. */
   public final DMatrixRMaj positionOffset;

   /** distance inside the convex region required for the constrained variable. */
   private double deltaInside = 0.0;

   /** convex polygon in which to constrain the variable. */
   private final ConvexPolygon2D convexPolygon = new ConvexPolygon2D();

   private final ConvexPolygon2D scaledConvexPolygon = new ConvexPolygon2D();

   private final ConvexPolygonScaler scaler = new ConvexPolygonScaler();

   /**
    * Creates the Constraint To Convex Region. Refer to the class documentation: {@link ConstraintToConvexRegion}.
    *
    * @param maximumNumberOfVertices maximum number of vertices to initialize the convex region size.
    */
   public ConstraintToConvexRegion(int maximumNumberOfVertices)
   {
      super(maximumNumberOfVertices, maximumNumberOfVertices);
      positionOffset = new DMatrixRMaj(2, 1);
   }

   /**
    * Resets the convex region, requiring all the vertices to be readded. Should be called whenever
    * the shape of the convex region changes.
    */
   public void reset()
   {
      super.reset();
      positionOffset.zero();
      convexPolygon.clear();
      scaledConvexPolygon.clear();
   }

   /**
    * Adds a vertex to the convex region.
    *
    * @param vertex vertex to add.
    */
   public void addVertex(FramePoint2DReadOnly vertex)
   {
      vertex.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());

      convexPolygon.addVertex(vertex);
   }

   public void addPolygon(ConvexPolygon2DReadOnly polygon)
   {
      convexPolygon.addVertices(polygon);
   }

   /**
    * Adds a vertex to the convex region.
    *
    * @param vertex vertex to add.
    */
   public void addVertex(FramePoint3DReadOnly vertex)
   {
      vertex.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());

      convexPolygon.addVertex(vertex);
   }

   /**
    * Sets the convex polygon for the constraint as the convex hull of a planar region.
    *
    * @param planarRegion planar region to constrain to.
    * @return success
    */
   public boolean addPlanarRegion(PlanarRegion planarRegion, double deltaInside)
   {
      if (planarRegion == null)
         return false;

      convexPolygon.set(planarRegion.getConvexHull());
      this.deltaInside = deltaInside;
      return true;
   }

   public boolean addPlanarRegion(ConvexPolygon2DReadOnly convexPolygon, double deltaInside)
   {
      if (convexPolygon == null)
         return false;

      this.convexPolygon.set(convexPolygon);
      this.deltaInside = deltaInside;
      return true;
   }

   public boolean addPlanarRegion(ConvexPolygon2DReadOnly convexPolygon)
   {
      if (convexPolygon == null)
         return false;

      this.convexPolygon.set(convexPolygon);
      return true;
   }

   /**
    * Sets the polygon for the convex constraint to use. Should be called after
    * all the vertices have been added using {@link #addVertex(FramePoint3D)} or
    * {@link #addVertex(FramePoint2D)}.
    */
   public void setPolygon()
   {
      convexPolygon.update();
   }

   /**
    * Sets the position offset of the variable that is being constrained. This is
    * useful when the constrained variable is a vector from a point, such as a feedback
    * modification.
    * @param offset offset from the constrained variable.
    */
   public void setPositionOffset(DMatrixRMaj offset)
   {
      MatrixTools.setMatrixBlock(positionOffset, 0, 0, offset, 0, 0, 2, 1, 1.0);
   }

   /**
    * Sets a desired distance inside the convex region for the constrained variable to
    * be. It is useful to use this as some small value, so that the constrained variable
    * does not sit completely on the bounds of the convex region.
    *
    * @param deltaInside distance inside the convex region.
    */
   public void setDeltaInside(double deltaInside)
   {
      this.deltaInside = deltaInside;
   }

   /**
    * Formulates the actual constraint from the {@link #convexPolygon}
    */
   public void formulateConstraint()
   {
      scaler.scaleConvexPolygon(convexPolygon, deltaInside, scaledConvexPolygon);
      PolygonWiggler.convertToInequalityConstraints(scaledConvexPolygon, Aineq, bineq, 0.0);
      CommonOps_DDRM.multAdd(-1.0, Aineq, positionOffset, bineq);
   }

   /**
    * Gets the total size of the required inequality constraint.
    *
    * @return number of vertices.
    */
   public int getInequalityConstraintSize()
   {
      /*
       * Ideally, the constraint size should always be determined from the scaledConvexPolygon as it is
       * the one used to formulate the constraint and might have a different number of vertices than the
       * convexPolygon (in rare occasion such as when a foot has tiny support polygon as for instance when
       * doing toe-off). However, this method is called before formulating the constraint as a
       * quick-check, in which case it is fine to use the convexPolygon instead. Since during reset the
       * scaledConvexPolygon is cleared, and it is only updated once formulating the constraint, we can
       * use the fact that it is empty as test for whether or not to use it.
       */
      ConvexPolygon2D polygon = scaledConvexPolygon.isEmpty() ? convexPolygon : scaledConvexPolygon;
      int numberOfVertices = polygon.getNumberOfVertices();
      if (numberOfVertices > 2)
         return numberOfVertices;
      else if (numberOfVertices > 0)
         return 4;
      else
         return 0;
   }

}
