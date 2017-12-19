package us.ihmc.commonWalkingControlModules.capturePoint.optimization.qpInput;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.footstepPlanning.polygonWiggling.PolygonWiggler;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.linearAlgebra.MatrixTools;

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
public class ConstraintToConvexRegion
{
   /** position offset of the constrained variable. */
   public final DenseMatrix64F positionOffset;

   /** matrix multiplier of the constrained variable. */
   public final DenseMatrix64F Aineq;
   /** desired convex region for the constrained variable. */
   public final DenseMatrix64F bineq;

   /** distance inside the convex region required for the constrained variable. */
   private double deltaInside = 0.0;

   /** convex polygon in which to constrain the variable. */
   private final ConvexPolygon2D convexPolygon = new ConvexPolygon2D();

   /**
    * Creates the Constraint To Convex Region. Refer to the class documentation: {@link ConstraintToConvexRegion}.
    *
    * @param maximumNumberOfVertices maximum number of vertices to initialize the convex region size.
    */
   public ConstraintToConvexRegion(int maximumNumberOfVertices)
   {
      positionOffset = new DenseMatrix64F(2, 1);

      Aineq = new DenseMatrix64F(maximumNumberOfVertices, maximumNumberOfVertices);
      bineq = new DenseMatrix64F(maximumNumberOfVertices, 1);
   }

   /**
    * Resets the convex region, requiring all the vertices to be readded. Should be called whenever
    * the shape of the convex region changes.
    */
   public void reset()
   {
      positionOffset.zero();
      convexPolygon.clear();

      Aineq.zero();
      bineq.zero();
   }

   /**
    * Adds a vertex to the convex region.
    *
    * @param vertex vertex to add.
    */
   public void addVertex(FramePoint2D vertex)
   {
      vertex.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());

      convexPolygon.addVertex(vertex);
   }

   public void addPolygon(FrameConvexPolygon2d polygon)
   {
      convexPolygon.addVertices(polygon.getConvexPolygon2d());
   }

   /**
    * Adds a vertex to the convex region.
    *
    * @param vertex vertex to add.
    */
   public void addVertex(FramePoint3D vertex)
   {
      vertex.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());

      convexPolygon.addVertex(vertex.getX(), vertex.getY());
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
   public void setPositionOffset(DenseMatrix64F offset)
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
      PolygonWiggler.convertToInequalityConstraints(convexPolygon, Aineq, bineq, deltaInside);
      CommonOps.multAdd(-1.0, Aineq, positionOffset, bineq);
   }

   /**
    * Gets the total size of the required inequality constraint.
    *
    * @return number of vertices.
    */
   public int getInequalityConstraintSize()
   {
      int numberOfVertices = convexPolygon.getNumberOfVertices();
      if (numberOfVertices > 2)
         return numberOfVertices;
      else if (numberOfVertices > 0)
         return 4;
      else
         return 0;
   }
}
