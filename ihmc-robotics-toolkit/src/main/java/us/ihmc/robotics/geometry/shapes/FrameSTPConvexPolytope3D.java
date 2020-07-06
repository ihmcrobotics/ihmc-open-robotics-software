package us.ihmc.robotics.geometry.shapes;

import static us.ihmc.euclid.tools.EuclidCoreIOTools.DEFAULT_FORMAT;

import java.util.List;

import us.ihmc.euclid.geometry.interfaces.Vertex3DSupplier;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameShape3DPoseBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameBoundingBox3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameShape3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVertex3DSupplier;
import us.ihmc.euclid.referenceFrame.polytope.FrameConvexPolytope3D;
import us.ihmc.euclid.referenceFrame.polytope.FrameFace3D;
import us.ihmc.euclid.referenceFrame.polytope.FrameHalfEdge3D;
import us.ihmc.euclid.referenceFrame.polytope.FrameVertex3D;
import us.ihmc.euclid.referenceFrame.polytope.interfaces.FrameConvexPolytope3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameShapeIOTools;
import us.ihmc.euclid.shape.convexPolytope.interfaces.ConvexPolytope3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.tools.EuclidPolytopeConstructionTools;
import us.ihmc.euclid.tools.EuclidCoreFactories;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.geometry.shapes.STPShape3DTools.STPConvexPolytope3DSupportingVertexCalculator;
import us.ihmc.robotics.geometry.shapes.interfaces.FrameSTPConvexPolytope3DReadOnly;
import us.ihmc.robotics.geometry.shapes.interfaces.STPConvexPolytope3DReadOnly;
import us.ihmc.robotics.geometry.shapes.interfaces.STPShape3DBasics;
import us.ihmc.robotics.geometry.shapes.interfaces.STPShape3DReadOnly;

/**
 * Convex polytope that implements the sphere-torus-patches (STP) method to make shapes strictly
 * convex.
 * <p>
 * <strong> WARNING: STP convex polytope does not properly cover all scenarios and may result in a
 * non-convex shape. A STP convex polytope should always be visualized first and validate its
 * geometry, see the examples in the <i>simulation-construction-set-visualizers</i> repository. For
 * now, it is recommended to stick with primitive shapes. </strong>
 * </p>
 * 
 * @see STPShape3DReadOnly
 * @author Sylvain Bertrand
 */
public class FrameSTPConvexPolytope3D implements FrameSTPConvexPolytope3DReadOnly, FrameShape3DBasics, STPShape3DBasics
{
   private double minimumMargin, maximumMargin;
   private double largeRadius, smallRadius;
   private final FrameConvexPolytope3D rawConvexPolytope3D;
   private final FrameBoundingBox3DReadOnly boundingBox;
   private final STPConvexPolytope3DSupportingVertexCalculator supportingVertexCalculator = new STPConvexPolytope3DSupportingVertexCalculator();

   private boolean stpRadiiDirty = true;

   /**
    * Creates a new empty convex polytope initializes its reference frame to
    * {@link ReferenceFrame#getWorldFrame()}.
    */
   public FrameSTPConvexPolytope3D()
   {
      this(ReferenceFrame.getWorldFrame());
   }

   /**
    * Creates a new empty convex polytope and initializes its reference frame.
    *
    * @param referenceFrame this polytope initial frame.
    */
   public FrameSTPConvexPolytope3D(ReferenceFrame referenceFrame)
   {
      this(referenceFrame, EuclidPolytopeConstructionTools.DEFAULT_CONSTRUCTION_EPSILON);
   }

   /**
    * Creates a new empty convex polytope.
    *
    * @param referenceFrame      this polytope initial frame.
    * @param constructionEpsilon tolerance used when adding vertices to a convex polytope to trigger a
    *                            series of edge-cases.
    */
   public FrameSTPConvexPolytope3D(ReferenceFrame referenceFrame, double constructionEpsilon)
   {
      rawConvexPolytope3D = new FrameConvexPolytope3D(referenceFrame, constructionEpsilon);
      Point3DReadOnly rawMinPoint = rawConvexPolytope3D.getBoundingBox().getMinPoint();
      Point3DReadOnly rawMaxPoint = rawConvexPolytope3D.getBoundingBox().getMaxPoint();
      Point3DReadOnly minPoint = EuclidCoreFactories.newLinkedPoint3DReadOnly(() -> rawMinPoint.getX() + maximumMargin,
                                                                              () -> rawMinPoint.getY() + maximumMargin,
                                                                              () -> rawMinPoint.getZ() + maximumMargin);
      Point3DReadOnly maxPoint = EuclidCoreFactories.newLinkedPoint3DReadOnly(() -> rawMaxPoint.getX() - maximumMargin,
                                                                              () -> rawMaxPoint.getY() - maximumMargin,
                                                                              () -> rawMaxPoint.getZ() - maximumMargin);
      boundingBox = STPShape3DTools.newLinkedFrameBoundingBox3DReadOnly(this, minPoint, maxPoint);
   }

   /**
    * Creates a new convex polytope and adds vertices provided by the given supplier.
    *
    * @param referenceFrame   this polytope initial frame.
    * @param vertex3DSupplier the vertex supplier to get the vertices to add to this convex polytope.
    */
   public FrameSTPConvexPolytope3D(ReferenceFrame referenceFrame, Vertex3DSupplier vertex3DSupplier)
   {
      this(referenceFrame);
      addVertices(vertex3DSupplier);
   }

   /**
    * Creates a new convex polytope, adds vertices provided by the given supplier, its reference frame
    * is initialized to match the reference frame of the vertex supplier.
    *
    * @param vertex3DSupplier the vertex supplier to get the vertices to add to this convex polytope.
    */
   public FrameSTPConvexPolytope3D(FrameVertex3DSupplier vertex3DSupplier)
   {
      this(vertex3DSupplier.getReferenceFrame(), vertex3DSupplier);
   }

   /**
    * Creates a new convex polytope, adds vertices provided by the given supplier.
    *
    * @param referenceFrame      this polytope initial frame.
    * @param vertex3DSupplier    the vertex supplier to get the vertices to add to this convex
    *                            polytope.
    * @param constructionEpsilon tolerance used when adding vertices to a convex polytope to trigger a
    *                            series of edge-cases.
    */
   public FrameSTPConvexPolytope3D(ReferenceFrame referenceFrame, Vertex3DSupplier vertex3DSupplier, double constructionEpsilon)
   {
      this(referenceFrame, constructionEpsilon);
      addVertices(vertex3DSupplier);
   }

   /**
    * Creates a new convex polytope, adds vertices provided by the given supplier, its reference frame
    * is initialized to match the reference frame of the vertex supplier.
    *
    * @param vertex3DSupplier    the vertex supplier to get the vertices to add to this convex
    *                            polytope.
    * @param constructionEpsilon tolerance used when adding vertices to a convex polytope to trigger a
    *                            series of edge-cases.
    */
   public FrameSTPConvexPolytope3D(FrameVertex3DSupplier vertex3DSupplier, double constructionEpsilon)
   {
      this(vertex3DSupplier.getReferenceFrame(), vertex3DSupplier, constructionEpsilon);
   }

   /**
    * Creates a new convex polytope identical to {@code other}.
    *
    * @param referenceFrame this polytope initial frame.
    * @param other          the other convex polytope to copy. Not modified.
    */
   public FrameSTPConvexPolytope3D(ReferenceFrame referenceFrame, ConvexPolytope3DReadOnly other)
   {
      this(referenceFrame, other.getConstructionEpsilon());
      set(other);
   }

   /**
    * Creates a new convex polytope identical to {@code other}.
    *
    * @param referenceFrame this polytope initial frame.
    * @param other          the other convex polytope to copy. Not modified.
    */
   public FrameSTPConvexPolytope3D(ReferenceFrame referenceFrame, STPConvexPolytope3DReadOnly other)
   {
      this(referenceFrame, other.getConstructionEpsilon());
      set(other);
   }

   /**
    * Creates a new convex polytope identical to {@code other}.
    *
    * @param other the other convex polytope to copy. Not modified.
    */
   public FrameSTPConvexPolytope3D(FrameConvexPolytope3DReadOnly other)
   {
      this(other.getReferenceFrame(), other);
   }

   /**
    * Creates a new convex polytope identical to {@code other}.
    *
    * @param other the other convex polytope to copy. Not modified.
    */
   public FrameSTPConvexPolytope3D(FrameSTPConvexPolytope3DReadOnly other)
   {
      this(other.getReferenceFrame(), other);
   }

   /**
    * Sets this convex polytope to be identical to {@code other}.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param other the other convex polytope to copy. Not modified.
    */
   public void set(ConvexPolytope3DReadOnly other)
   {
      rawConvexPolytope3D.set(other);
      stpRadiiDirty = true;
   }

   /**
    * Sets this convex polytope to be identical to {@code other}.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param other the other convex polytope to copy. Not modified.
    */
   public void set(STPConvexPolytope3DReadOnly other)
   {
      rawConvexPolytope3D.set(other);
      minimumMargin = other.getMinimumMargin();
      maximumMargin = other.getMaximumMargin();
      stpRadiiDirty = true;
   }

   /**
    * Sets this convex polytope to be identical to {@code other}.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param other the other polytope to copy. Not modified.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame {@code this}.
    */
   public void set(FrameConvexPolytope3DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      set((ConvexPolytope3DReadOnly) other);
   }

   /**
    * Sets this convex polytope to be identical to {@code other}.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param other the other polytope to copy. Not modified.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame {@code this}.
    */
   public void set(FrameSTPConvexPolytope3DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      set((STPConvexPolytope3DReadOnly) other);
   }

   /**
    * Sets this convex polytope to be identical to {@code other}.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param other the other polytope to copy. Not modified.
    */
   public void setIncludingFrame(FrameConvexPolytope3DReadOnly other)
   {
      setReferenceFrame(other.getReferenceFrame());
      set((ConvexPolytope3DReadOnly) other);
   }

   /**
    * Sets this convex polytope to be identical to {@code other}.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param other the other polytope to copy. Not modified.
    */
   public void setIncludingFrame(FrameSTPConvexPolytope3DReadOnly other)
   {
      setReferenceFrame(other.getReferenceFrame());
      set((STPConvexPolytope3DReadOnly) other);
   }

   /** {@inheritDoc} */
   @Override
   public void setReferenceFrame(ReferenceFrame referenceFrame)
   {
      rawConvexPolytope3D.setReferenceFrame(referenceFrame);
   }

   /** {@inheritDoc} */
   @Override
   public void changeFrame(ReferenceFrame desiredFrame)
   {
      rawConvexPolytope3D.changeFrame(desiredFrame);
   }

   @Override
   public void setToNaN()
   {
      rawConvexPolytope3D.setToNaN();
      stpRadiiDirty = true;
   }

   @Override
   public void setToZero()
   {
      rawConvexPolytope3D.setToZero();
      stpRadiiDirty = true;
   }

   /**
    * Adds a new vertex to this convex polytope.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param vertexToAdd the vertex that is to be added to the convex polytope. Not modified.
    * @return {@code true} if the vertex was added to this convex polytope, {@code false} if it was
    *         rejected.
    */
   public boolean addVertex(Point3DReadOnly vertexToAdd)
   {
      return addVertices(Vertex3DSupplier.asVertex3DSupplier(vertexToAdd));
   }

   /**
    * Adds a new vertex to this convex polytope.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param vertexToAdd the vertex that is to be added to the convex polytope. Not modified.
    * @return {@code true} if the vertex was added to this convex polytope, {@code false} if it was
    *         rejected.
    */
   public boolean addVertex(FramePoint3DReadOnly vertexToAdd)
   {
      checkReferenceFrameMatch(vertexToAdd);
      return addVertex((Point3DReadOnly) vertexToAdd);
   }

   /**
    * Adds a new vertex to this convex polytope.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param vertex3DSupplier the vertex supplier to get the vertices to add to this convex polytope.
    * @return {@code true} if the vertex was added to this convex polytope, {@code false} if it was
    *         rejected.
    */
   public boolean addVertices(Vertex3DSupplier vertex3DSupplier)
   {
      boolean wasAdded = rawConvexPolytope3D.addVertices(vertex3DSupplier);
      if (wasAdded)
         stpRadiiDirty = true;
      return wasAdded;
   }

   /**
    * Adds a new vertex to this convex polytope.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param vertex3DSupplier the vertex supplier to get the vertices to add to this convex polytope.
    * @return {@code true} if the vertex was added to this convex polytope, {@code false} if it was
    *         rejected.
    */
   public boolean addVertices(FrameVertex3DSupplier vertex3DSupplier)
   {
      checkReferenceFrameMatch(vertex3DSupplier);
      return addVertices((Vertex3DSupplier) vertex3DSupplier);
   }

   @Override
   public double getMinimumMargin()
   {
      return minimumMargin;
   }

   @Override
   public double getMaximumMargin()
   {
      return maximumMargin;
   }

   @Override
   public double getSmallRadius()
   {
      updateRadii();
      return smallRadius;
   }

   @Override
   public double getLargeRadius()
   {
      updateRadii();
      return largeRadius;
   }

   @Override
   public void setMargins(double minimumMargin, double maximumMargin)
   {
      if (maximumMargin <= minimumMargin)
         throw new IllegalArgumentException("The maximum margin has to be strictly greater that the minimum margin, max margin: " + maximumMargin
               + ", min margin: " + minimumMargin);
      this.minimumMargin = minimumMargin;
      this.maximumMargin = maximumMargin;
      stpRadiiDirty = true;
   }

   /**
    * <pre>
    * r = h
    *      r^2 - g^2 - 0.25 * l<sub>max</sub>
    * R = ------------------------
    *           2 * (r - g)
    * </pre>
    *
    * where:
    * <ul>
    * <li><tt>R</tt> is {@link #largeRadius}
    * <li><tt>r</tt> is {@link #smallRadius}
    * <li><tt>h</tt> is {@link #minimumMargin}
    * <li><tt>g</tt> is {@link #maximumMargin}
    * <li><tt>l<sub>max</max></tt> is the maximum edge length that needs to be covered by the large
    * bounding sphere.
    * </ul>
    */
   protected void updateRadii()
   {
      if (!stpRadiiDirty)
         return;

      stpRadiiDirty = false;

      if (minimumMargin == 0.0 && maximumMargin == 0.0)
      {
         smallRadius = Double.NaN;
         largeRadius = Double.NaN;
      }
      else
      {
         smallRadius = minimumMargin;
         largeRadius = STPShape3DTools.computeLargeRadiusFromMargins(minimumMargin,
                                                                     maximumMargin,
                                                                     STPShape3DTools.computeConvexPolytope3DMaximumEdgeLengthSquared(rawConvexPolytope3D));
      }
   }

   @Override
   public boolean containsNaN()
   {
      return rawConvexPolytope3D.containsNaN();
   }

   @Override
   public double getVolume()
   {
      return rawConvexPolytope3D.getVolume();
   }

   @Override
   public List<FrameFace3D> getFaces()
   {
      return rawConvexPolytope3D.getFaces();
   }

   @Override
   public List<FrameHalfEdge3D> getHalfEdges()
   {
      return rawConvexPolytope3D.getHalfEdges();
   }

   @Override
   public List<FrameVertex3D> getVertices()
   {
      return rawConvexPolytope3D.getVertices();
   }

   @Override
   public double getConstructionEpsilon()
   {
      return rawConvexPolytope3D.getConstructionEpsilon();
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return rawConvexPolytope3D.getReferenceFrame();
   }

   @Override
   public FrameBoundingBox3DReadOnly getBoundingBox()
   {
      return boundingBox;
   }

   @Override
   public FrameSTPConvexPolytope3D copy()
   {
      return new FrameSTPConvexPolytope3D(this);
   }

   @Override
   public FramePoint3DReadOnly getCentroid()
   {
      return rawConvexPolytope3D.getCentroid();
   }

   @Override
   public boolean getSupportingVertex(Vector3DReadOnly supportDirection, Point3DBasics supportingVertexToPack)
   {
      return supportingVertexCalculator.getSupportingVertex(this, getSmallRadius(), getLargeRadius(), supportDirection, supportingVertexToPack);
   }

   @Override
   public FixedFrameShape3DPoseBasics getPose()
   {
      return null;
   }

   /** {@inheritDoc} */
   @Override
   public void applyTransform(Transform transform)
   {
      rawConvexPolytope3D.applyTransform(transform);
   }

   /** {@inheritDoc} */
   @Override
   public void applyInverseTransform(Transform transform)
   {
      rawConvexPolytope3D.applyInverseTransform(transform);
   }

   /**
    * Tests if the given {@code object}'s class is the same as this, in which case the method returns
    * {@link #equals(FrameSTPConvexPolytope3DReadOnly)}, it returns {@code false} otherwise.
    *
    * @param object the object to compare against this. Not modified.
    * @return {@code true} if {@code object} and this are exactly equal, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object object)
   {
      if (object instanceof FrameSTPConvexPolytope3DReadOnly)
         return FrameSTPConvexPolytope3DReadOnly.super.equals((FrameSTPConvexPolytope3DReadOnly) object);
      else
         return false;
   }

   /**
    * Calculates and returns a hash code value from the value of each component of this convex polytope
    * 3D.
    *
    * @return the hash code value for this convex polytope 3D.
    */
   @Override
   public int hashCode()
   {
      long hash = EuclidHashCodeTools.combineHashCode(rawConvexPolytope3D.hashCode(), EuclidHashCodeTools.toLongHashCode(minimumMargin, maximumMargin));
      return EuclidHashCodeTools.toIntHashCode(hash);
   }

   /**
    * Provides a {@code String} representation of this convex polytope 3D as follows:
    *
    * <pre>
    * STP Convex polytope 3D: number of: [faces: 4, edges: 12, vertices: 4
    * Face list:
    *    centroid: ( 0.582, -0.023,  0.160 ), normal: ( 0.516, -0.673,  0.530 )
    *    centroid: ( 0.420,  0.176,  0.115 ), normal: (-0.038,  0.895, -0.444 )
    *    centroid: ( 0.264, -0.253, -0.276 ), normal: ( 0.506,  0.225, -0.833 )
    *    centroid: ( 0.198, -0.176, -0.115 ), normal: (-0.643, -0.374,  0.668 )
    * Edge list:
    *    [( 0.674,  0.482,  0.712 ); ( 0.870,  0.251,  0.229 )]
    *    [( 0.870,  0.251,  0.229 ); ( 0.204, -0.803, -0.461 )]
    *    [( 0.204, -0.803, -0.461 ); ( 0.674,  0.482,  0.712 )]
    *    [( 0.870,  0.251,  0.229 ); ( 0.674,  0.482,  0.712 )]
    *    [( 0.674,  0.482,  0.712 ); (-0.283, -0.207, -0.595 )]
    *    [(-0.283, -0.207, -0.595 ); ( 0.870,  0.251,  0.229 )]
    *    [( 0.204, -0.803, -0.461 ); ( 0.870,  0.251,  0.229 )]
    *    [( 0.870,  0.251,  0.229 ); (-0.283, -0.207, -0.595 )]
    *    [(-0.283, -0.207, -0.595 ); ( 0.204, -0.803, -0.461 )]
    *    [( 0.674,  0.482,  0.712 ); ( 0.204, -0.803, -0.461 )]
    *    [( 0.204, -0.803, -0.461 ); (-0.283, -0.207, -0.595 )]
    *    [(-0.283, -0.207, -0.595 ); ( 0.674,  0.482,  0.712 )]
    * Vertex list:
    *    ( 0.674,  0.482,  0.712 )
    *    ( 0.870,  0.251,  0.229 )
    *    ( 0.204, -0.803, -0.461 )
    *    (-0.283, -0.207, -0.595 )
    * worldFrame
    * small radius: 0.001, large radius: 1.000
    * </pre>
    *
    * @return the {@code String} representing this convex polytope 3D.
    */
   @Override
   public String toString()
   {
      String stpSuffix = String.format("\nsmall radius: " + DEFAULT_FORMAT + ", large radius: " + DEFAULT_FORMAT + "]", getSmallRadius(), getLargeRadius());
      return "STP" + EuclidFrameShapeIOTools.getFrameConvexPolytope3DString(this) + stpSuffix;
   }
}
