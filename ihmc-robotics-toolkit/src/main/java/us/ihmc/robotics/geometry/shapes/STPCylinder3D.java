package us.ihmc.robotics.geometry.shapes;

import static us.ihmc.euclid.tools.EuclidCoreIOTools.DEFAULT_FORMAT;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.shape.primitives.interfaces.Cylinder3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DChangeListener;
import us.ihmc.euclid.shape.tools.EuclidShapeIOTools;
import us.ihmc.euclid.tools.EuclidCoreFactories;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.tuple3D.UnitVector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.UnitVector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.geometry.shapes.STPShape3DTools.STPCylinder3DSupportingVertexCalculator;
import us.ihmc.robotics.geometry.shapes.interfaces.STPCylinder3DBasics;
import us.ihmc.robotics.geometry.shapes.interfaces.STPCylinder3DReadOnly;

public class STPCylinder3D implements STPCylinder3DBasics
{
   private final List<Shape3DChangeListener> changeListeners = new ArrayList<>();
   private double minimumMargin, maximumMargin;
   private double largeRadius, smallRadius;
   private final STPCylinder3DSupportingVertexCalculator supportingVertexCalculator = new STPCylinder3DSupportingVertexCalculator();

   /** Position of this cylinder's center. */
   private final Point3DBasics position = EuclidCoreFactories.newObservablePoint3DBasics((axis, value) -> notifyChangeListeners(), null);
   /** Axis of revolution of this cylinder. */
   private final UnitVector3DBasics axis = EuclidCoreFactories.newObservableUnitVector3DBasics((axis, value) -> notifyChangeListeners(),
                                                                                               null,
                                                                                               new UnitVector3D(Axis3D.Z));

   /** This cylinder radius. */
   private double radius;
   /** This cylinder length. */
   private double length;
   /** This cylinder half-length. */
   private double halfLength;

   /** Position of the top half-sphere center linked to this cylinder properties. */
   private final Point3DReadOnly topCenter = EuclidCoreFactories.newLinkedPoint3DReadOnly(() -> halfLength * axis.getX() + position.getX(),
                                                                                          () -> halfLength * axis.getY() + position.getY(),
                                                                                          () -> halfLength * axis.getZ() + position.getZ());
   /** Position of the bottom half-sphere center linked to this cylinder properties. */
   private final Point3DReadOnly bottomCenter = EuclidCoreFactories.newLinkedPoint3DReadOnly(() -> -halfLength * axis.getX() + position.getX(),
                                                                                             () -> -halfLength * axis.getY() + position.getY(),
                                                                                             () -> -halfLength * axis.getZ() + position.getZ());

   private boolean stpRadiiDirty = true;

   /**
    * Creates a new cylinder which axis is along the z-axis, a length of 1, and radius of 0.5.
    */
   public STPCylinder3D()
   {
      this(1.0, 0.5);
   }

   /**
    * Creates a new cylinder which axis is along the z-axis and initializes its size.
    *
    * @param length the length of this cylinder.
    * @param radius the radius of this cylinder.
    * @throws IllegalArgumentException if {@code length} or {@code radius} is negative.
    */
   public STPCylinder3D(double length, double radius)
   {
      setSize(length, radius);
      setupListeners();
   }

   /**
    * Creates a new cylinder 3D and initializes its pose and size.
    *
    * @param position the position of the center. Not modified.
    * @param axis     the axis of revolution. Not modified.
    * @param length   the length of this cylinder.
    * @param radius   the radius of this cylinder.
    * @throws IllegalArgumentException if {@code length} or {@code radius} is negative.
    */
   public STPCylinder3D(Point3DReadOnly position, Vector3DReadOnly axis, double length, double radius)
   {
      set(position, axis, length, radius);
      setupListeners();
   }

   /**
    * Creates a new cylinder 3D identical to {@code other}.
    *
    * @param other the other cylinder to copy. Not modified.
    */
   public STPCylinder3D(Cylinder3DReadOnly other)
   {
      set(other);
      setupListeners();
   }

   /**
    * Creates a new cylinder 3D identical to {@code other}.
    *
    * @param other the other cylinder to copy. Not modified.
    */
   public STPCylinder3D(STPCylinder3DReadOnly other)
   {
      set(other);
      setupListeners();
   }

   private void setupListeners()
   {
      addChangeListener(() -> stpRadiiDirty = true);
   }

   /** {@inheritDoc} */
   @Override
   public void setRadius(double radius)
   {
      if (radius < 0.0)
         throw new IllegalArgumentException("The radius of a Cylinder3D cannot be negative: " + radius);
      this.radius = radius;
      notifyChangeListeners();
   }

   /** {@inheritDoc} */
   @Override
   public void setLength(double length)
   {
      if (length < 0.0)
         throw new IllegalArgumentException("The length of a Cylinder3D cannot be negative: " + length);
      this.length = length;
      halfLength = 0.5 * length;
      notifyChangeListeners();
   }

   /** {@inheritDoc} */
   @Override
   public double getRadius()
   {
      return radius;
   }

   /** {@inheritDoc} */
   @Override
   public double getLength()
   {
      return length;
   }

   /** {@inheritDoc} */
   @Override
   public double getHalfLength()
   {
      return halfLength;
   }

   /** {@inheritDoc} */
   @Override
   public Point3DBasics getPosition()
   {
      return position;
   }

   /** {@inheritDoc} */
   @Override
   public UnitVector3DBasics getAxis()
   {
      return axis;
   }

   /** {@inheritDoc} */
   @Override
   public Point3DReadOnly getTopCenter()
   {
      return topCenter;
   }

   /** {@inheritDoc} */
   @Override
   public Point3DReadOnly getBottomCenter()
   {
      return bottomCenter;
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
         largeRadius = STPShape3DTools.computeLargeRadiusFromMargins(minimumMargin, maximumMargin, EuclidCoreTools.square(length));
      }
   }

   @Override
   public boolean getSupportingVertex(Vector3DReadOnly supportDirection, Point3DBasics supportingVertexToPack)
   {
      return supportingVertexCalculator.getSupportingVertex(this, getSmallRadius(), getLargeRadius(), supportDirection, supportingVertexToPack);
   }

   /**
    * Notifies the internal listeners that this shape has changed.
    */
   public void notifyChangeListeners()
   {
      for (int i = 0; i < changeListeners.size(); i++)
      {
         changeListeners.get(i).changed();
      }
   }

   /**
    * Registers a list of listeners to be notified when this shape changes.
    *
    * @param listeners the listeners to register.
    */
   public void addChangeListeners(List<? extends Shape3DChangeListener> listeners)
   {
      for (int i = 0; i < listeners.size(); i++)
      {
         addChangeListener(listeners.get(i));
      }
   }

   /**
    * Registers a listener to be notified when this shape changes.
    *
    * @param listener the listener to register.
    */
   public void addChangeListener(Shape3DChangeListener listener)
   {
      changeListeners.add(listener);
   }

   /**
    * Removes a previously registered listener.
    * <p>
    * This listener will no longer be notified of changes from this pose.
    * </p>
    *
    * @param listener the listener to remove.
    * @return {@code true} if the listener was removed successful, {@code false} if the listener could
    *         not be found.
    */
   public boolean removeChangeListener(Shape3DChangeListener listener)
   {
      return changeListeners.remove(listener);
   }

   @Override
   public STPCylinder3D copy()
   {
      return new STPCylinder3D(this);
   }

   /**
    * Tests if the given {@code object}'s class is the same as this, in which case the method returns
    * {@link #equals(STPCylinder3DReadOnly)}, it returns {@code false} otherwise.
    *
    * @param object the object to compare against this. Not modified.
    * @return {@code true} if {@code object} and this are exactly equal, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object object)
   {
      if (object instanceof STPCylinder3DReadOnly)
         return STPCylinder3DBasics.super.equals((STPCylinder3DReadOnly) object);
      else
         return false;
   }

   /**
    * Calculates and returns a hash code value from the value of each component of this cylinder 3D.
    *
    * @return the hash code value for this cylinder 3D.
    */
   @Override
   public int hashCode()
   {
      long hash = 1L;
      hash = EuclidHashCodeTools.toLongHashCode(length, radius);
      hash = EuclidHashCodeTools.combineHashCode(hash, EuclidHashCodeTools.toLongHashCode(position, axis));
      hash = EuclidHashCodeTools.combineHashCode(hash, EuclidHashCodeTools.toLongHashCode(minimumMargin, maximumMargin));
      return EuclidHashCodeTools.toIntHashCode(hash);
   }

   /**
    * Provides a {@code String} representation of this cylinder 3D as follows:
    *
    * <pre>
    * STP Cylinder 3D: [position: (-0.362, -0.617,  0.066 ), axis: ( 0.634, -0.551, -0.543 ), length:  0.170, radius:  0.906, small radius: 0.001, large radius: 1.000]
    * </pre>
    *
    * @return the {@code String} representing this cylinder 3D.
    */
   @Override
   public String toString()
   {
      String stpSuffix = String.format(", small radius: " + DEFAULT_FORMAT + ", large radius: " + DEFAULT_FORMAT + "]", getSmallRadius(), getLargeRadius());
      return "STP " + EuclidShapeIOTools.getCylinder3DString(this).replace("]", stpSuffix);
   }
}
