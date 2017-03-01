package us.ihmc.robotics.geometry;

import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * Represents an infinitely long 3D line defined by a 3D point and a 3D vector.
 * 
 * @author Sylvain Bertrand
 *
 */
public class Line3d implements GeometryObject<Line3d>
{
   /** Coordinates of a point located on this line. */
   private final Point3D point = new Point3D();
   /** Normalized direction of this line. */
   private final Vector3D normalizedVector = new Vector3D();

   /**
    * Default constructor that initializes both {@link #point} and {@link #normalizedVector} to zero.
    * This point and vector have to be set to valid values to make this line usable.
    */
   public Line3d()
   {
   }

   /**
    * Initializes this line to be passing through the given point, with the vector as the direction.
    * 
    * @param point point on this line. Not modified.
    * @param vector direction of this line. Not modified.
    */
   public Line3d(Point3DReadOnly point, Vector3DReadOnly vector)
   {
      set(point, vector);
   }

   /**
    * Initializes this line to be passing through the two given points.
    * 
    * @param firstPointOnLine first point on this line. Not modified.
    * @param secondPointOnLine second point on this line. Not modified.
    */
   public Line3d(Point3DReadOnly firstPointOnLine, Point3DReadOnly secondPointOnLine)
   {
      set(firstPointOnLine, secondPointOnLine);
   }

   /**
    * @return the reference to the point through which this line is going.
    */
   public Point3D getPoint()
   {
      return point;
   }

   /**
    * @return the reference to the direction of this line.
    */
   public Vector3D getNormalizedVector()
   {
      return normalizedVector;
   }

   /**
    * Changes the point through which this line has to go.
    * 
    * @param point new point on this line. Not modified.
    */
   public void setPoint(Point3DReadOnly point)
   {
      this.point.set(point);
   }

   /**
    * Changes the direction of this line by setting to the normalized value of the given vector.
    * 
    * @param vector new direction of this line. Not modified.
    */
   public void setVector(Vector3DReadOnly vector)
   {
      this.normalizedVector.set(vector);
      normalize();
   }

   /**
    * Redefines this line with a new point and a new direction vector.
    * 
    * @param point new point on this line. Not modified.
    * @param vector new direction of this line. Not modified.
    */
   public void set(Point3DReadOnly point, Vector3DReadOnly vector)
   {
      setPoint(point);
      setVector(vector);
   }

   /**
    * Redefines this line such that it goes through the two given points.
    * 
    * @param firstPointOnLine first point on this line. Not modified.
    * @param secondPointOnLine second point on this line. Not modified.
    */
   public void set(Point3DReadOnly firstPointOnLine, Point3DReadOnly secondPointOnLine)
   {
      setPoint(firstPointOnLine);
      normalizedVector.sub(secondPointOnLine, firstPointOnLine);
      normalizedVector.normalize();
   }

   /**
    * Sets this line to be the same as the given line.
    * 
    * @param other the other line to copy. Not modified.
    */
   @Override
   public void set(Line3d other)
   {
      point.set(other.point);
      normalizedVector.set(other.normalizedVector);
   }

   /**
    * Sets the point and vector of this line to zero.
    * After calling this method, this line becomes invalid.
    * A new valid point and valid vector will have to be set so this line is again usable.
    */
   @Override
   public void setToZero()
   {
      point.setToZero();
      normalizedVector.setToZero();
   }

   /**
    * Sets the point and vector of this line to {@link Double#NaN}.
    * After calling this method, this line becomes invalid.
    * A new valid point and valid vector will have to be set so this line is again usable.
    */
   @Override
   public void setToNaN()
   {
      point.setToNaN();
      normalizedVector.setToNaN();
   }

   /**
    * Tests if this line contains {@link Double#NaN}.
    * 
    * @return {@code true} if {@link #point} and/or {@link #normalizedVector} contains {@link Double#NaN}, {@code false} otherwise.
    */
   @Override
   public boolean containsNaN()
   {
      if (point.containsNaN())
         return true;
      if (normalizedVector.containsNaN())
         return true;

      return false;
   }

   /**
    * Computes the minimum distance the given 3D point and this line.
    * <p>
    * Edge cases:
    * <ul>
    *    <li> if {@code normalizedVector.length() < Epsilons.ONE_TRILLIONTH}, this method returns the distance between {@code point} and the given {@code point}.
    * </ul>
    * </p>
    *
    * @param point 3D point to compute the distance from the line. Not modified.
    * @return the minimum distance between the 3D point and this 3D line.
    */
   public double distance(Point3DReadOnly point)
   {
      return GeometryTools.distanceFromPointToLine(point, this.point, this.normalizedVector);
   }

   /**
    * This methods computes the minimum distance between this line and {@code otherLine}.
    * <a href="http://geomalgorithms.com/a07-_distance.html"> Useful link</a>.
    * 
    * @param otherLine the other line to compute the distance from. Not modified.
    * @return the minimum distance between the two lines.
    */
   public double distance(Line3d otherLine)
   {
      return GeometryTools.distanceBetweenTwoLines(point, normalizedVector, otherLine.point, otherLine.normalizedVector);
   }

   /**
    * This methods computes two points P &in; this line and Q &in; {@code otherLine} such that the distance || P - Q || is the minimum distance between the two 3D lines.
    * <a href="http://geomalgorithms.com/a07-_distance.html"> Useful link</a>.
    * 
    * @param otherLine the second line. Not modified.
    * @param closestPointOnThisLineToPack the 3D coordinates of the point P are packed in this 3D point. Modified. Can be {@code null}.
    * @param closestPointOnOtherLineToPack the 3D coordinates of the point Q are packed in this 3D point. Modified. Can be {@code null}.
    * @return the minimum distance between the two lines.
    */
   public double getClosestPointsWith(Line3d otherLine, Point3DBasics closestPointOnThisLineToPack, Point3DBasics closestPointOnOtherLineToPack)
   {
      return GeometryTools.getClosestPointsForTwoLines(point, normalizedVector, otherLine.point, otherLine.normalizedVector, closestPointOnThisLineToPack, closestPointOnOtherLineToPack);
   }

   /**
    * Transforms this line using the given homogeneous transformation matrix.
    * 
    * @param transform the transform to apply on this line's point and vector. Not modified.
    */
   @Override
   public void applyTransform(Transform transform)
   {
      point.applyTransform(transform);
      normalizedVector.applyTransform(transform);
   }

   /**
    * Tests on a per-component basis on the point and vector if this line is equal to {@code other} with the tolerance {@code epsilon}.
    * This method will return {@code false} if the two lines are physically the same but either the point or vector of each line is different.
    * For instance, if {@code this.point == other.point} and {@code this.normalizedVector == - other.normalizedVector}, the two lines
    * are physically the same but this method returns {@code false}.
    * 
    * @param other the query. Not modified.
    * @param epsilon the tolerance to use.
    * @return {@code true} if the two lines are equal, {@code false} otherwise.
    */
   @Override
   public boolean epsilonEquals(Line3d other, double epsilon)
   {
      if (!point.epsilonEquals(other.point, epsilon))
         return false;
      if (!normalizedVector.epsilonEquals(other.normalizedVector, epsilon))
         return false;

      return true;
   }

   /**
    * Normalize the vector of this line.
    * If its magnitude is too small, it is set to {@link Double#NaN}.
    */
   private void normalize()
   {
      if (GeometryTools.isZero(normalizedVector, 1e-12))
      {
         normalizedVector.setToNaN();
      }

      normalizedVector.normalize();
   }
}
