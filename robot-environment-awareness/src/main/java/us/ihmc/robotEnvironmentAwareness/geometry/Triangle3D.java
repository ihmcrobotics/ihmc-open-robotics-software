package us.ihmc.robotEnvironmentAwareness.geometry;

import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;

public class Triangle3D implements GeometryObject<Triangle3D>
{
   private final Point3D a = new Point3D();
   private final Point3D b = new Point3D();
   private final Point3D c = new Point3D();

   public Triangle3D()
   {
   }

   public Triangle3D(Tuple3DReadOnly a, Tuple3DReadOnly b, Tuple3DReadOnly c)
   {
      set(a, b, c);
   }

   @Override
   public void set(Triangle3D other)
   {
      a.set(other.a);
      b.set(other.b);
      c.set(other.c);
   }

   @Override
   public void setToZero()
   {
      a.setToZero();
      b.setToZero();
      c.setToZero();
   }

   @Override
   public void setToNaN()
   {
      a.setToNaN();
      b.setToNaN();
      c.setToNaN();
   }

   @Override
   public boolean containsNaN()
   {
      return a.containsNaN() || b.containsNaN() || c.containsNaN();
   }

   public void set(Tuple3DReadOnly a, Tuple3DReadOnly b, Tuple3DReadOnly c)
   {
      this.a.set(a);
      this.b.set(b);
      this.c.set(c);
   }

   public Point3D getA()
   {
      return a;
   }

   public Point3D getB()
   {
      return b;
   }

   public Point3D getC()
   {
      return c;
   }

   @Override
   public void applyTransform(Transform transform)
   {
      a.applyTransform(transform);
      b.applyTransform(transform);
      c.applyTransform(transform);
   }

   @Override
   public void applyInverseTransform(Transform transform)
   {
      a.applyInverseTransform(transform);
      b.applyInverseTransform(transform);
      c.applyInverseTransform(transform);
   }

   @Override
   public boolean epsilonEquals(Triangle3D other, double epsilon)
   {
      return a.epsilonEquals(other.a, epsilon) && b.epsilonEquals(other.b, epsilon) && c.epsilonEquals(other.c, epsilon);
   }

   @Override
   public boolean geometricallyEquals(Triangle3D other, double epsilon)
   {
      if (a.geometricallyEquals(other.a, epsilon))
         return b.geometricallyEquals(other.b, epsilon) && c.geometricallyEquals(other.c, epsilon);
      else if (a.geometricallyEquals(other.b, epsilon))
         return b.geometricallyEquals(other.c, epsilon) && c.geometricallyEquals(other.a, epsilon);
      else if (a.geometricallyEquals(other.c, epsilon))
         return b.geometricallyEquals(other.a, epsilon) && c.geometricallyEquals(other.b, epsilon);
      else
         return false;
   }

   @Override
   public String toString()
   {
      return "Triangle 3D: " + a + " - " + b + " - " + c;
   }
}
