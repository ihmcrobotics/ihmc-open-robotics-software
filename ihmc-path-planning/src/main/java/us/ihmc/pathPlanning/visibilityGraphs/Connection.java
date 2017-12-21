package us.ihmc.pathPlanning.visibilityGraphs;

import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.Transformable;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

public class Connection implements Transformable, EpsilonComparable<Connection>
{
   private ConnectionPoint3D source;
   private ConnectionPoint3D target;

   public Connection(Connection other)
   {
      this.source = new ConnectionPoint3D(other.source);
      this.target = new ConnectionPoint3D(other.target);
   }

   public Connection(Point2DReadOnly source, int sourceRegionId, Point2DReadOnly target, int targetRegionId)
   {
      this.source = new ConnectionPoint3D(source, sourceRegionId);
      this.target = new ConnectionPoint3D(target, targetRegionId);
   }

   public Connection(ConnectionPoint3D source, ConnectionPoint3D target)
   {
      this.source = new ConnectionPoint3D(source);
      this.target = new ConnectionPoint3D(target);
   }

   public Connection(Point3DReadOnly source, int sourceRegionId, Point3DReadOnly target, int targetRegionId)
   {
      this.source = new ConnectionPoint3D(source, sourceRegionId);
      this.target = new ConnectionPoint3D(target, targetRegionId);
   }

   public ConnectionPoint3D getSourcePoint()
   {
      return source;
   }

   public ConnectionPoint3D getTargetPoint()
   {
      return target;
   }

   public Point2D getSourcePoint2D()
   {
      return new Point2D(source);
   }

   public Point2D getTargetPoint2D()
   {
      return new Point2D(target);
   }

   public void applyTransform(Transform transform)
   {
      source = source.applyTransform(transform);
      target = target.applyTransform(transform);
   }

   @Override
   public void applyInverseTransform(Transform transform)
   {
      source = source.applyInverseTransform(transform);
      target = target.applyInverseTransform(transform);
   }

   @Override
   public boolean epsilonEquals(Connection other, double epsilon)
   {
      return source.epsilonEquals(other.source, epsilon) && target.epsilonEquals(other.target, epsilon) || source.epsilonEquals(other.target, epsilon) && target.epsilonEquals(other.source, epsilon);
   }

   @Override
   public String toString()
   {
      return "Connection: source = " + EuclidCoreIOTools.getTuple3DString(source) + ", target = " + EuclidCoreIOTools.getTuple3DString(target);
   }
}