package us.ihmc.pathPlanning.visibilityGraphs.dataStructure;

import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.Transformable;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
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

   public double distanceSquared(Point3DReadOnly query)
   {
      return EuclidGeometryTools.distanceSquaredFromPoint3DToLineSegment3D(query, source, target);
   }

   public double percentageAlongConnection(Point3DReadOnly query)
   {
      return EuclidGeometryTools.percentageAlongLineSegment3D(query, source, target);
   }

   public ConnectionPoint3D orthogonalProjection(Point3DReadOnly pointToProject, int regionId)
   {
      Point3D projection = EuclidGeometryTools.orthogonalProjectionOnLineSegment3D(pointToProject, source, target);
      return new ConnectionPoint3D(projection, regionId);
   }

   public ConnectionPoint3D getPointGivenPercentage(double percentage, int regionId)
   {
      Point3D result = new Point3D();
      result.interpolate(source, target, percentage);
      return new ConnectionPoint3D(result, regionId);
   }

   public void flip()
   {
      ConnectionPoint3D temp = source;
      source = target;
      target = temp;
   }

   public double length()
   {
      return source.distance(target);
   }

   public double lengthSquared()
   {
      return source.distanceSquared(target);
   }

   @Override
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
      return source.epsilonEquals(other.source, epsilon) && target.epsilonEquals(other.target, epsilon)
            || source.epsilonEquals(other.target, epsilon) && target.epsilonEquals(other.source, epsilon);
   }

   @Override
   public int hashCode()
   {
      return source.hashCode() + target.hashCode();
   }

   @Override
   public boolean equals(Object obj)
   {
      try
      {
         return equals((Connection) obj);
      }
      catch (ClassCastException e)
      {
         return false;
      }
   }

   public boolean equals(Connection other)
   {
      if (other == null)
         return false;
      else
         return (source.equals(other.source) && target.equals(other.target)) || (source.equals(other.target) && target.equals(other.source));
   }

   @Override
   public String toString()
   {
      return "Connection: source = " + EuclidCoreIOTools.getTuple3DString(source) + ", target = " + EuclidCoreIOTools.getTuple3DString(target);
   }
}