package us.ihmc.pathPlanning.visibilityGraphs;

import us.ihmc.euclid.interfaces.Transformable;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

public class Connection implements Transformable
{
   private ConnectionPoint3D source;
   private ConnectionPoint3D target;

   public Connection(Point3DReadOnly source, Point3DReadOnly target)
   {
      this.source = new ConnectionPoint3D(source);
      this.target = new ConnectionPoint3D(target);
   }

   public ConnectionPoint3D getSourcePoint()
   {
      return source;
   }

   public ConnectionPoint3D getTargetPoint()
   {
      return target;
   }

   public void applyTransform(Transform transform)
   {
      source.applyTransform(transform);
      target.applyTransform(transform);
   }

   @Override
   public void applyInverseTransform(Transform transform)
   {
      source.applyInverseTransform(transform);
      target.applyInverseTransform(transform);
   }
}