package us.ihmc.simulationconstructionset.physics.collision.simple;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;

public class SingleContact
{
   private final Point3D worldA = new Point3D();
   private final Point3D worldB = new Point3D();
   private final Vector3D normalA = new Vector3D();
   private double distance;
   
   public void set(Point3D worldA, Point3D worldB, Vector3D normalA, double distance)
   {
      this.worldA.set(worldA);
      this.worldB.set(worldB);
      this.normalA.set(normalA);
      this.distance = distance;
   }
   
   public void getWorldA(Point3D worldAToPack)
   {
      worldAToPack.set(worldA);
   }
   
   public void getWorldB(Point3D worldBToPack)
   {
      worldBToPack.set(worldB);
   }
   
   public void getNormalA(Vector3D normalAToPack)
   {
      normalAToPack.set(normalA);
   }
   
   public double getDistance()
   {
      return distance;
   }
}
