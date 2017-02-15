package us.ihmc.simulationconstructionset.physics.collision.simple;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

public class SingleContact
{
   private final Point3d worldA = new Point3d();
   private final Point3d worldB = new Point3d();
   private final Vector3d normalA = new Vector3d();
   private double distance;
   
   public void set(Point3d worldA, Point3d worldB, Vector3d normalA, double distance)
   {
      this.worldA.set(worldA);
      this.worldB.set(worldB);
      this.normalA.set(normalA);
      this.distance = distance;
   }
   
   public void getWorldA(Point3d worldAToPack)
   {
      worldAToPack.set(worldA);
   }
   
   public void getWorldB(Point3d worldBToPack)
   {
      worldBToPack.set(worldB);
   }
   
   public void getNormalA(Vector3d normalAToPack)
   {
      normalAToPack.set(normalA);
   }
   
   public double getDistance()
   {
      return distance;
   }
}
