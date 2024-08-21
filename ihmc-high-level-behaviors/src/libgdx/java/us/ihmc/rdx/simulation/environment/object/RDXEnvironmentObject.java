package us.ihmc.rdx.simulation.environment.object;

import us.ihmc.euclid.tuple3D.Point3D;

/**
 * TODO: This class probably needs renaming and setup as layers of interfaces.
 */
public class RDXEnvironmentObject extends RDXSimpleObject
{
   private float mass = 0.0f;
   private final Point3D centerOfMassInModelFrame = new Point3D();
   public RDXEnvironmentObject(String titleCasedName, RDXEnvironmentObjectFactory factory)
   {
      super(titleCasedName, factory);
   }

   public RDXEnvironmentObject duplicate()
   {
      return factory.getSupplier().get();
   }

   public void setMass(float mass)
   {
      this.mass = mass;
   }

   public float getMass()
   {
      return mass;
   }

   public Point3D getCenterOfMassInModelFrame()
   {
      return centerOfMassInModelFrame;
   }
}
