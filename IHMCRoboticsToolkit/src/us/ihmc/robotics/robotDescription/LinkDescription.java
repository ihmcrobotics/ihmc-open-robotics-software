package us.ihmc.robotics.robotDescription;

import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;

public class LinkDescription
{
   private String name;

   private double mass;
   private final Vector3d centerOfMassLocation = new Vector3d();
   private final DenseMatrix64F momentOfInertia = new DenseMatrix64F();

   private LinkGraphicsDescription linkGraphics;

   public LinkDescription(String name)
   {
      this.name = name;
   }

   public LinkGraphicsDescription getLinkGraphics()
   {
      return linkGraphics;
   }

   public void setLinkGraphics(LinkGraphicsDescription linkGraphics)
   {
      this.linkGraphics = linkGraphics;
   }

   public double getMass()
   {
      return mass;
   }

   public void setMass(double mass)
   {
      if (mass < 0.0) throw new RuntimeException("mass < 0.0");
      this.mass = mass;
   }

   public void getCenterOfMassLocation(Vector3d centerOfMassLocationToPack)
   {
      centerOfMassLocationToPack.set(centerOfMassLocation);
   }

   public void setMomentOfInertia(DenseMatrix64F momentOfInertiaToPack)
   {
      momentOfInertiaToPack.set(momentOfInertia);
   }

   public void getMomentOfInertia(DenseMatrix64F momentOfInertiaToPack)
   {
      momentOfInertiaToPack.set(momentOfInertia);
   }

   public void setMomentOfInertia(double Ixx, double Iyy, double Izz)
   {
      this.momentOfInertia.zero();
      this.momentOfInertia.set(0, 0, Ixx);
      this.momentOfInertia.set(1, 1, Iyy);
      this.momentOfInertia.set(2, 2, Izz);

   }

}
