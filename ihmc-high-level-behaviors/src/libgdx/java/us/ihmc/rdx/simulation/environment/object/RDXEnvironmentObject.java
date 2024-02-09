package us.ihmc.rdx.simulation.environment.object;

public class RDXEnvironmentObject extends RDXSimpleObject
{
   private float mass = 0.0f;

   public RDXEnvironmentObject(String titleCasedName, RDXEnvironmentObjectFactory factory)
   {
      super(titleCasedName, factory);
   }

   public void setMass(float mass)
   {
      this.mass = mass;
   }

   public float getMass()
   {
      return mass;
   }

   public void setSelected(boolean selected)
   {
      // (NK) Do Nothing
   }
}
