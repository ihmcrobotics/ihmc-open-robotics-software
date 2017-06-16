package us.ihmc.simulationConstructionSetTools.util.perturbance;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;


public class CollisionPerturbance implements DirectedPerturbance
{
   private final String name = "CollisionPerturbance";
   private final YoVariableRegistry registry = new YoVariableRegistry(name);
   private final Collidable collidable;
   private final double ballVelocityMagnitude;
   private final YoDouble disturbanceEnergy = new YoDouble("disturbanceEnergy", registry);
   private final YoDouble coefficientOfRestitution = new YoDouble("coefficientOfRestitution", registry);

   public CollisionPerturbance(Collidable collidable, double ballVelocity, double disturbanceEnergy, double coefficientOfRestitution, YoVariableRegistry parentRegistry)
   {
      this.collidable = collidable;
      this.ballVelocityMagnitude = ballVelocity;
      this.disturbanceEnergy.set(disturbanceEnergy);
      this.coefficientOfRestitution.set(coefficientOfRestitution);
      
      if (parentRegistry != null)
         parentRegistry.addChild(registry);
   }

   @Override
   public void perturb(Vector3D direction)
   {
      Vector3D ballVelocity = new Vector3D(direction);
      if (ballVelocity.lengthSquared() > 0.0)
      {
         ballVelocity.normalize();
         ballVelocity.scale(ballVelocityMagnitude);
         collidable.handleCollision(ballVelocity, getBallMass(), coefficientOfRestitution);
      }
   }

   @Override
   public double getBallMass()
   {
      return 2.0 * disturbanceEnergy.getDoubleValue() / (ballVelocityMagnitude * ballVelocityMagnitude);
   }
   
   @Override
   public double getBallVelocityMagnitude()
   {
      return ballVelocityMagnitude;
   }

   @Override
   public void doEveryTick()
   {
      // empty
   }
}
