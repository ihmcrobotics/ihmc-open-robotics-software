package us.ihmc.robotics.physics;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemBasics;
import us.ihmc.mecano.tools.MultiBodySystemStateIntegrator;

public class MultiRobotFirstOrderIntegrator
{
   private final List<MultiBodySystemBasics> multiBobySystems = new ArrayList<>();
   private final MultiBodySystemStateIntegrator integrator = new MultiBodySystemStateIntegrator();

   public void addMultiBodySystem(MultiBodySystemBasics multiBodySystem)
   {
      multiBobySystems.add(multiBodySystem);
   }

   public void removeMultiBodySystem(MultiBodySystemBasics multiBodySystem)
   {
      multiBobySystems.remove(multiBodySystem);
   }

   public void integrate(double dt)
   {
      integrator.setIntegrationDT(dt);

      for (MultiBodySystemBasics system : multiBobySystems)
      {
         integrator.doubleIntegrateFromAccelerationSubtree(system.getRootBody());
      }
   }
}
