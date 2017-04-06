package us.ihmc.simulationconstructionset.physics.featherstone;

import us.ihmc.simulationconstructionset.Robot;

public abstract class RobotWithExplicitDynamics extends Robot
{
   public RobotWithExplicitDynamics(String name)
   {
      super(name);
   }

   /**
    * Checks joint accelerations computed from physics engine are close to those computed from the Lagrangian
    * @param epsilon error bound on joint accelerations
    */
   public abstract void assertStateIsCloseToLagrangianCalculation(double epsilon);
}
