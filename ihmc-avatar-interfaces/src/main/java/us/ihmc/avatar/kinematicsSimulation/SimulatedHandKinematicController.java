package us.ihmc.avatar.kinematicsSimulation;

import us.ihmc.yoVariables.registry.YoRegistry;

public interface SimulatedHandKinematicController
{
   public void initialize();

   public void doControl();

   public void cleanup();

   public YoRegistry getRegistry();
}
