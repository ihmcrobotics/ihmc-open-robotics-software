package us.ihmc.atlas.behaviors.scsSensorSimulation;

import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.registry.YoRegistry;

public class FunctionalRobotController implements RobotController
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private String name = getClass().getSimpleName();
   private String description = "Robot controller for " + getName() + ".";

   private Runnable initialize = () -> {};
   private Runnable doControl = () -> {};

   @Override
   public void initialize()
   {
      initialize.run();
   }

   @Override
   public void doControl()
   {
      doControl.run();
   }

   @Override
   public YoRegistry getYoRegistry()
   {
      return registry;
   }

   @Override
   public String getDescription()
   {
      return description;
   }

   @Override
   public String getName()
   {
      return name;
   }

   public void setInitialize(Runnable initialize)
   {
      this.initialize = initialize;
   }

   public void setDoControl(Runnable doControl)
   {
      this.doControl = doControl;
   }
}
