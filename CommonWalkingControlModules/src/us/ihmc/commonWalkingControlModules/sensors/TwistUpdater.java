package us.ihmc.commonWalkingControlModules.sensors;

import us.ihmc.simulationconstructionset.robotController.SensorProcessor;
import us.ihmc.robotics.screwTheory.TwistCalculator;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;

public class TwistUpdater implements SensorProcessor
{
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);
   private final TwistCalculator twistCalculator;

   public TwistUpdater(TwistCalculator twistCalculator)
   {
      this.twistCalculator = twistCalculator;
   }

   public void initialize()
   {
      update();
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   public String getName()
   {
      return name;
   }

   public String getDescription()
   {
      return getName();
   }

   public void update()
   {
      twistCalculator.compute();
   }
}
