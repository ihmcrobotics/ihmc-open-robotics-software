package us.ihmc.commonWalkingControlModules.sensors;

import us.ihmc.utilities.screwTheory.TwistCalculator;

import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.robotController.SensorProcessor;

public class TwistUpdater implements SensorProcessor
{
   private static final long serialVersionUID = -4141102690274454009L;
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
