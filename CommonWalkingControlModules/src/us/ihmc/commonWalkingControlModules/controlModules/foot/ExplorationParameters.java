package us.ihmc.commonWalkingControlModules.controlModules.foot;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

/**
 * This class provides parameters for foothold exploration that are common for both feet. In this
 * way they are not created twice for left and right and tuning is easier.
 *
 * @author Georg
 *
 */
public class ExplorationParameters
{
   private final String name = this.getClass().getSimpleName();
   private final YoVariableRegistry registry;

   private final DoubleYoVariable geometricDetectionAngleThreshold;

   private final static double defaultGeometricDetectionAngleThreshold = 10.0 * Math.PI/180.0;

   public ExplorationParameters(YoVariableRegistry parentRegistry)
   {
      registry = new YoVariableRegistry(name);
      parentRegistry.addChild(registry);

      String namePrefix = "Exploration";
      geometricDetectionAngleThreshold = new DoubleYoVariable(namePrefix + "GeometricAngleThreshold", registry);
      geometricDetectionAngleThreshold.set(defaultGeometricDetectionAngleThreshold);
   }

   public DoubleYoVariable getGeometricDetectionAngleThreshold()
   {
      return geometricDetectionAngleThreshold;
   }
}
