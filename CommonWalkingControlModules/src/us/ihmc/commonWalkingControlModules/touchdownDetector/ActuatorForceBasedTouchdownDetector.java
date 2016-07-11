package us.ihmc.commonWalkingControlModules.touchdownDetector;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.robotics.sensors.ForceSensorDataReadOnly;

import javax.vecmath.Vector3d;

public class ActuatorForceBasedTouchdownDetector implements TouchdownDetector
{
   private final BooleanYoVariable touchdownDetected;

   private final ForceSensorDataReadOnly foreSensorData;
   private final DoubleYoVariable touchdownForceThreshold;

   private final Wrench wrenchToPack = new Wrench();
   private final Vector3d vectorToPack = new Vector3d();

   public ActuatorForceBasedTouchdownDetector(String name, ForceSensorDataReadOnly forceSensorData, double touchdownForceThreshold, YoVariableRegistry registry)
   {
      this.foreSensorData = forceSensorData;
      this.touchdownForceThreshold = new DoubleYoVariable(name + "_touchdownForceThreshold", registry);
      this.touchdownForceThreshold.set(touchdownForceThreshold);

      touchdownDetected = new BooleanYoVariable(name + "_touchdownDetected", registry);
   }

   @Override
   public boolean hasTouchedDown()
   {
      foreSensorData.getWrench(wrenchToPack);
      wrenchToPack.getLinearPart(vectorToPack);

      touchdownDetected.set(vectorToPack.length() > touchdownForceThreshold.getDoubleValue());

      return touchdownDetected.getBooleanValue();
   }
}
