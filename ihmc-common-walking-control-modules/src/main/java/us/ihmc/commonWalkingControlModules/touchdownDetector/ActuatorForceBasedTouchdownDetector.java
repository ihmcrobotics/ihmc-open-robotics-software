package us.ihmc.commonWalkingControlModules.touchdownDetector;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.robotics.math.filters.GlitchFilteredYoBoolean;
import us.ihmc.robotics.sensors.ForceSensorDataReadOnly;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class ActuatorForceBasedTouchdownDetector implements TouchdownDetector
{
   private final int GLITCH_FLITER_WINDOW_SIZE = 10;

   private final YoBoolean touchdownDetected;
   private final YoBoolean touchdownForSureDetected;
   private final GlitchFilteredYoBoolean touchdownDetectedFiltered;
   private final GlitchFilteredYoBoolean touchdownForSureDetectedFiltered;

   private final ForceSensorDataReadOnly foreSensorData;
   private final YoDouble touchdownForceThreshold;
   private final YoDouble definitelyTouchdownForceThreshold;

   private final Wrench wrenchToPack = new Wrench();
   private final Vector3D vectorToPack = new Vector3D();

   public ActuatorForceBasedTouchdownDetector(String name, ForceSensorDataReadOnly forceSensorData, double touchdownForceThreshold, double defintielyTouchdownForceThreshold,
                                              YoVariableRegistry registry)
   {
      this.foreSensorData = forceSensorData;
      this.touchdownForceThreshold = new YoDouble(name + "_touchdownForceThreshold", registry);
      this.definitelyTouchdownForceThreshold = new YoDouble(name + "_definitelyTouchdownForceThreshold", registry);
      this.touchdownForceThreshold.set(touchdownForceThreshold);
      this.definitelyTouchdownForceThreshold.set(defintielyTouchdownForceThreshold);

      touchdownDetected = new YoBoolean(name + "_touchdownDetected", registry);
      touchdownForSureDetected = new YoBoolean(name + "_touchdownForSureDetected", registry);
      touchdownDetectedFiltered = new GlitchFilteredYoBoolean(touchdownDetected.getName() + "Filtered", registry, touchdownDetected, GLITCH_FLITER_WINDOW_SIZE);
      touchdownForSureDetectedFiltered = new GlitchFilteredYoBoolean(touchdownForSureDetected.getName() + "Filtered", registry, touchdownForSureDetected, GLITCH_FLITER_WINDOW_SIZE);
   }

   @Override
   public boolean hasTouchedDown()
   {
      return touchdownDetectedFiltered.getBooleanValue();
   }

   @Override
   public boolean hasForSureTouchedDown()
   {
      return touchdownForSureDetectedFiltered.getBooleanValue();
   }

   @Override
   public void update()
   {
      foreSensorData.getWrench(wrenchToPack);
      vectorToPack.set(wrenchToPack.getLinearPart());

      touchdownDetected.set(vectorToPack.length() > touchdownForceThreshold.getDoubleValue());
      touchdownForSureDetected.set(vectorToPack.length() > definitelyTouchdownForceThreshold.getDoubleValue());
      touchdownDetectedFiltered.update();
      touchdownForSureDetectedFiltered.update();
   }

   @Override
   public void reset()
   {
      touchdownDetected.set(false);
      touchdownForSureDetected.set(false);
      touchdownDetectedFiltered.set(false);
      touchdownForSureDetectedFiltered.set(false);
   }

   @Override
   public String getName()
   {
      return getClass().getSimpleName();
   }
}
