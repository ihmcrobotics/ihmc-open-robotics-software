package us.ihmc.sensorProcessing.diagnostic;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.sensors.ForceSensorDefinition;

public class WrenchSensorValidityChecker implements DiagnosticUpdatable
{
   private final YoVariableRegistry registry;

   private final YoFrameTupleValidityChecker forceChecker;
   private final YoFrameTupleValidityChecker torqueChecker;

   public WrenchSensorValidityChecker(ForceSensorDefinition forceSensorDefinition, YoFrameVector forceMeasurement, YoFrameVector torqueMeasurement, YoVariableRegistry parentRegistry)
   {
      String wrenchSensorName = forceSensorDefinition.getSensorName();
      registry = new YoVariableRegistry(wrenchSensorName + "WrenchSensorValidityChecker");
      parentRegistry.addChild(registry);

      verifyYoVariableNames(wrenchSensorName, forceMeasurement, torqueMeasurement);

      forceChecker = new YoFrameTupleValidityChecker(forceMeasurement, registry);
      torqueChecker = new YoFrameTupleValidityChecker(torqueMeasurement, registry);
   }

   @Override
   public void update()
   {
      forceChecker.update();
      torqueChecker.update();
   }

   public boolean areSensorValuesSane()
   {
      return forceChecker.isInputSane() && torqueChecker.isInputSane();
   }

   public boolean areAllSensorsAlive()
   {
      return forceChecker.isInputAlive() && torqueChecker.isInputAlive();
   }

   private void verifyYoVariableNames(String wrenchSensorName, YoFrameVector forceMeasurement, YoFrameVector torqueMeasurement)
   {
      if (!forceMeasurement.getYoX().getName().contains(wrenchSensorName))
         throw new RuntimeException("The force variable: " + forceMeasurement.getYoX().getName() + " may not belong to the wrench sensor: " + wrenchSensorName);
      if (!forceMeasurement.getYoY().getName().contains(wrenchSensorName))
         throw new RuntimeException("The force variable: " + forceMeasurement.getYoY().getName() + " may not belong to the wrench sensor: " + wrenchSensorName);
      if (!forceMeasurement.getYoZ().getName().contains(wrenchSensorName))
         throw new RuntimeException("The force variable: " + forceMeasurement.getYoZ().getName() + " may not belong to the wrench sensor: " + wrenchSensorName);

      if (!torqueMeasurement.getYoX().getName().contains(wrenchSensorName))
         throw new RuntimeException("The torque variable: " + torqueMeasurement.getYoX().getName() + " may not belong to the wrench sensor: " + wrenchSensorName);
      if (!torqueMeasurement.getYoY().getName().contains(wrenchSensorName))
         throw new RuntimeException("The torque variable: " + torqueMeasurement.getYoY().getName() + " may not belong to the wrench sensor: " + wrenchSensorName);
      if (!torqueMeasurement.getYoZ().getName().contains(wrenchSensorName))
         throw new RuntimeException("The torque variable: " + torqueMeasurement.getYoZ().getName() + " may not belong to the wrench sensor: " + wrenchSensorName);
   }
}
