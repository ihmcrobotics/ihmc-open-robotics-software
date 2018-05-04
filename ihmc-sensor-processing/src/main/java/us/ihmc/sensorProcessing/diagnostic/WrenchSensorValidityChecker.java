package us.ihmc.sensorProcessing.diagnostic;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFrameVector3D;
import us.ihmc.robotics.sensors.ForceSensorDefinition;

public class WrenchSensorValidityChecker implements DiagnosticUpdatable
{
   private final YoVariableRegistry registry;

   private final YoFrameTupleValidityChecker forceChecker;
   private final YoFrameTupleValidityChecker torqueChecker;

   public WrenchSensorValidityChecker(ForceSensorDefinition forceSensorDefinition, YoFrameVector3D forceMeasurement, YoFrameVector3D torqueMeasurement, YoVariableRegistry parentRegistry)
   {
      String wrenchSensorName = forceSensorDefinition.getSensorName();
      registry = new YoVariableRegistry(wrenchSensorName + "WrenchSensorValidityChecker");
      parentRegistry.addChild(registry);

      verifyYoVariableNames(wrenchSensorName, forceMeasurement, torqueMeasurement);

      forceChecker = new YoFrameTupleValidityChecker(forceMeasurement, registry);
      torqueChecker = new YoFrameTupleValidityChecker(torqueMeasurement, registry);
   }

   public void setupForLogging()
   {
      String loggerName = registry.getName();
      forceChecker.setupForLogging(loggerName);
      torqueChecker.setupForLogging(loggerName);
   }

   @Override
   public void update()
   {
      forceChecker.update();
      torqueChecker.update();
   }

   @Override
   public void enable()
   {
      forceChecker.enable();
      torqueChecker.enable();
   }

   @Override
   public void disable()
   {
      forceChecker.disable();
      torqueChecker.disable();
   }

   public boolean areSensorValuesSane()
   {
      return forceChecker.isInputSane() && torqueChecker.isInputSane();
   }

   public boolean areAllSensorsAlive()
   {
      return forceChecker.isInputAlive() && torqueChecker.isInputAlive();
   }

   private void verifyYoVariableNames(String wrenchSensorName, YoFrameVector3D forceMeasurement, YoFrameVector3D torqueMeasurement)
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
