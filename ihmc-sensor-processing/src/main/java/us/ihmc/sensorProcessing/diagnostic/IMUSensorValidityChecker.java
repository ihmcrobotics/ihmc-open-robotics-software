package us.ihmc.sensorProcessing.diagnostic;

import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameQuaternion;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;

public class IMUSensorValidityChecker implements DiagnosticUpdatable
{
   private final YoRegistry registry;
   private final YoFrameQuaternionValidityChecker orientationChecker;
   private final YoFrameTupleValidityChecker angularVelocityChecker;
   private final YoFrameTupleValidityChecker linearAccelerationChecker;

   public IMUSensorValidityChecker(IMUDefinition imuDefinition, YoFrameQuaternion orientation, YoFrameVector3D angularVelocity, YoFrameVector3D linearAcceleration, YoRegistry parentRegistry)
   {
      String imuName = imuDefinition.getName();
      registry = new YoRegistry(imuName + "IMUSensorValidityChecker");
      parentRegistry.addChild(registry);

      verifyYovariableNames(imuName, orientation, angularVelocity, linearAcceleration);

      orientationChecker = new YoFrameQuaternionValidityChecker(orientation, registry);
      angularVelocityChecker = new YoFrameTupleValidityChecker(angularVelocity, registry);
      linearAccelerationChecker = new YoFrameTupleValidityChecker(linearAcceleration, registry);
   }

   @Override
   public void update()
   {
      orientationChecker.update();
      angularVelocityChecker.update();
      linearAccelerationChecker.update();
   }

   public void setupForLogging()
   {
      String loggerName = registry.getName();
      orientationChecker.setupForLogging(loggerName);
      angularVelocityChecker.setupForLogging(loggerName);
      linearAccelerationChecker.setupForLogging(loggerName);
   }

   @Override
   public void enable()
   {
      orientationChecker.enable();
      angularVelocityChecker.enable();
      linearAccelerationChecker.enable();
   }

   @Override
   public void disable()
   {
      orientationChecker.disable();
      angularVelocityChecker.disable();
      linearAccelerationChecker.disable();
   }

   public boolean areSensorValuesSane()
   {
      return orientationChecker.isInputSane() && angularVelocityChecker.isInputSane() && linearAccelerationChecker.isInputSane();
   }

   public boolean areAllSensorsAlive()
   {
      return orientationChecker.isInputAlive() && angularVelocityChecker.isInputAlive() && linearAccelerationChecker.isInputAlive();
   }

   public boolean sensorsCannotBeTrusted()
   {
      return orientationChecker.variableCannotBeTrusted() || angularVelocityChecker.variableCannotBeTrusted() || linearAccelerationChecker.variableCannotBeTrusted();
   }

   public void verifyYovariableNames(String imuName, YoFrameQuaternion orientation, YoFrameVector3D angularVelocity, YoFrameVector3D linearAcceleration)
   {
      if (!orientation.getYoQx().getName().contains(imuName))
         throw new RuntimeException("The orientation variable: " + orientation.getYoQx().getName() + " may not belong to the IMU: " + imuName);
      if (!orientation.getYoQy().getName().contains(imuName))
         throw new RuntimeException("The orientation variable: " + orientation.getYoQy().getName() + " may not belong to the IMU: " + imuName);
      if (!orientation.getYoQz().getName().contains(imuName))
         throw new RuntimeException("The orientation variable: " + orientation.getYoQz().getName() + " may not belong to the IMU: " + imuName);
      if (!orientation.getYoQs().getName().contains(imuName))
         throw new RuntimeException("The orientation variable: " + orientation.getYoQs().getName() + " may not belong to the IMU: " + imuName);

      if (!angularVelocity.getYoX().getName().contains(imuName))
         throw new RuntimeException("The angular velocity variable: " + angularVelocity.getYoX().getName() + " may not belong to the IMU: " + imuName);
      if (!angularVelocity.getYoY().getName().contains(imuName))
         throw new RuntimeException("The angular velocity variable: " + angularVelocity.getYoY().getName() + " may not belong to the IMU: " + imuName);
      if (!angularVelocity.getYoZ().getName().contains(imuName))
         throw new RuntimeException("The angular velocity variable: " + angularVelocity.getYoZ().getName() + " may not belong to the IMU: " + imuName);

      if (!linearAcceleration.getYoX().getName().contains(imuName))
         throw new RuntimeException("The linear acceleration variable: " + linearAcceleration.getYoX().getName() + " may not belong to the IMU: " + imuName);
      if (!linearAcceleration.getYoY().getName().contains(imuName))
         throw new RuntimeException("The linear acceleration variable: " + linearAcceleration.getYoY().getName() + " may not belong to the IMU: " + imuName);
      if (!linearAcceleration.getYoZ().getName().contains(imuName))
         throw new RuntimeException("The linear acceleration variable: " + linearAcceleration.getYoZ().getName() + " may not belong to the IMU: " + imuName);
   }
}
