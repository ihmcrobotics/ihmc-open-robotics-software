package us.ihmc.robotics.math.trajectories;

import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.commons.MathTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.math.frames.YoFrameQuaternion;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.interpolators.OrientationInterpolationCalculator;
import us.ihmc.robotics.trajectories.providers.DoubleProvider;
import us.ihmc.robotics.trajectories.providers.OrientationProvider;

public class OrientationInterpolationTrajectoryGenerator implements OrientationTrajectoryGenerator
{
   private final YoVariableRegistry registry;
   private final YoDouble currentTime;
   private final YoDouble trajectoryTime;
   private final YoPolynomial parameterPolynomial;
   private final YoFrameQuaternion initialOrientation;
   private final YoFrameQuaternion finalOrientation;
   
   private final YoFrameQuaternion desiredOrientation;
   private final YoFrameVector desiredAngularVelocity;
   private final YoFrameVector desiredAngularAcceleration;

   private final DoubleProvider trajectoryTimeProvider;
   private final OrientationProvider initialOrientationProvider;
   private final OrientationProvider finalOrientationProvider;

   private final YoBoolean continuouslyUpdateFinalOrientation;
   
   private final FrameQuaternion tempInitialOrientation;
   private final FrameQuaternion tempFinalOrientation;
   
   private final OrientationInterpolationCalculator orientationInterpolationCalculator = new OrientationInterpolationCalculator();

   public OrientationInterpolationTrajectoryGenerator(String namePrefix, ReferenceFrame referenceFrame, DoubleProvider trajectoryTimeProvider,
                                                      OrientationProvider initialOrientationProvider, OrientationProvider finalOrientationProvider,
                                                      YoVariableRegistry parentRegistry)
   {
      this.registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());
      this.trajectoryTime = new YoDouble(namePrefix + "TrajectoryTime", registry);
      this.currentTime = new YoDouble(namePrefix + "Time", registry);
      this.parameterPolynomial = new YoPolynomial(namePrefix + "ParameterPolynomial", 6, registry);
      
      this.initialOrientation = new YoFrameQuaternion(namePrefix + "InitialOrientation", referenceFrame, registry);
      this.finalOrientation = new YoFrameQuaternion(namePrefix + "FinalOrientation", referenceFrame, registry);
      this.continuouslyUpdateFinalOrientation = new YoBoolean(namePrefix + "ContinuouslyUpdate", registry);
      
      this.desiredOrientation = new YoFrameQuaternion(namePrefix + "desiredOrientation", referenceFrame, registry);
      this.desiredAngularVelocity = new YoFrameVector(namePrefix + "desiredAngularVelocity", referenceFrame, registry);
      this.desiredAngularAcceleration = new YoFrameVector(namePrefix + "desiredAngularAcceleration", referenceFrame, registry);

      this.trajectoryTimeProvider = trajectoryTimeProvider;
      this.initialOrientationProvider = initialOrientationProvider;
      this.finalOrientationProvider = finalOrientationProvider;

      tempInitialOrientation = new FrameQuaternion(referenceFrame);
      tempFinalOrientation = new FrameQuaternion(referenceFrame);

      parentRegistry.addChild(registry);
   }

   public void setContinuouslyUpdateFinalOrientation(boolean continuouslyUpdateFinalOrientation)
   {
      this.continuouslyUpdateFinalOrientation.set(continuouslyUpdateFinalOrientation);
   }

   public void initialize()
   {
      double trajectoryTime = trajectoryTimeProvider.getValue();
      MathTools.checkIntervalContains(trajectoryTime, 0.0, Double.POSITIVE_INFINITY);
      this.trajectoryTime.set(trajectoryTime);
      currentTime.set(0.0);
      parameterPolynomial.setQuintic(0.0, trajectoryTime, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
      
      updateInitialOrientation();
      updateFinalOrientation();

      desiredOrientation.set(initialOrientation);
      desiredAngularVelocity.setToZero();
      desiredAngularAcceleration.setToZero();
   }

   private void updateInitialOrientation()
   {
      initialOrientationProvider.getOrientation(tempInitialOrientation);      
      tempInitialOrientation.changeFrame(initialOrientation.getReferenceFrame());
      initialOrientation.set(tempInitialOrientation);
      initialOrientation.checkQuaternionIsUnitMagnitude();
   }

   private void updateFinalOrientation()
   {
      finalOrientationProvider.getOrientation(tempFinalOrientation);
      tempFinalOrientation.changeFrame(finalOrientation.getReferenceFrame());
      finalOrientation.set(tempFinalOrientation);
      finalOrientation.checkQuaternionIsUnitMagnitude();
   }

   public void compute(double time)
   {
      if (continuouslyUpdateFinalOrientation.getBooleanValue())
         updateFinalOrientation();

      this.currentTime.set(time);
      time = MathTools.clamp(time, 0.0, trajectoryTime.getDoubleValue());
      parameterPolynomial.compute(time);
      
      double parameter = isDone() ? 1.0 : parameterPolynomial.getPosition();
      desiredOrientation.interpolate(initialOrientation, finalOrientation, parameter);
      double parameterd = isDone() ? 0.0 : parameterPolynomial.getVelocity();
      orientationInterpolationCalculator.computeAngularVelocity(desiredAngularVelocity, initialOrientation, finalOrientation, parameterd);
      double parameterdd = isDone() ? 0.0 : parameterPolynomial.getAcceleration();
      orientationInterpolationCalculator.computeAngularAcceleration(desiredAngularAcceleration, initialOrientation, finalOrientation, parameterdd);
   }

   public boolean isDone()
   {
      return currentTime.getDoubleValue() >= trajectoryTime.getDoubleValue();
   }

   public void getOrientation(FrameQuaternion orientationToPack)
   {
      desiredOrientation.getFrameOrientationIncludingFrame(orientationToPack);
   }

   public void getAngularVelocity(FrameVector3D velocityToPack)
   {
      desiredAngularVelocity.getFrameTupleIncludingFrame(velocityToPack);
   }

   public void getAngularAcceleration(FrameVector3D accelerationToPack)
   {
      desiredAngularAcceleration.getFrameTupleIncludingFrame(accelerationToPack);
   }

   public void getAngularData(FrameQuaternion orientationToPack, FrameVector3D angularVelocityToPack, FrameVector3D angularAccelerationToPack)
   {
      getOrientation(orientationToPack);
      getAngularVelocity(angularVelocityToPack);
      getAngularAcceleration(angularAccelerationToPack);
   }
}
