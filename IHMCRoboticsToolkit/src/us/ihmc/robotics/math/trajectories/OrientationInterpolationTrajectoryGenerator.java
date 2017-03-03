package us.ihmc.robotics.math.trajectories;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFrameQuaternion;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.interpolators.OrientationInterpolationCalculator;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.trajectories.providers.DoubleProvider;
import us.ihmc.robotics.trajectories.providers.OrientationProvider;

public class OrientationInterpolationTrajectoryGenerator implements OrientationTrajectoryGenerator
{
   private final YoVariableRegistry registry;
   private final DoubleYoVariable currentTime;
   private final DoubleYoVariable trajectoryTime;
   private final YoPolynomial parameterPolynomial;
   private final YoFrameQuaternion initialOrientation;
   private final YoFrameQuaternion finalOrientation;
   
   private final YoFrameQuaternion desiredOrientation;
   private final YoFrameVector desiredAngularVelocity;
   private final YoFrameVector desiredAngularAcceleration;

   private final DoubleProvider trajectoryTimeProvider;
   private final OrientationProvider initialOrientationProvider;
   private final OrientationProvider finalOrientationProvider;

   private final BooleanYoVariable continuouslyUpdateFinalOrientation;
   
   private final FrameOrientation tempInitialOrientation;
   private final FrameOrientation tempFinalOrientation;
   
   private final OrientationInterpolationCalculator orientationInterpolationCalculator = new OrientationInterpolationCalculator();

   public OrientationInterpolationTrajectoryGenerator(String namePrefix, ReferenceFrame referenceFrame, DoubleProvider trajectoryTimeProvider,
                                                      OrientationProvider initialOrientationProvider, OrientationProvider finalOrientationProvider,
                                                      YoVariableRegistry parentRegistry)
   {
      this.registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());
      this.trajectoryTime = new DoubleYoVariable(namePrefix + "TrajectoryTime", registry);
      this.currentTime = new DoubleYoVariable(namePrefix + "Time", registry);
      this.parameterPolynomial = new YoPolynomial(namePrefix + "ParameterPolynomial", 6, registry);
      
      this.initialOrientation = new YoFrameQuaternion(namePrefix + "InitialOrientation", referenceFrame, registry);
      this.finalOrientation = new YoFrameQuaternion(namePrefix + "FinalOrientation", referenceFrame, registry);
      this.continuouslyUpdateFinalOrientation = new BooleanYoVariable(namePrefix + "ContinuouslyUpdate", registry);
      
      this.desiredOrientation = new YoFrameQuaternion(namePrefix + "desiredOrientation", referenceFrame, registry);
      this.desiredAngularVelocity = new YoFrameVector(namePrefix + "desiredAngularVelocity", referenceFrame, registry);
      this.desiredAngularAcceleration = new YoFrameVector(namePrefix + "desiredAngularAcceleration", referenceFrame, registry);

      this.trajectoryTimeProvider = trajectoryTimeProvider;
      this.initialOrientationProvider = initialOrientationProvider;
      this.finalOrientationProvider = finalOrientationProvider;

      tempInitialOrientation = new FrameOrientation(referenceFrame);
      tempFinalOrientation = new FrameOrientation(referenceFrame);

      parentRegistry.addChild(registry);
   }

   public void setContinuouslyUpdateFinalOrientation(boolean continuouslyUpdateFinalOrientation)
   {
      this.continuouslyUpdateFinalOrientation.set(continuouslyUpdateFinalOrientation);
   }

   public void initialize()
   {
      double trajectoryTime = trajectoryTimeProvider.getValue();
      MathTools.checkIfInRange(trajectoryTime, 0.0, Double.POSITIVE_INFINITY);
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

   public void getOrientation(FrameOrientation orientationToPack)
   {
      desiredOrientation.getFrameOrientationIncludingFrame(orientationToPack);
   }

   public void getAngularVelocity(FrameVector velocityToPack)
   {
      desiredAngularVelocity.getFrameTupleIncludingFrame(velocityToPack);
   }

   public void getAngularAcceleration(FrameVector accelerationToPack)
   {
      desiredAngularAcceleration.getFrameTupleIncludingFrame(accelerationToPack);
   }

   public void getAngularData(FrameOrientation orientationToPack, FrameVector angularVelocityToPack, FrameVector angularAccelerationToPack)
   {
      getOrientation(orientationToPack);
      getAngularVelocity(angularVelocityToPack);
      getAngularAcceleration(angularAccelerationToPack);
   }
}
