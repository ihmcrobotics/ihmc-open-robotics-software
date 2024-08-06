package us.ihmc.robotics.math.trajectories;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.robotics.math.interpolators.OrientationInterpolationCalculator;
import us.ihmc.robotics.math.trajectories.interfaces.FixedFrameOrientationTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.yoVariables.YoPolynomial;
import us.ihmc.robotics.trajectories.providers.FrameOrientationProvider;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameQuaternion;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

@Deprecated
public class OrientationInterpolationTrajectoryGenerator implements FixedFrameOrientationTrajectoryGenerator
{
   private final YoRegistry registry;
   private final YoDouble currentTime;
   private final YoDouble trajectoryTime;
   private final ReferenceFrame referenceFrame;
   private final YoPolynomial parameterPolynomial;
   private final YoFrameQuaternion initialOrientation;
   private final YoFrameQuaternion finalOrientation;
   
   private final YoFrameQuaternion desiredOrientation;
   private final YoFrameVector3D desiredAngularVelocity;
   private final YoFrameVector3D desiredAngularAcceleration;

   private final DoubleProvider trajectoryTimeProvider;
   private final FrameOrientationProvider initialOrientationProvider;
   private final FrameOrientationProvider finalOrientationProvider;

   private final YoBoolean continuouslyUpdateFinalOrientation;
   
   private final FrameQuaternion tempInitialOrientation;
   private final FrameQuaternion tempFinalOrientation;
   
   private final OrientationInterpolationCalculator orientationInterpolationCalculator = new OrientationInterpolationCalculator();

   public OrientationInterpolationTrajectoryGenerator(String namePrefix, ReferenceFrame referenceFrame, DoubleProvider trajectoryTimeProvider,
                                                      FrameOrientationProvider initialOrientationProvider, FrameOrientationProvider finalOrientationProvider,
                                                      YoRegistry parentRegistry)
   {
      this.registry = new YoRegistry(namePrefix + getClass().getSimpleName());
      this.trajectoryTime = new YoDouble(namePrefix + "TrajectoryTime", registry);
      this.currentTime = new YoDouble(namePrefix + "Time", registry);
      this.parameterPolynomial = new YoPolynomial(namePrefix + "ParameterPolynomial", 6, registry);
      this.referenceFrame = referenceFrame;
      
      this.initialOrientation = new YoFrameQuaternion(namePrefix + "InitialOrientation", referenceFrame, registry);
      this.finalOrientation = new YoFrameQuaternion(namePrefix + "FinalOrientation", referenceFrame, registry);
      this.continuouslyUpdateFinalOrientation = new YoBoolean(namePrefix + "ContinuouslyUpdate", registry);
      
      this.desiredOrientation = new YoFrameQuaternion(namePrefix + "desiredOrientation", referenceFrame, registry);
      this.desiredAngularVelocity = new YoFrameVector3D(namePrefix + "desiredAngularVelocity", referenceFrame, registry);
      this.desiredAngularAcceleration = new YoFrameVector3D(namePrefix + "desiredAngularAcceleration", referenceFrame, registry);

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
      initialOrientation.setMatchingFrame(initialOrientationProvider.getOrientation());
      initialOrientation.checkIfUnitary();
   }

   private void updateFinalOrientation()
   {
      finalOrientation.setMatchingFrame(finalOrientationProvider.getOrientation());
      finalOrientation.checkIfUnitary();
   }

   public void compute(double time)
   {
      if (continuouslyUpdateFinalOrientation.getBooleanValue())
         updateFinalOrientation();

      this.currentTime.set(time);
      time = MathTools.clamp(time, 0.0, trajectoryTime.getDoubleValue());
      parameterPolynomial.compute(time);
      
      double parameter = isDone() ? 1.0 : parameterPolynomial.getValue();
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

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

   @Override
   public FrameQuaternionReadOnly getOrientation()
   {
      return desiredOrientation;
   }

   @Override
   public FrameVector3DReadOnly getAngularVelocity()
   {
      return desiredAngularVelocity;
   }

   @Override
   public FrameVector3DReadOnly getAngularAcceleration()
   {
      return desiredAngularAcceleration;
   }
}
