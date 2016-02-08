package us.ihmc.robotics.math.trajectories;

import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.RotationTools;
import us.ihmc.robotics.math.QuaternionCalculus;
import us.ihmc.robotics.math.frames.YoFrameQuaternion;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

/**
 * Currently under development. Trying to fix this trajectory properly but it is not usable yet.
 * Two useful references from the web:
 * <p> a) <a href="http://www.geometrictools.com/Documentation/Quaternions.pdf">(SLERP & SQUAD methods)</a> </p>
 * <p> b) <a href="http://www.cim.mcgill.ca/~mpersson/docs/quat_calc_notes.pdf">(Proper quaternion calculus)</a> </p>
 * 
 * I got inspired by the first reference with the SQUAD method, but watch out there the math is totally WRONG.
 * The second reference provides a proper approach to general quaternion calculus. I used it a lot to derive time derivatives with quaternion & angular velocity. Really useful ref.
 *
 * <p>
 * This class needs a total cleanup and probably to extract several methods to a "tool" class.
 * I'll do so as soon as the math is right and it works.
 * </p>
 *
 * @author Sylvain 
 */
public class VelocityConstrainedOrientationTrajectoryGenerator implements OrientationTrajectoryGenerator
{
   private static final double PI = 1.0 * Math.PI;
   private final YoVariableRegistry registry;
   private final DoubleYoVariable currentTime;
   private final DoubleYoVariable trajectoryTime;
   private final YoPolynomial parameterPolynomial;

   private final YoFrameQuaternion initialOrientation;
   private final YoFrameQuaternion initialOrientationDrifted;
   private final YoFrameVector initialAngularVelocity;
   private final YoFrameQuaternion finalOrientation;
   private final YoFrameQuaternion finalOrientationDrifted;
   private final YoFrameVector finalAngularVelocity;

   private final YoFrameQuaternion desiredOrientation;
   private final YoFrameVector desiredAngularVelocity;
   private final YoFrameVector desiredAngularAcceleration;

   private final YoPolynomial saturationPolynomial;
   private final DoubleYoVariable maxAngularVelocityMagnitudeAtLimits;
   private final DoubleYoVariable initialAngularVelocityMagnitude;
   private final DoubleYoVariable finalAngularVelocityMagnitude;
   private final BooleanYoVariable initialDriftSaturated;
   private final BooleanYoVariable finalDriftSaturated;
   private final DoubleYoVariable initialAlphaSaturation;
   private final DoubleYoVariable finalAlphaSaturation;

   private final ReferenceFrame trajectoryFrame;

   private final QuaternionCalculus quaternionCalculus = new QuaternionCalculus();

   /**
    * Does not need to match the dt at which the trajectory will be updated.
    */
   private final double dtForFiniteDifference = 1.0e-4;

   public VelocityConstrainedOrientationTrajectoryGenerator(String namePrefix, ReferenceFrame referenceFrame, YoVariableRegistry parentRegistry)
   {
      registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());
      trajectoryTime = new DoubleYoVariable(namePrefix + "TrajectoryTime", registry);
      currentTime = new DoubleYoVariable(namePrefix + "Time", registry);
      parameterPolynomial = new YoPolynomial(namePrefix + "ParameterPolynomial", 6, registry);
      trajectoryFrame = referenceFrame;

      initialOrientation = new YoFrameQuaternion(namePrefix + "InitialOrientation", trajectoryFrame, registry);
      initialOrientationDrifted = new YoFrameQuaternion(namePrefix + "InitialOrientationDrifted", trajectoryFrame, registry);
      initialAngularVelocity = new YoFrameVector(namePrefix + "InitialAngularVelocity", trajectoryFrame, registry);
      finalOrientation = new YoFrameQuaternion(namePrefix + "FinalOrientation", trajectoryFrame, registry);
      finalOrientationDrifted = new YoFrameQuaternion(namePrefix + "FinalOrientationDrifted", trajectoryFrame, registry);
      finalAngularVelocity = new YoFrameVector(namePrefix + "FinalAngularVelocity", trajectoryFrame, registry);

      desiredOrientation = new YoFrameQuaternion(namePrefix + "DesiredOrientation", trajectoryFrame, registry);
      desiredAngularVelocity = new YoFrameVector(namePrefix + "DesiredAngularVelocity", trajectoryFrame, registry);
      desiredAngularAcceleration = new YoFrameVector(namePrefix + "DesiredAngularAcceleration", trajectoryFrame, registry);

      saturationPolynomial = new YoPolynomial(namePrefix + "SaturationPolynomial", 6, registry);
      saturationPolynomial.setQuintic(0, PI, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
      maxAngularVelocityMagnitudeAtLimits = new DoubleYoVariable(namePrefix + "MaxAngularVelocityMagnitudeAtLimits", registry);
      initialAngularVelocityMagnitude = new DoubleYoVariable(namePrefix + "InitialAngularVelocityMagnitude", registry);
      finalAngularVelocityMagnitude = new DoubleYoVariable(namePrefix + "FinalAngularVelocityMagnitude", registry);

      initialDriftSaturated = new BooleanYoVariable(namePrefix + "InitialDriftSaturated", registry);
      finalDriftSaturated = new BooleanYoVariable(namePrefix + "FinalDriftSaturated", registry);
      initialAlphaSaturation = new DoubleYoVariable(namePrefix + "InitialAlphaSaturation", registry);
      finalAlphaSaturation = new DoubleYoVariable(namePrefix + "FinalAlphaSaturation", registry);

      parentRegistry.addChild(registry);
   }

   public void setTrajectoryTime(double duration)
   {
      MathTools.checkIfInRange(duration, 0.0, Double.POSITIVE_INFINITY);
      trajectoryTime.set(duration);
   }

   private final FrameOrientation tempOrientation = new FrameOrientation();

   public void setInitialOrientation(FrameOrientation initialOrientation)
   {
      tempOrientation.setIncludingFrame(initialOrientation);
      tempOrientation.changeFrame(trajectoryFrame);
      this.initialOrientation.set(tempOrientation);
   }

   public void setFinalOrientation(FrameOrientation finalOrientation)
   {
      tempOrientation.setIncludingFrame(finalOrientation);
      tempOrientation.changeFrame(trajectoryFrame);
      this.finalOrientation.set(tempOrientation);
   }

   public void setFinalOrientation(FramePose finalPose)
   {
      finalPose.getOrientationIncludingFrame(tempOrientation);
      tempOrientation.changeFrame(trajectoryFrame);
      this.finalOrientation.set(tempOrientation);
   }

   public void setInitialAngularVelocity(FrameVector initialAngularVelocity)
   {
      this.initialAngularVelocity.setAndMatchFrame(initialAngularVelocity);
   }

   public void setFinalAngularVelocity(FrameVector finalAngularVelocity)
   {
      this.finalAngularVelocity.setAndMatchFrame(finalAngularVelocity);
   }

   public void setInitialVelocityToZero()
   {
      initialAngularVelocity.setToZero();
   }

   public void setFinalVelocityToZero()
   {
      finalAngularVelocity.setToZero();
   }

   public void setInitialConditions(FrameOrientation initialOrientation, FrameVector initialAngularVelocity)
   {
      setInitialOrientation(initialOrientation);
      setInitialAngularVelocity(initialAngularVelocity);
   }

   public void setFinalConditions(FrameOrientation finalOrientation, FrameVector finalAngularVelocity)
   {
      setFinalOrientation(finalOrientation);
      setFinalAngularVelocity(finalAngularVelocity);
   }

   @Override
   public void initialize()
   {
      currentTime.set(0.0);

      initialAngularVelocityMagnitude.set(initialAngularVelocity.length());
      finalAngularVelocityMagnitude.set(finalAngularVelocity.length());

      maxAngularVelocityMagnitudeAtLimits.set(PI / trajectoryTime.getDoubleValue());
      initialDriftSaturated.set(initialAngularVelocityMagnitude.getDoubleValue() > maxAngularVelocityMagnitudeAtLimits.getDoubleValue());
      finalDriftSaturated.set(finalAngularVelocityMagnitude.getDoubleValue() > maxAngularVelocityMagnitudeAtLimits.getDoubleValue());

      parameterPolynomial.setQuintic(0, trajectoryTime.getDoubleValue(), 0.0, 0.0, 0.0, 1.0, 0.0, 0.0);

      if (initialOrientation.dot(finalOrientation) < 0.0)
         finalOrientation.negate();

      desiredOrientation.set(initialOrientation);
      desiredAngularVelocity.set(initialAngularVelocity);
      desiredAngularAcceleration.setToZero();
   }

   private final Quat4d initialQuaternionDrifted = new Quat4d();
   private final Quat4d finalQuaternionDrifted = new Quat4d();

   private final Vector3d tempAngularVelocity = new Vector3d();
   private final Vector3d tempAngularAcceleration = new Vector3d();

   private final Quat4d qInterpolatedPrevious = new Quat4d();
   private final Quat4d qInterpolated = new Quat4d();
   private final Quat4d qInterpolatedNext = new Quat4d();

   private final Quat4d qDot = new Quat4d();
   private final Quat4d qDDot = new Quat4d();

   @Override
   public void compute(double time)
   {
      this.currentTime.set(time);

      if (isDone())
      {
         desiredOrientation.set(finalOrientation);
         desiredAngularVelocity.set(finalAngularVelocity);
         desiredAngularAcceleration.setToZero();
         return;
      }
      else if (currentTime.getDoubleValue() < 0.0)
      {
         desiredOrientation.set(initialOrientation);
         desiredAngularVelocity.set(initialAngularVelocity);
         desiredAngularAcceleration.setToZero();
         return;
      }

      time = MathTools.clipToMinMax(time, 0.0, trajectoryTime.getDoubleValue());

      interpolateOrientation(time, initialQuaternionDrifted, finalQuaternionDrifted, qInterpolated);

      initialOrientationDrifted.set(initialQuaternionDrifted);
      finalOrientationDrifted.set(finalQuaternionDrifted);

      interpolateOrientation(time - dtForFiniteDifference, initialQuaternionDrifted, finalQuaternionDrifted, qInterpolatedPrevious);
      interpolateOrientation(time + dtForFiniteDifference, initialQuaternionDrifted, finalQuaternionDrifted, qInterpolatedNext);

      quaternionCalculus.computeQDotByFiniteDifferenceCentral(qInterpolatedPrevious, qInterpolatedNext, dtForFiniteDifference, qDot);
      quaternionCalculus.computeQDDotByFiniteDifferenceCentral(qInterpolatedPrevious, qInterpolated, qInterpolatedNext, dtForFiniteDifference, qDDot);

      quaternionCalculus.computeAngularVelocity(qInterpolated, qDot, tempAngularVelocity);
      quaternionCalculus.computeAngularAcceleration(qInterpolated, qDot, qDDot, tempAngularAcceleration);

      desiredOrientation.set(qInterpolated);
      desiredAngularVelocity.set(tempAngularVelocity);
      desiredAngularAcceleration.set(tempAngularAcceleration);
   }

   private final Quat4d initialDrift = new Quat4d();
   private final Quat4d finalDrift = new Quat4d();
   private final Quat4d interpolatedDrift = new Quat4d();

   private void interpolateOrientation(double time, Quat4d initialQuaternionDriftedToPack, Quat4d finalQuaternionDriftedToPack, Quat4d qInterpolated)
   {
      parameterPolynomial.compute(time);

      double alpha = parameterPolynomial.getPosition();

      if (initialDriftSaturated.getBooleanValue())
      {
         double integratedAngle = initialAngularVelocityMagnitude.getDoubleValue() * time;
         saturationPolynomial.compute(MathTools.clipToMinMax(Math.abs(integratedAngle), 0.0, PI));
         double alphaSaturation = saturationPolynomial.getPosition();
         initialAlphaSaturation.set(alphaSaturation);
         computeDriftSaturated(time, alphaSaturation, initialAngularVelocity, initialAngularVelocityMagnitude, initialDrift);
      }
      else
      {
         computeDrift(time, initialAngularVelocity, initialDrift);
      }

      double finalDriftIntegrationTime = time - trajectoryTime.getDoubleValue();
      if (finalDriftSaturated.getBooleanValue())
      {
         double integratedAngle = finalAngularVelocityMagnitude.getDoubleValue() * finalDriftIntegrationTime;
         saturationPolynomial.compute(MathTools.clipToMinMax(Math.abs(integratedAngle), 0.0, PI));
         double alphaSaturation = saturationPolynomial.getPosition();
         finalAlphaSaturation.set(alphaSaturation);
         computeDriftSaturated(finalDriftIntegrationTime, alphaSaturation, finalAngularVelocity, finalAngularVelocityMagnitude, finalDrift);
      }
      else
      {
         computeDrift(finalDriftIntegrationTime, finalAngularVelocity, finalDrift);
      }

      initialOrientation.get(initialQuaternionDriftedToPack);
      finalOrientation.get(finalQuaternionDriftedToPack);
      quaternionCalculus.interpolate(alpha, initialQuaternionDriftedToPack, finalQuaternionDriftedToPack, qInterpolated, true);
      quaternionCalculus.interpolate(alpha, initialDrift, finalDrift, interpolatedDrift, false);
      qInterpolated.mul(interpolatedDrift, qInterpolated);

      initialQuaternionDriftedToPack.mul(initialDrift, initialQuaternionDriftedToPack);
      finalQuaternionDriftedToPack.mul(finalDrift, finalQuaternionDriftedToPack);
   }

   private final Vector3d tempAngularVelocityForDrift = new Vector3d();

   private void computeDrift(double time, YoFrameVector angularVelocity, Quat4d driftToPack)
   {
      angularVelocity.get(tempAngularVelocityForDrift);
      RotationTools.integrateAngularVelocity(tempAngularVelocityForDrift, time, driftToPack);
   }

   private void computeDriftSaturated(double time, double alphaSaturation, YoFrameVector angularVelocity, DoubleYoVariable angularVelocityMagnitude, Quat4d driftToPack)
   {
      angularVelocity.get(tempAngularVelocityForDrift);

      double maxTime = Math.signum(time) * PI / angularVelocityMagnitude.getDoubleValue() - 1.0e-5;
      time = alphaSaturation * maxTime + (1.0 - alphaSaturation) * time;

      RotationTools.integrateAngularVelocity(tempAngularVelocityForDrift, time, driftToPack);
   }

   public void getInitialOrientationDrifted(FrameOrientation initialOrientationDriftedToPack)
   {
      initialOrientationDrifted.getFrameOrientationIncludingFrame(initialOrientationDriftedToPack);
   }

   public void getFinalOrientationDrifted(FrameOrientation finalOrientationDriftedToPack)
   {
      finalOrientationDrifted.getFrameOrientationIncludingFrame(finalOrientationDriftedToPack);
   }

   @Override
   public boolean isDone()
   {
      return currentTime.getDoubleValue() >= trajectoryTime.getDoubleValue();
   }

   @Override
   public void get(FrameOrientation orientationToPack)
   {
      desiredOrientation.getFrameOrientationIncludingFrame(orientationToPack);
   }

   @Override
   public void packAngularVelocity(FrameVector velocityToPack)
   {
      desiredAngularVelocity.getFrameTupleIncludingFrame(velocityToPack);
   }

   @Override
   public void packAngularAcceleration(FrameVector accelerationToPack)
   {
      desiredAngularAcceleration.getFrameTupleIncludingFrame(accelerationToPack);
   }

   @Override
   public void packAngularData(FrameOrientation orientationToPack, FrameVector angularVelocityToPack, FrameVector angularAccelerationToPack)
   {
      get(orientationToPack);
      packAngularVelocity(angularVelocityToPack);
      packAngularAcceleration(angularAccelerationToPack);
   }
}
