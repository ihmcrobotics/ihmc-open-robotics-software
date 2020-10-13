package us.ihmc.robotics.math.trajectories;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.Vector4D;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.robotics.geometry.RotationTools;
import us.ihmc.robotics.math.QuaternionCalculus;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameQuaternion;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoMutableFrameQuaternion;
import us.ihmc.yoVariables.euclid.referenceFrame.YoMutableFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * This trajectory generator aims at interpolating between two orientations q0 and qf for given angular velocities at the limits w0 and wf.
 * It does a nice job for "long distance" type trajectories but totally fails when used for interpolating between, it generates extra accelerations instead of being "lazy".
 * Look at {@link HermiteCurveBasedOrientationTrajectoryGenerator} for interpolating between waypoints.
 *
 * To build this trajectory generator, I got inspired by the two following refs.
 * <p> a) <a href="http://www.geometrictools.com/Documentation/Quaternions.pdf">(SLERP & SQUAD methods)</a> </p>
 * <p> b) <a href="http://www.cim.mcgill.ca/~mpersson/docs/quat_calc_notes.pdf">(Proper quaternion calculus, this link seems to be dead, but I can't find a new one. Ask me for the PDF file.)</a> </p>
 *
 * I got inspired by the first reference with the SQUAD method, but watch out there the math seems to be totally wrong.
 * The second reference provides a proper approach to general quaternion calculus. I used it a lot to derive time derivatives with quaternion & angular velocity. Really useful ref.
 *
 * The method I'm using here is as follows:
 * - I interpolated from the initial orientation to the final orientation using the classic SLERP method using a fifth-order polynomial function to avoid generating angular velocity at the limits.
 * - To induce the initial and final desired angular velocities I make the initial and final orientations drift at a constant velocity (the initial and final angular velocities respectively).
 *
 * I ended up giving up on computing the angular velocity analytically. It seems that computing the derivative of a quaternion q raised to the power alpha where both q and alpha depend on time is still an open problem.
 * So I'm just computing the angular velocity and angular acceleration using finite difference.
 * Also, I had some issues when the initial and/or final angular velocity are large.
 * As soon as the integrated (initial or final) velocity over the trajectory goes beyond 2 Pi, there is some flip going.
 * It seems that it is totally doable to deal with those flips, but I thought that on a robot you probably don't want to flip a hand or a foot for instance.
 * So I ended up saturating the drifted orientation when the angular velocity at the bounds are too high.
 *
 * @author Sylvain
 */
public class VelocityConstrainedOrientationTrajectoryGenerator extends OrientationTrajectoryGeneratorInMultipleFrames
{
   private static final double PI = 1.0 * Math.PI;
   private final YoRegistry registry;
   private final YoDouble currentTime;
   private final YoDouble trajectoryTime;
   private final YoPolynomial parameterPolynomial;

   private final FrameQuaternionBasics initialOrientation;
   private final FrameQuaternionBasics initialOrientationDrifted;
   private final FrameVector3DBasics initialAngularVelocity;
   private final FrameQuaternionBasics finalOrientation;
   private final FrameQuaternionBasics finalOrientationDrifted;
   private final FrameVector3DBasics finalAngularVelocity;

   private final FrameQuaternionBasics currentOrientation;
   private final FrameVector3DBasics currentAngularVelocity;
   private final FrameVector3DBasics currentAngularAcceleration;

   private final YoPolynomial saturationPolynomial;
   private final YoDouble maxAngularVelocityMagnitudeAtLimits;
   private final YoDouble initialAngularVelocityMagnitude;
   private final YoDouble finalAngularVelocityMagnitude;
   private final YoBoolean initialDriftSaturated;
   private final YoBoolean finalDriftSaturated;
   private final YoDouble initialAlphaSaturation;
   private final YoDouble finalAlphaSaturation;

   private final ReferenceFrame trajectoryFrame;

   private final QuaternionCalculus quaternionCalculus = new QuaternionCalculus();

   /**
    * Does not need to match the dt at which the trajectory will be updated.
    */
   private final double dtForFiniteDifference = 1.0e-4;

   public VelocityConstrainedOrientationTrajectoryGenerator(String namePrefix, ReferenceFrame referenceFrame, YoRegistry parentRegistry)
   {
      registry = new YoRegistry(namePrefix + getClass().getSimpleName());
      trajectoryTime = new YoDouble(namePrefix + "TrajectoryTime", registry);
      currentTime = new YoDouble(namePrefix + "Time", registry);
      parameterPolynomial = new YoPolynomial(namePrefix + "ParameterPolynomial", 6, registry);
      trajectoryFrame = referenceFrame;

      String initialOrientationName = "InitialOrientation";
      String initialOrientationDriftedName = "InitialOrientationDrifted";
      String initialAngularVelocityName = "InitialAngularVelocity";
      String finalOrientationName = "FinalOrientation";
      String finalOrientationDriftedName = "FinalOrientationDrifted";
      String finalAngularVelocityName = "FinalAngularVelocity";
      String currentOrientationName = "CurrentOrientation";
      String currentAngularVelocityName = "CurrentAngularVelocity";
      String currentAngularAccelerationName = "CurrentAngularAcceleration";

      initialOrientation = new YoMutableFrameQuaternion(namePrefix + initialOrientationName, "", registry, trajectoryFrame);
      initialOrientationDrifted = new YoMutableFrameQuaternion(namePrefix + initialOrientationDriftedName, "", registry, trajectoryFrame);
      initialAngularVelocity = new YoMutableFrameVector3D(namePrefix + initialAngularVelocityName, "", registry, trajectoryFrame);
      finalOrientation = new YoMutableFrameQuaternion(namePrefix + finalOrientationName, "", registry, trajectoryFrame);
      finalOrientationDrifted = new YoMutableFrameQuaternion(namePrefix + finalOrientationDriftedName, "", registry, trajectoryFrame);
      finalAngularVelocity = new YoMutableFrameVector3D(namePrefix + finalAngularVelocityName, "", registry, trajectoryFrame);

      currentOrientation = new YoMutableFrameQuaternion(namePrefix + currentOrientationName, "", registry, trajectoryFrame);
      currentAngularVelocity = new YoMutableFrameVector3D(namePrefix + currentAngularVelocityName, "", registry, trajectoryFrame);
      currentAngularAcceleration = new YoMutableFrameVector3D(namePrefix + currentAngularAccelerationName, "", registry, trajectoryFrame);

      registerFrameChangeables(initialOrientation, initialOrientationDrifted, initialAngularVelocity);
      registerFrameChangeables(finalOrientation, finalOrientationDrifted, finalAngularVelocity);
      registerFrameChangeables(currentOrientation, currentAngularVelocity, currentAngularAcceleration);

      saturationPolynomial = new YoPolynomial(namePrefix + "SaturationPolynomial", 6, registry);
      saturationPolynomial.setQuintic(0, PI, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
      maxAngularVelocityMagnitudeAtLimits = new YoDouble(namePrefix + "MaxAngularVelocityMagnitudeAtLimits", registry);
      initialAngularVelocityMagnitude = new YoDouble(namePrefix + "InitialAngularVelocityMagnitude", registry);
      finalAngularVelocityMagnitude = new YoDouble(namePrefix + "FinalAngularVelocityMagnitude", registry);

      initialDriftSaturated = new YoBoolean(namePrefix + "InitialDriftSaturated", registry);
      finalDriftSaturated = new YoBoolean(namePrefix + "FinalDriftSaturated", registry);
      initialAlphaSaturation = new YoDouble(namePrefix + "InitialAlphaSaturation", registry);
      finalAlphaSaturation = new YoDouble(namePrefix + "FinalAlphaSaturation", registry);

      parentRegistry.addChild(registry);
   }

   public void setTrajectoryTime(double duration)
   {
      MathTools.checkIntervalContains(duration, 0.0, Double.POSITIVE_INFINITY);
      trajectoryTime.set(duration);
   }

   public void setInitialOrientation(FrameQuaternionReadOnly initialOrientation)
   {
      this.initialOrientation.setMatchingFrame(initialOrientation);
   }

   public void setFinalOrientation(FrameQuaternionReadOnly finalOrientation)
   {
      this.finalOrientation.setMatchingFrame(finalOrientation);
   }

   public void setFinalOrientation(FramePose3D finalPose)
   {
      this.finalOrientation.setMatchingFrame(finalPose.getOrientation());
   }

   public void setInitialAngularVelocity(FrameVector3DReadOnly initialAngularVelocity)
   {
      this.initialAngularVelocity.setMatchingFrame(initialAngularVelocity);
   }

   public void setFinalAngularVelocity(FrameVector3DReadOnly finalAngularVelocity)
   {
      this.finalAngularVelocity.setMatchingFrame(finalAngularVelocity);
   }

   public void setInitialVelocityToZero()
   {
      initialAngularVelocity.setToZero();
   }

   public void setFinalVelocityToZero()
   {
      finalAngularVelocity.setToZero();
   }

   public void setInitialConditions(FrameQuaternion initialOrientation, FrameVector3D initialAngularVelocity)
   {
      setInitialOrientation(initialOrientation);
      setInitialAngularVelocity(initialAngularVelocity);
   }

   public void setInitialConditions(YoFrameQuaternion initialOrientation, YoFrameVector3D initialAngularVelocity)
   {
      setInitialOrientation(initialOrientation);
      setInitialAngularVelocity(initialAngularVelocity);
   }

   public void setFinalConditions(FrameQuaternion finalOrientation, FrameVector3D finalAngularVelocity)
   {
      setFinalOrientation(finalOrientation);
      setFinalAngularVelocity(finalAngularVelocity);
   }

   public void setFinalConditions(YoFrameQuaternion finalOrientation, YoFrameVector3D finalAngularVelocity)
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

      currentOrientation.set(initialOrientation);
      currentAngularVelocity.set(initialAngularVelocity);
      currentAngularAcceleration.setToZero();
   }

   private final Quaternion initialQuaternionDrifted = new Quaternion();
   private final Quaternion finalQuaternionDrifted = new Quaternion();

   private final Vector3D tempAngularVelocity = new Vector3D();
   private final Vector3D tempAngularAcceleration = new Vector3D();

   private final Quaternion qInterpolatedPrevious = new Quaternion();
   private final Quaternion qInterpolated = new Quaternion();
   private final Quaternion qInterpolatedNext = new Quaternion();

   private final Vector4D qDot = new Vector4D();
   private final Vector4D qDDot = new Vector4D();

   @Override
   public void compute(double time)
   {
      this.currentTime.set(time);

      if (isDone())
      {
         currentOrientation.set(finalOrientation);
         currentAngularVelocity.set(finalAngularVelocity);
         currentAngularAcceleration.setToZero();
         return;
      }
      else if (currentTime.getDoubleValue() < 0.0)
      {
         currentOrientation.set(initialOrientation);
         currentAngularVelocity.set(initialAngularVelocity);
         currentAngularAcceleration.setToZero();
         return;
      }

      time = MathTools.clamp(time, 0.0, trajectoryTime.getDoubleValue());

      interpolateOrientation(time - dtForFiniteDifference, initialQuaternionDrifted, finalQuaternionDrifted, qInterpolatedPrevious);
      interpolateOrientation(time + dtForFiniteDifference, initialQuaternionDrifted, finalQuaternionDrifted, qInterpolatedNext);
      interpolateOrientation(time, initialQuaternionDrifted, finalQuaternionDrifted, qInterpolated);

      initialOrientationDrifted.set(initialQuaternionDrifted);
      finalOrientationDrifted.set(finalQuaternionDrifted);

      quaternionCalculus.computeQDotByFiniteDifferenceCentral(qInterpolatedPrevious, qInterpolatedNext, dtForFiniteDifference, qDot);
      quaternionCalculus.computeQDDotByFiniteDifferenceCentral(qInterpolatedPrevious, qInterpolated, qInterpolatedNext, dtForFiniteDifference, qDDot);

      quaternionCalculus.computeAngularVelocityInWorldFrame(qInterpolated, qDot, tempAngularVelocity);
      quaternionCalculus.computeAngularAcceleration(qInterpolated, qDot, qDDot, tempAngularAcceleration);

      currentOrientation.set(qInterpolated);
      currentAngularVelocity.set(tempAngularVelocity);
      currentAngularAcceleration.set(tempAngularAcceleration);
   }

   private final Quaternion initialDrift = new Quaternion();
   private final Quaternion finalDrift = new Quaternion();
   private final Quaternion interpolatedDrift = new Quaternion();

   private void interpolateOrientation(double time, Quaternion initialQuaternionDriftedToPack, Quaternion finalQuaternionDriftedToPack, Quaternion qInterpolated)
   {
      parameterPolynomial.compute(time);
      double alpha = parameterPolynomial.getPosition();

      if (initialDriftSaturated.getBooleanValue())
      {
         double integratedAngle = initialAngularVelocityMagnitude.getDoubleValue() * time;
         saturationPolynomial.compute(MathTools.clamp(Math.abs(integratedAngle), 0.0, PI));
         double alphaSaturation = saturationPolynomial.getPosition();
         initialAlphaSaturation.set(alphaSaturation);
         computeDriftSaturated(time, alphaSaturation, initialAngularVelocity, initialAngularVelocityMagnitude, initialDrift);
      }
      else
      {
         double alphaDecay = MathTools.clamp(2.0 * time / trajectoryTime.getDoubleValue(), 0.0, 1.0);
         computeDrift(time, alphaDecay, initialAngularVelocity, initialDrift);
      }

      double finalDriftIntegrationTime = time - trajectoryTime.getDoubleValue();
      if (finalDriftSaturated.getBooleanValue())
      {
         double integratedAngle = finalAngularVelocityMagnitude.getDoubleValue() * finalDriftIntegrationTime;
         saturationPolynomial.compute(MathTools.clamp(Math.abs(integratedAngle), 0.0, PI));
         double alphaSaturation = saturationPolynomial.getPosition();
         finalAlphaSaturation.set(alphaSaturation);
         computeDriftSaturated(finalDriftIntegrationTime, alphaSaturation, finalAngularVelocity, finalAngularVelocityMagnitude, finalDrift);
      }
      else
      {
         double alphaDecay = MathTools.clamp(1.0 - 2.0 * time / trajectoryTime.getDoubleValue(), 0.0, 1.0);
         computeDrift(finalDriftIntegrationTime, alphaDecay, finalAngularVelocity, finalDrift);
      }

      initialQuaternionDriftedToPack.set(initialOrientation);
      finalQuaternionDriftedToPack.set(finalOrientation);
      quaternionCalculus.interpolate(alpha, initialQuaternionDriftedToPack, finalQuaternionDriftedToPack, qInterpolated, true);
      quaternionCalculus.interpolate(alpha, initialDrift, finalDrift, interpolatedDrift, false);
      qInterpolated.multiply(interpolatedDrift, qInterpolated);

      initialQuaternionDriftedToPack.multiply(initialDrift, initialQuaternionDriftedToPack);
      finalQuaternionDriftedToPack.multiply(finalDrift, finalQuaternionDriftedToPack);
   }

   private final Vector3D tempAngularVelocityForDrift = new Vector3D();

   private void computeDrift(double time, double alphaDecay, FrameVector3DReadOnly angularVelocity, QuaternionBasics driftToPack)
   {
      tempAngularVelocityForDrift.set(angularVelocity);
      tempAngularVelocity.scale(alphaDecay);
      RotationTools.integrateAngularVelocity(tempAngularVelocityForDrift, time, driftToPack);
   }

   private void computeDriftSaturated(double time, double alphaSaturation, FrameVector3DReadOnly angularVelocity, YoDouble angularVelocityMagnitude,
                                      QuaternionBasics driftToPack)
   {
      tempAngularVelocityForDrift.set(angularVelocity);

      double maxTime = Math.signum(time) * PI / angularVelocityMagnitude.getDoubleValue() - 1.0e-5;
      time = alphaSaturation * maxTime + (1.0 - alphaSaturation) * time;

      RotationTools.integrateAngularVelocity(tempAngularVelocityForDrift, time, driftToPack);
   }

   public void getInitialOrientationDrifted(FrameQuaternion initialOrientationDriftedToPack)
   {
      initialOrientationDriftedToPack.setIncludingFrame(initialOrientationDrifted);
   }

   public void getFinalOrientationDrifted(FrameQuaternion finalOrientationDriftedToPack)
   {
      finalOrientationDriftedToPack.setIncludingFrame(finalOrientationDrifted);
   }

   @Override
   public boolean isDone()
   {
      return currentTime.getDoubleValue() >= trajectoryTime.getDoubleValue();
   }

   @Override
   public void getOrientation(FrameQuaternion orientationToPack)
   {
      orientationToPack.setIncludingFrame(currentOrientation);
   }

   @Override
   public void getAngularVelocity(FrameVector3D velocityToPack)
   {
      velocityToPack.setIncludingFrame(currentAngularVelocity);
   }

   @Override
   public void getAngularAcceleration(FrameVector3D accelerationToPack)
   {
      accelerationToPack.setIncludingFrame(currentAngularAcceleration);
   }

   @Override
   public void getAngularData(FrameQuaternion orientationToPack, FrameVector3D angularVelocityToPack, FrameVector3D angularAccelerationToPack)
   {
      getOrientation(orientationToPack);
      getAngularVelocity(angularVelocityToPack);
      getAngularAcceleration(angularAccelerationToPack);
   }

   @Override
   public String toString()
   {
      String ret = "";

      ret += "Current time: " + currentTime.getDoubleValue() + ", trajectory time: " + trajectoryTime.getDoubleValue();
      ret += "\nCurrent orientation: " + currentOrientation.toString();
      ret += "\nCurrent angular velocity: " + currentAngularVelocity.toString();
      ret += "\nCurrent angular acceleration: " + currentAngularAcceleration.toString();
      return ret;
   }
}
