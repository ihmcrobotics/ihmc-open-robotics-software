package us.ihmc.robotics.math.trajectories;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.Vector4D;
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
import us.ihmc.robotics.math.frames.YoFrameQuaternionInMultipleFrames;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.frames.YoFrameVectorInMultipleFrames;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

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

   private final YoFrameQuaternion currentOrientation;
   private final YoFrameVector currentAngularVelocity;
   private final YoFrameVector currentAngularAcceleration;

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
      this(namePrefix, false, referenceFrame, parentRegistry);
   }

   public VelocityConstrainedOrientationTrajectoryGenerator(String namePrefix, boolean allowMultipleFrames, ReferenceFrame referenceFrame, YoVariableRegistry parentRegistry)
   {
      super(allowMultipleFrames, referenceFrame);

      registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());
      trajectoryTime = new DoubleYoVariable(namePrefix + "TrajectoryTime", registry);
      currentTime = new DoubleYoVariable(namePrefix + "Time", registry);
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

      if (allowMultipleFrames)
      {
         YoFrameQuaternionInMultipleFrames initialOrientation = new YoFrameQuaternionInMultipleFrames(namePrefix + initialOrientationName, registry, trajectoryFrame);
         YoFrameQuaternionInMultipleFrames initialOrientationDrifted = new YoFrameQuaternionInMultipleFrames(namePrefix + initialOrientationDriftedName, registry, trajectoryFrame);
         YoFrameVectorInMultipleFrames initialAngularVelocity = new YoFrameVectorInMultipleFrames(namePrefix + initialAngularVelocityName, registry, trajectoryFrame);
         YoFrameQuaternionInMultipleFrames finalOrientation = new YoFrameQuaternionInMultipleFrames(namePrefix + finalOrientationName, registry, trajectoryFrame);
         YoFrameQuaternionInMultipleFrames finalOrientationDrifted = new YoFrameQuaternionInMultipleFrames(namePrefix + finalOrientationDriftedName, registry, trajectoryFrame);
         YoFrameVectorInMultipleFrames finalAngularVelocity = new YoFrameVectorInMultipleFrames(namePrefix + finalAngularVelocityName, registry, trajectoryFrame);

         YoFrameQuaternionInMultipleFrames currentOrientation = new YoFrameQuaternionInMultipleFrames(namePrefix + currentOrientationName, registry, trajectoryFrame);
         YoFrameVectorInMultipleFrames currentAngularVelocity = new YoFrameVectorInMultipleFrames(namePrefix + currentAngularVelocityName, registry, trajectoryFrame);
         YoFrameVectorInMultipleFrames currentAngularAcceleration = new YoFrameVectorInMultipleFrames(namePrefix + currentAngularAccelerationName, registry, trajectoryFrame);

         registerMultipleFramesHolders(initialOrientation, initialOrientationDrifted, initialAngularVelocity);
         registerMultipleFramesHolders(finalOrientation, finalOrientationDrifted, finalAngularVelocity);
         registerMultipleFramesHolders(currentOrientation, currentAngularVelocity, currentAngularAcceleration);

         this.initialOrientation = initialOrientation;
         this.initialOrientationDrifted = initialOrientationDrifted;
         this.initialAngularVelocity = initialAngularVelocity;
         this.finalOrientation = finalOrientation;
         this.finalOrientationDrifted = finalOrientationDrifted;
         this.finalAngularVelocity = finalAngularVelocity;
         this.currentOrientation = currentOrientation;
         this.currentAngularVelocity = currentAngularVelocity;
         this.currentAngularAcceleration = currentAngularAcceleration;
      }
      else
      {
         initialOrientation = new YoFrameQuaternion(namePrefix + initialOrientationName, trajectoryFrame, registry);
         initialOrientationDrifted = new YoFrameQuaternion(namePrefix + initialOrientationDriftedName, trajectoryFrame, registry);
         initialAngularVelocity = new YoFrameVector(namePrefix + initialAngularVelocityName, trajectoryFrame, registry);
         finalOrientation = new YoFrameQuaternion(namePrefix + finalOrientationName, trajectoryFrame, registry);
         finalOrientationDrifted = new YoFrameQuaternion(namePrefix + finalOrientationDriftedName, trajectoryFrame, registry);
         finalAngularVelocity = new YoFrameVector(namePrefix + finalAngularVelocityName, trajectoryFrame, registry);

         currentOrientation = new YoFrameQuaternion(namePrefix + currentOrientationName, trajectoryFrame, registry);
         currentAngularVelocity = new YoFrameVector(namePrefix + currentAngularVelocityName, trajectoryFrame, registry);
         currentAngularAcceleration = new YoFrameVector(namePrefix + currentAngularAccelerationName, trajectoryFrame, registry);
      }

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

   public void setInitialOrientation(YoFrameQuaternion initialOrientation)
   {
      initialOrientation.getFrameOrientationIncludingFrame(tempOrientation);
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

   public void setFinalOrientation(YoFrameQuaternion finalOrientation)
   {
      finalOrientation.getFrameOrientationIncludingFrame(tempOrientation);
      tempOrientation.changeFrame(trajectoryFrame);
      this.finalOrientation.set(tempOrientation);
   }

   public void setInitialAngularVelocity(FrameVector initialAngularVelocity)
   {
      this.initialAngularVelocity.setAndMatchFrame(initialAngularVelocity);
   }

   public void setInitialAngularVelocity(YoFrameVector initialAngularVelocity)
   {
      this.initialAngularVelocity.setAndMatchFrame(initialAngularVelocity);
   }

   public void setFinalAngularVelocity(FrameVector finalAngularVelocity)
   {
      this.finalAngularVelocity.setAndMatchFrame(finalAngularVelocity);
   }

   public void setFinalAngularVelocity(YoFrameVector finalAngularVelocity)
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

   public void setInitialConditions(YoFrameQuaternion initialOrientation, YoFrameVector initialAngularVelocity)
   {
      setInitialOrientation(initialOrientation);
      setInitialAngularVelocity(initialAngularVelocity);
   }

   public void setFinalConditions(FrameOrientation finalOrientation, FrameVector finalAngularVelocity)
   {
      setFinalOrientation(finalOrientation);
      setFinalAngularVelocity(finalAngularVelocity);
   }

   public void setFinalConditions(YoFrameQuaternion finalOrientation, YoFrameVector finalAngularVelocity)
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

      initialOrientation.get(initialQuaternionDriftedToPack);
      finalOrientation.get(finalQuaternionDriftedToPack);
      quaternionCalculus.interpolate(alpha, initialQuaternionDriftedToPack, finalQuaternionDriftedToPack, qInterpolated, true);
      quaternionCalculus.interpolate(alpha, initialDrift, finalDrift, interpolatedDrift, false);
      qInterpolated.multiply(interpolatedDrift, qInterpolated);

      initialQuaternionDriftedToPack.multiply(initialDrift, initialQuaternionDriftedToPack);
      finalQuaternionDriftedToPack.multiply(finalDrift, finalQuaternionDriftedToPack);
   }

   private final Vector3D tempAngularVelocityForDrift = new Vector3D();

   private void computeDrift(double time, double alphaDecay, YoFrameVector angularVelocity, Quaternion driftToPack)
   {
      angularVelocity.get(tempAngularVelocityForDrift);
      tempAngularVelocity.scale(alphaDecay);
      RotationTools.integrateAngularVelocity(tempAngularVelocityForDrift, time, driftToPack);
   }

   private void computeDriftSaturated(double time, double alphaSaturation, YoFrameVector angularVelocity, DoubleYoVariable angularVelocityMagnitude, Quaternion driftToPack)
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
   public void getOrientation(FrameOrientation orientationToPack)
   {
      currentOrientation.getFrameOrientationIncludingFrame(orientationToPack);
   }

   @Override
   public void getAngularVelocity(FrameVector velocityToPack)
   {
      currentAngularVelocity.getFrameTupleIncludingFrame(velocityToPack);
   }

   @Override
   public void getAngularAcceleration(FrameVector accelerationToPack)
   {
      currentAngularAcceleration.getFrameTupleIncludingFrame(accelerationToPack);
   }

   @Override
   public void getAngularData(FrameOrientation orientationToPack, FrameVector angularVelocityToPack, FrameVector angularAccelerationToPack)
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
