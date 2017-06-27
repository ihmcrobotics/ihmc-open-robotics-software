package us.ihmc.robotics.math.trajectories;

import us.ihmc.euclid.tools.QuaternionTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.Vector4D;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.Vector4DReadOnly;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.QuaternionCalculus;
import us.ihmc.robotics.math.frames.YoFrameQuaternion;
import us.ihmc.robotics.math.frames.YoFrameQuaternionInMultipleFrames;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.frames.YoFrameVectorInMultipleFrames;
import us.ihmc.robotics.math.trajectories.waypoints.FrameSO3TrajectoryPoint;
import us.ihmc.robotics.math.trajectories.waypoints.YoFrameSO3TrajectoryPoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

/**
 * This trajectory generator aims at interpolating between two orientations q0 and qf for given
 * angular velocities at the limits w0 and wf. The method used here differs from the trajectory
 * generator implemented in {@link VelocityConstrainedOrientationTrajectoryGenerator}. It seems that
 * the approach used here is a better fit for interpolating between waypoints.
 *
 * I basically implemented the method called Hermite Quaternion Curve that is presented in the
 * following paper:
 * <p>
 * <a href="http://azrael.digipen.edu/MAT351/papers/Kim2.pdf"> Paper on quaternion interpolation
 * (PDF link 1)</a>
 * </p>
 * <p>
 * <a href="http://graphics.cs.cmu.edu/nsp/course/15-464/Fall05/papers/kimKimShin.pdf"> Paper on
 * quaternion interpolation (PDF link 2)</a>
 * </p>
 * <p>
 * <a href=
 * "https://www.researchgate.net/publication/2388093_A_General_Construction_Scheme_for_Unit_Quaternion_Curves_with_Simple_High_Order_Derivatives">
 * Paper on quaternion interpolation (ResearchGate link)</a>
 * </p>
 * <p>
 * See also the Word document for deriving the angular velocity and angular acceleration:
 * <a href="https://1drv.ms/w/s!AtjeMRpLgFtkiPswDYIdKoj43dLNUw">Angular Velocity & Acceleration</a>.
 * </p>
 *
 * Watch out for the typo in equation 9 when reading the paper though. The beta on the right hand
 * side of the equation should have for subscripts {j,n} instead of {i,n}. Also the statement that
 * arbitrary high velocities can be assigned is false. Both endpoint velocities after scaling with
 * the trajectory time have a maximum magnitude of 6 pi.
 *
 * @author Sylvain
 */
public class HermiteCurveBasedOrientationTrajectoryGenerator extends OrientationTrajectoryGeneratorInMultipleFrames
{
   private final YoVariableRegistry registry;
   private final YoDouble currentTime;
   private final YoDouble trajectoryTime;
   private final YoDouble trajectoryTimeScale;
   private final YoInteger piInteger;

   private final YoDouble[] cumulativeBeziers;
   private final YoDouble[] cumulativeBeziersDot;
   private final YoDouble[] cumulativeBeziersDDot;

   private final YoFrameQuaternion[] yoControlQuaternions;
   private final YoFrameVector[] controlAngularVelocities;

   private final YoFrameQuaternion initialOrientation;
   private final YoFrameVector initialAngularVelocity;
   private final YoFrameQuaternion finalOrientation;
   private final YoFrameVector finalAngularVelocity;

   private final YoFrameQuaternion currentOrientation;
   private final YoFrameVector currentAngularVelocity;
   private final YoFrameVector currentAngularVelocityFD;
   private final YoFrameVector currentAngularAcceleration;
   private final YoFrameVector currentAngularAccelerationFD;

   private final ReferenceFrame trajectoryFrame;

   private final QuaternionCalculus quaternionCalculus = new QuaternionCalculus();

   /**
    * Does not need to match the dt at which the trajectory will be updated.
    */
   private final double dtForFiniteDifference = 1.0e-3;

   public HermiteCurveBasedOrientationTrajectoryGenerator(String name, ReferenceFrame referenceFrame, YoVariableRegistry parentRegistry)
   {
      this(name, false, referenceFrame, parentRegistry);
   }

   public HermiteCurveBasedOrientationTrajectoryGenerator(String name, boolean allowMultipleFrames, ReferenceFrame referenceFrame,
                                                          YoVariableRegistry parentRegistry)
   {
      super(allowMultipleFrames, referenceFrame);

      registry = new YoVariableRegistry(name);
      trajectoryTime = new YoDouble(name + "TrajectoryTime", registry);
      trajectoryTimeScale = new YoDouble(name + "TrajectoryTimeScale", registry);
      piInteger = new YoInteger(name + "PiInteger", registry);
      currentTime = new YoDouble(name + "Time", registry);
      trajectoryFrame = referenceFrame;

      cumulativeBeziers = new YoDouble[4];
      cumulativeBeziersDot = new YoDouble[4];
      cumulativeBeziersDDot = new YoDouble[4];
      yoControlQuaternions = new YoFrameQuaternion[4];
      controlAngularVelocities = new YoFrameVector[4];

      for (int i = 1; i <= 3; i++)
      {
         cumulativeBeziers[i] = new YoDouble(name + "CumulativeBezier" + i, registry);
         cumulativeBeziersDot[i] = new YoDouble(name + "CumulativeBezierDot" + i, registry);
         cumulativeBeziersDDot[i] = new YoDouble(name + "CumulativeBezierDDot" + i, registry);
      }

      String initialOrientationName = "InitialOrientation";
      String initialAngularVelocityName = "InitialAngularVelocity";
      String finalOrientationName = "FinalOrientation";
      String finalAngularVelocityName = "FinalAngularVelocity";
      String currentOrientationName = "CurrentOrientation";
      String currentAngularVelocityName = "CurrentAngularVelocity";
      String currentAngularAccelerationName = "CurrentAngularAcceleration";
      String controlQuaternionName = "ControlQuaternion";
      String controlAngularVelocityName = "ControlAngularVelocity";

      if (allowMultipleFrames)
      {
         YoFrameQuaternionInMultipleFrames initialOrientation = new YoFrameQuaternionInMultipleFrames(name + initialOrientationName, registry, trajectoryFrame);
         YoFrameVectorInMultipleFrames initialAngularVelocity = new YoFrameVectorInMultipleFrames(name + initialAngularVelocityName, registry, trajectoryFrame);
         YoFrameQuaternionInMultipleFrames finalOrientation = new YoFrameQuaternionInMultipleFrames(name + finalOrientationName, registry, trajectoryFrame);
         YoFrameVectorInMultipleFrames finalAngularVelocity = new YoFrameVectorInMultipleFrames(name + finalAngularVelocityName, registry, trajectoryFrame);

         YoFrameQuaternionInMultipleFrames currentOrientation = new YoFrameQuaternionInMultipleFrames(name + currentOrientationName, registry, trajectoryFrame);
         YoFrameVectorInMultipleFrames currentAngularVelocity = new YoFrameVectorInMultipleFrames(name + currentAngularVelocityName, registry, trajectoryFrame);
         YoFrameVectorInMultipleFrames currentAngularVelocityFD = new YoFrameVectorInMultipleFrames(name + currentAngularVelocityName + "FD", registry,
                                                                                                    trajectoryFrame);
         YoFrameVectorInMultipleFrames currentAngularAcceleration = new YoFrameVectorInMultipleFrames(name + currentAngularAccelerationName, registry,
                                                                                                      trajectoryFrame);
         YoFrameVectorInMultipleFrames currentAngularAccelerationFD = new YoFrameVectorInMultipleFrames(name + currentAngularAccelerationName + "FD", registry,
                                                                                                        trajectoryFrame);

         registerMultipleFramesHolders(initialOrientation, initialAngularVelocity);
         registerMultipleFramesHolders(finalOrientation, finalAngularVelocity);
         registerMultipleFramesHolders(currentOrientation, currentAngularVelocity, currentAngularAcceleration);

         this.initialOrientation = initialOrientation;
         this.initialAngularVelocity = initialAngularVelocity;
         this.finalOrientation = finalOrientation;
         this.finalAngularVelocity = finalAngularVelocity;
         this.currentOrientation = currentOrientation;
         this.currentAngularVelocity = currentAngularVelocity;
         this.currentAngularVelocityFD = currentAngularVelocityFD;
         this.currentAngularAcceleration = currentAngularAcceleration;
         this.currentAngularAccelerationFD = currentAngularAccelerationFD;

         for (int i = 0; i <= 3; i++)
         {
            YoFrameQuaternionInMultipleFrames controlQuaternion = new YoFrameQuaternionInMultipleFrames(name + controlQuaternionName + i, registry,
                                                                                                        trajectoryFrame);
            registerMultipleFramesHolders(controlQuaternion);
            yoControlQuaternions[i] = controlQuaternion;
         }

         for (int i = 1; i <= 3; i++)
         {
            YoFrameVectorInMultipleFrames controlAngularVelocity = new YoFrameVectorInMultipleFrames(name + controlAngularVelocityName + i, registry,
                                                                                                     trajectoryFrame);
            registerMultipleFramesHolders(controlAngularVelocity);
            controlAngularVelocities[i] = controlAngularVelocity;
         }
      }
      else
      {
         initialOrientation = new YoFrameQuaternion(name + initialOrientationName, trajectoryFrame, registry);
         initialAngularVelocity = new YoFrameVector(name + initialAngularVelocityName, trajectoryFrame, registry);
         finalOrientation = new YoFrameQuaternion(name + finalOrientationName, trajectoryFrame, registry);
         finalAngularVelocity = new YoFrameVector(name + finalAngularVelocityName, trajectoryFrame, registry);

         currentOrientation = new YoFrameQuaternion(name + currentOrientationName, trajectoryFrame, registry);
         currentAngularVelocity = new YoFrameVector(name + currentAngularVelocityName, trajectoryFrame, registry);
         currentAngularVelocityFD = new YoFrameVector(name + currentAngularVelocityName + "FD", trajectoryFrame, registry);
         currentAngularAcceleration = new YoFrameVector(name + currentAngularAccelerationName, trajectoryFrame, registry);
         currentAngularAccelerationFD = new YoFrameVector(name + currentAngularAccelerationName + "FD", trajectoryFrame, registry);

         for (int i = 0; i <= 3; i++)
            yoControlQuaternions[i] = new YoFrameQuaternion(name + controlQuaternionName + i, trajectoryFrame, registry);

         for (int i = 1; i <= 3; i++)
            controlAngularVelocities[i] = new YoFrameVector(name + controlAngularVelocityName + i, trajectoryFrame, registry);
      }

      parentRegistry.addChild(registry);
   }

   public void setTrajectoryTime(double duration)
   {
      MathTools.checkIntervalContains(duration, 0.0, Double.POSITIVE_INFINITY);
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
      finalOrientation.set(tempOrientation);
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

   /**
    * Sets an Integer n to add n/2 full revolutions to the trajectory.
    *
    */
   public void setPiInteger(int piInteger)
   {
      this.piInteger.set(piInteger);
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

   private final FrameOrientation tempFrameOrientation = new FrameOrientation();
   private final FrameVector tempFrameVector = new FrameVector();

   public void setTrajectoryParameters(FrameSO3TrajectoryPoint initialFrameSO3Waypoint, FrameSO3TrajectoryPoint finalFrameSO3Waypoint)
   {
      //TODO: Clean this up by making an initialSO3Waypoint, finalSO3Waypoint as YoFrameSO3Waypoints.
      setTrajectoryTime(finalFrameSO3Waypoint.getTime() - initialFrameSO3Waypoint.getTime());

      initialFrameSO3Waypoint.getOrientationIncludingFrame(tempFrameOrientation);
      initialFrameSO3Waypoint.getAngularVelocityIncludingFrame(tempFrameVector);

      initialOrientation.set(tempFrameOrientation);
      initialAngularVelocity.set(tempFrameVector);

      finalFrameSO3Waypoint.getOrientationIncludingFrame(tempFrameOrientation);
      finalFrameSO3Waypoint.getAngularVelocityIncludingFrame(tempFrameVector);

      finalOrientation.set(tempFrameOrientation);
      finalAngularVelocity.set(tempFrameVector);
   }

   public void setTrajectoryParameters(YoFrameSO3TrajectoryPoint initialYoFrameSO3Waypoint, YoFrameSO3TrajectoryPoint finalYoFrameSO3Waypoint)
   {
      setTrajectoryTime(finalYoFrameSO3Waypoint.getTime() - initialYoFrameSO3Waypoint.getTime());

      initialOrientation.set(initialYoFrameSO3Waypoint.getOrientation());
      initialAngularVelocity.set(initialYoFrameSO3Waypoint.getAngularVelocity());

      finalOrientation.set(finalYoFrameSO3Waypoint.getOrientation());
      finalAngularVelocity.set(finalYoFrameSO3Waypoint.getAngularVelocity());
   }

   /**
    * Before Initializing the isSolvable() method should be called to confirm that the limit
    * conditions are within the bounds
    */
   @Override
   public void initialize()
   {
      piInteger.set(0);
      currentTime.set(0.0);
      trajectoryTimeScale.set(1.0 / trajectoryTime.getDoubleValue());

      if (initialOrientation.dot(finalOrientation) < 0.0)
         finalOrientation.negate();

      updateControlQuaternions();

      currentOrientation.set(initialOrientation);
      currentAngularVelocity.set(initialAngularVelocity);
      currentAngularVelocityFD.set(initialAngularVelocity);
      currentAngularAcceleration.setToZero();
      currentAngularAccelerationFD.setToZero();
   }

   private final Quaternion[] controlQuaternions = new Quaternion[] {new Quaternion(), new Quaternion(), new Quaternion(), new Quaternion()};

   private void updateControlQuaternions()
   {
      Quaternion q0 = controlQuaternions[0];
      Quaternion q1 = controlQuaternions[1];
      Quaternion q2 = controlQuaternions[2];
      Quaternion q3 = controlQuaternions[3];

      Vector3D omega = tempAngularVelocity;

      // q0 = qInitial
      initialOrientation.get(q0);
      // q3 = qFinal
      finalOrientation.get(q3);

      // q1 = qInitial * exp(omegaInitial / 3.0)
      initialOrientation.get(q1);
      initialAngularVelocity.get(omega);
      q0.inverseTransform(omega); // Switch to quaternion coordinates
      q1.multiply(exp(1.0 / 3.0, omega));

      // q2 = qFinal * exp(omegaFinal / 3.0)
      finalOrientation.get(q2);
      finalAngularVelocity.get(omega);
      q3.inverseTransform(omega); // Switch to quaternion coordinates
      q2.multiply(exp(-1.0 / 3.0, omega));

      for (int i = 1; i <= 3; i++)
      {
         controlAngularVelocities[i].set(logOfDifference(controlQuaternions[i - 1], controlQuaternions[i]));
      }

      if (piInteger.getIntegerValue() != 0)
      {
         controlAngularVelocities[2].get(omega);
         if (omega.lengthSquared() > 1.0e-10)
         {
            omega.normalize();
            omega.scale(piInteger.getIntegerValue() * Math.PI);
            controlAngularVelocities[2].add(omega);
         }
      }

      for (int i = 0; i <= 3; i++)
      {
         yoControlQuaternions[i].set(controlQuaternions[i]);
      }
   }

   private final Vector3D tempAngularVelocity = new Vector3D();
   private final Vector3D tempAngularAcceleration = new Vector3D();

   private final Quaternion qInterpolatedPrevious = new Quaternion();
   private final Quaternion qInterpolated = new Quaternion();
   private final Quaternion qInterpolatedNext = new Quaternion();

   private final Vector3D angularVelocityInterpolatedPrevious = new Vector3D();
   private final Vector3D angularVelocityInterpolated = new Vector3D();
   private final Vector3D angularVelocityInterpolatedNext = new Vector3D();

   private final Vector3D angularAccelerationInterpolated = new Vector3D();

   private final Vector4D qDot = new Vector4D();
   private final Vector4D qDDot = new Vector4D();

   @Override
   public void compute(double time)
   {
      currentTime.set(time);

      if (isDone())
      {
         currentOrientation.set(finalOrientation);
         currentAngularVelocity.set(finalAngularVelocity);
         currentAngularVelocityFD.set(finalAngularVelocity);
         currentAngularAcceleration.setToZero();
         currentAngularAccelerationFD.setToZero();
         return;
      }
      else if (currentTime.getDoubleValue() <= 0.0)
      {
         currentOrientation.set(initialOrientation);
         currentAngularVelocity.set(initialAngularVelocity);
         currentAngularVelocityFD.set(initialAngularVelocity);
         currentAngularAccelerationFD.setToZero();
         return;
      }

      time = MathTools.clamp(time, 0.0, trajectoryTime.getDoubleValue());
      double timePrevious = time - dtForFiniteDifference;
      double timeNext = time + dtForFiniteDifference;

      computeBezierBasedCurve(timePrevious, qInterpolatedPrevious, angularVelocityInterpolatedPrevious, angularAccelerationInterpolated);
      computeBezierBasedCurve(timeNext, qInterpolatedNext, angularVelocityInterpolatedNext, angularAccelerationInterpolated);
      computeBezierBasedCurve(time, qInterpolated, angularVelocityInterpolated, angularAccelerationInterpolated);

      quaternionCalculus.computeQDotByFiniteDifferenceCentral(qInterpolatedPrevious, qInterpolatedNext, dtForFiniteDifference, qDot);
      //      // This method of calculating qDDot will occasionally result in jerky behavior due to very small local minima/maxima in the quaternions
      quaternionCalculus.computeQDDotByFiniteDifferenceCentral(qInterpolatedPrevious, qInterpolated, qInterpolatedNext, dtForFiniteDifference, qDDot);

      quaternionCalculus.computeAngularVelocityInWorldFrame(qInterpolated, qDot, tempAngularVelocity);
      quaternionCalculus.computeAngularAcceleration(qInterpolated, qDot, qDDot, tempAngularAcceleration);

      tempAngularAcceleration.sub(angularVelocityInterpolated, angularVelocityInterpolatedPrevious);
      tempAngularAcceleration.scale(1.0 / dtForFiniteDifference);

      currentOrientation.set(qInterpolated);
      currentAngularVelocity.set(angularVelocityInterpolated);
      currentAngularVelocityFD.set(tempAngularVelocity);
      currentAngularAcceleration.set(angularAccelerationInterpolated);
      currentAngularAccelerationFD.set(tempAngularAcceleration);
   }

   private final Quaternion qStar = new Quaternion();
   private final Vector4D qStarDot = new Vector4D();
   private final Vector4D qStarDDot = new Vector4D();

   private final Vector3D w1 = new Vector3D();
   private final Vector3D w2 = new Vector3D();
   private final Vector3D w3 = new Vector3D();

   private final Quaternion expW1B1 = new Quaternion();
   private final Quaternion expW2B2 = new Quaternion();
   private final Quaternion expW1B1_expW2B2 = new Quaternion();
   private final Quaternion expW3B3 = new Quaternion();

   private final Vector4D w1B1Dot = new Vector4D();
   private final Vector4D w2B2Dot = new Vector4D();
   private final Vector4D w3B3Dot = new Vector4D();

   private final Vector4D w1B1DDot = new Vector4D();
   private final Vector4D w2B2DDot = new Vector4D();
   private final Vector4D w3B3DDot = new Vector4D();

   private final Vector4D qDot1 = new Vector4D();
   private final Vector4D qDot2 = new Vector4D();
   private final Vector4D qDot3 = new Vector4D();

   private final Vector4D qDDotTemp = new Vector4D();
   private final Vector4D qDDot1 = new Vector4D();
   private final Vector4D qDDot2 = new Vector4D();
   private final Vector4D qDDot3 = new Vector4D();

   private void updateBezierCoefficients(double t)
   {
      double tSquare = t * t;
      double tCube = tSquare * t;

      double oneMinusT = 1.0 - t;
      double oneMinusTSquare = oneMinusT * oneMinusT;
      double oneMinusTimeCube = oneMinusTSquare * oneMinusT;

      cumulativeBeziers[1].set(1.0 - oneMinusTimeCube);
      cumulativeBeziers[2].set(3.0 * tSquare - 2.0 * tCube);
      cumulativeBeziers[3].set(tCube);

      cumulativeBeziersDot[1].set(3.0 * oneMinusTSquare);
      cumulativeBeziersDot[2].set(6.0 * t * oneMinusT);
      cumulativeBeziersDot[3].set(3.0 * tSquare);

      cumulativeBeziersDDot[1].set(-6.0 * oneMinusT);
      cumulativeBeziersDDot[2].set(6.0 * (1.0 - 2.0 * t));
      cumulativeBeziersDDot[3].set(6.0 * t);
   }

   private void computeBezierBasedCurve(double time, QuaternionBasics q, Vector3D angularVelocity, Vector3D angularAcceleration)
   {
      updateBezierCoefficients(time);

      // Changing naming convention to make expressions smaller
      Quaternion q0 = controlQuaternions[0];
      yoControlQuaternions[0].get(q0);
      controlAngularVelocities[1].get(w1);
      controlAngularVelocities[2].get(w2);
      controlAngularVelocities[3].get(w3);

      // Update intermediate variables
      expW1B1.set(exp(cumulativeBeziers[1].getDoubleValue(), w1));
      expW2B2.set(exp(cumulativeBeziers[2].getDoubleValue(), w2));
      expW3B3.set(exp(cumulativeBeziers[3].getDoubleValue(), w3));
      expW1B1_expW2B2.set(expW1B1);
      expW1B1_expW2B2.multiply(expW2B2);

      // In page 1, the authors say they use a specific type of quaternion for which
      // exp(theta * u) = (cos(theta), u * sin(theta)) instead of:
      // exp(theta * u) = (cos(theta/2), u * sin(theta/2)).
      // Because of the latter, the quaternion derivatives in the paper are actually wrong.
      // When derivating the exponent of an exponential term, it should be as follows:
      // d/dt(exp(alpha(t) * u)) = 0.5 * alphaDot(t) * u * exp(alpha(t) * u)
      w1B1Dot.set(w1);
      w2B2Dot.set(w2);
      w3B3Dot.set(w3);
      w1B1Dot.scale(0.5 * cumulativeBeziersDot[1].getDoubleValue());
      w2B2Dot.scale(0.5 * cumulativeBeziersDot[2].getDoubleValue());
      w3B3Dot.scale(0.5 * cumulativeBeziersDot[3].getDoubleValue());

      w1B1DDot.set(w1);
      w2B2DDot.set(w2);
      w3B3DDot.set(w3);
      w1B1DDot.scale(0.5 * cumulativeBeziersDDot[1].getDoubleValue());
      w2B2DDot.scale(0.5 * cumulativeBeziersDDot[2].getDoubleValue());
      w3B3DDot.scale(0.5 * cumulativeBeziersDDot[3].getDoubleValue());

      // Calculate qStar = exp(w1*B1) * exp(w2*B2) * exp(w3*B3)
      qStar.set(expW1B1);
      qStar.multiply(expW2B2);
      qStar.multiply(expW3B3);

      // Calculate qStarDot = qDot1 + qDot2 + qDot3, with:
      // qDot1 = ((w1*BDot1) * qStar
      QuaternionTools.multiply(w1B1Dot, qStar, qDot1);
      // qDot2 = exp(w1*B1) * exp(w2*B2) * (w2*BDot2) * exp(w3*B3)
      QuaternionTools.multiply(expW1B1_expW2B2, w2B2Dot, qDot2);
      QuaternionTools.multiply(qDot2, expW3B3, qDot2);
      // qDot3 = qStar * (w3*BDot3)
      QuaternionTools.multiply(qStar, w3B3Dot, qDot3);
      // Now qStarDot
      qStarDot.add(qDot1, qDot2);
      qStarDot.add(qDot3);

      // Calculate qStarDDot:
      // qStarDDot = qDDot1 + qDDot2 + qDDot3
      // qDDot1 = (w1*BDDot1) * qStar + (w1*BDot1) * qStarDot
      QuaternionTools.multiply(w1B1DDot, qStar, qDDotTemp);
      QuaternionTools.multiply(w1B1Dot, qStarDot, qDDot1);
      qDDot1.add(qDDotTemp);

      // qDDot2 = exp(w1*B1)*exp(w2*B2)*{ (w2*BDDot2) + (w2*BDot2)*(w2*BDot2) }*exp(w3*B3) + (w1*BDot1)*qDot2 + qDot2*(w3*BDot3)
      QuaternionTools.multiply(w2B2Dot, w2B2Dot, qDDot2);
      qDDot2.add(w2B2DDot);
      QuaternionTools.multiply(expW1B1_expW2B2, qDDot2, qDDotTemp);
      QuaternionTools.multiply(qDDotTemp, expW3B3, qDDotTemp);
      qDDot2.set(qDDotTemp);
      QuaternionTools.multiply(w1B1Dot, qDot2, qDDotTemp);
      qDDot2.add(qDDotTemp);
      QuaternionTools.multiply(qDot2, w3B3Dot, qDDotTemp);
      qDDot2.add(qDDotTemp);

      // qDDot3 = qStar * (w3*BDDot3) + qStarDot * (w3*BDot3)
      QuaternionTools.multiply(qStar, w3B3DDot, qDDotTemp);
      QuaternionTools.multiply(qStarDot, w3B3Dot, qDDot3);
      qDDot3.add(qDDotTemp);

      // Now qStarDDot
      qStarDDot.add(qDDot1, qDDot2);
      qStarDDot.add(qDDot3);

      q.multiply(q0, qStar);
      QuaternionTools.multiply(q0, qStarDot, qDot);
      QuaternionTools.multiply(q0, qStarDDot, qDDot);

      angularVelocity.set(convertToAngularVelocity(q, qDot));
      angularAcceleration.set(convertToAngularAcceleration(q, qDot, qDDot));
   }

   private final Quaternion tempLogExpQuaternion = new Quaternion();
   private final Vector3D tempLogExpVector3D = new Vector3D();

   private QuaternionReadOnly exp(double alpha, Vector3DReadOnly rotation)
   {
      tempLogExpVector3D.setAndScale(alpha, rotation);
      tempLogExpQuaternion.set(tempLogExpVector3D);
      return tempLogExpQuaternion;
   }

   private Vector3DReadOnly logOfDifference(QuaternionReadOnly q0, QuaternionReadOnly q1)
   {
      tempLogExpQuaternion.difference(q0, q1);
      tempLogExpQuaternion.get(tempLogExpVector3D);
      return tempLogExpVector3D;
   }

   private final Vector3D tempConvertVector3D = new Vector3D();
   private final Vector4D tempConvertVector4D = new Vector4D();

   private Vector3DReadOnly convertToAngularVelocity(QuaternionReadOnly q, Vector4DReadOnly qDot)
   {// w = qDot * q^-1
      QuaternionTools.multiplyConjugateRight(qDot, q, tempConvertVector4D);
      tempConvertVector3D.setX(tempConvertVector4D.getX());
      tempConvertVector3D.setY(tempConvertVector4D.getY());
      tempConvertVector3D.setZ(tempConvertVector4D.getZ());
      tempConvertVector3D.scale(2.0);
      return tempConvertVector3D;
   }

   private Vector3DReadOnly convertToAngularAcceleration(QuaternionReadOnly q, Vector4DReadOnly qDot, Vector4DReadOnly qDDot)
   { // w = qDDot * q^-1 + qDot * qDot^-1
      QuaternionTools.multiplyConjugateRight(qDDot, q, tempConvertVector4D);
      tempConvertVector3D.setX(tempConvertVector4D.getX());
      tempConvertVector3D.setY(tempConvertVector4D.getY());
      tempConvertVector3D.setZ(tempConvertVector4D.getZ());

      QuaternionTools.multiplyConjugateRight(qDot, qDot, tempConvertVector4D);
      tempConvertVector3D.addX(tempConvertVector4D.getX());
      tempConvertVector3D.addY(tempConvertVector4D.getY());
      tempConvertVector3D.addZ(tempConvertVector4D.getZ());
      tempConvertVector3D.scale(2.0);
      return tempConvertVector3D;
   }

   public boolean isSolvable(double trajectoryTime, Vector3D omegaA, Vector3D omegaB)
   {
      omegaA.scale(trajectoryTime / 3.0);
      boolean omegaALength = omegaA.length() < 2.0 * Math.PI;

      omegaB.scale(trajectoryTime / 3.0);
      boolean omegaBLength = (omegaB.length() < 2.0 * Math.PI);

      omegaB.negate();
      boolean opposite = omegaA.epsilonEquals(omegaB, 1e-2);

      return omegaALength && omegaBLength && !opposite;
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
