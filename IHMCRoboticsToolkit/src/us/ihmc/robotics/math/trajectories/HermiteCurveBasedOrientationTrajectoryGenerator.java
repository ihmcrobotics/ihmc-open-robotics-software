package us.ihmc.robotics.math.trajectories;

import us.ihmc.euclid.tools.QuaternionTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.Vector4D;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
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

   private final YoFrameQuaternion[] controlQuaternions;
   private final YoFrameVector[] controlAngularVelocities;

   private final YoFrameQuaternion initialOrientation;
   private final YoFrameVector initialAngularVelocity;
   private final YoFrameQuaternion finalOrientation;
   private final YoFrameVector finalAngularVelocity;

   private final YoFrameQuaternion currentOrientation;
   private final YoFrameVector currentAngularVelocity;
   private final YoFrameVector currentAngularAcceleration;

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
      controlQuaternions = new YoFrameQuaternion[4];
      controlAngularVelocities = new YoFrameVector[4];

      for (int i = 1; i <= 3; i++)
      {
         cumulativeBeziers[i] = new YoDouble(name + "CumulativeBezier" + i, registry);
         cumulativeBeziersDot[i] = new YoDouble(name + "CumulativeBezierDot" + i, registry);
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
         YoFrameVectorInMultipleFrames currentAngularAcceleration = new YoFrameVectorInMultipleFrames(name + currentAngularAccelerationName, registry,
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
         this.currentAngularAcceleration = currentAngularAcceleration;

         for (int i = 0; i <= 3; i++)
         {
            YoFrameQuaternionInMultipleFrames controlQuaternion = new YoFrameQuaternionInMultipleFrames(name + controlQuaternionName + i, registry,
                                                                                                        trajectoryFrame);
            registerMultipleFramesHolders(controlQuaternion);
            controlQuaternions[i] = controlQuaternion;
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
         currentAngularAcceleration = new YoFrameVector(name + currentAngularAccelerationName, trajectoryFrame, registry);

         for (int i = 0; i <= 3; i++)
            controlQuaternions[i] = new YoFrameQuaternion(name + controlQuaternionName + i, trajectoryFrame, registry);

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
      currentAngularAcceleration.setToZero();
   }

   private final Quaternion[] tempControlQuaternions = new Quaternion[] {new Quaternion(), new Quaternion(), new Quaternion(), new Quaternion()};
   private final Quaternion tempQuatForControlQuats = new Quaternion();

   private void updateControlQuaternions()
   {
      initialOrientation.get(tempControlQuaternions[0]);
      finalOrientation.get(tempControlQuaternions[3]);

      initialOrientation.get(tempControlQuaternions[1]);
      initialAngularVelocity.get(tempAngularVelocity);

      tempControlQuaternions[0].inverseTransform(tempAngularVelocity);
      tempAngularVelocity.scale(trajectoryTime.getDoubleValue());
      tempAngularVelocity.scale(1 / 3.0);
      quaternionCalculus.exp(tempAngularVelocity, tempQuatForControlQuats);

      tempControlQuaternions[1].multiply(tempQuatForControlQuats);

      finalOrientation.get(tempControlQuaternions[2]);
      finalAngularVelocity.get(tempAngularVelocity);

      tempControlQuaternions[3].inverseTransform(tempAngularVelocity);
      tempAngularVelocity.scale(trajectoryTime.getDoubleValue());
      tempAngularVelocity.scale(1 / 3.0);
      tempAngularVelocity.negate();
      quaternionCalculus.exp(tempAngularVelocity, tempQuatForControlQuats);
      tempControlQuaternions[2].multiply(tempQuatForControlQuats);

      for (int i = 1; i <= 3; i++)
      {
         tempQuatForControlQuats.difference(tempControlQuaternions[i - 1], tempControlQuaternions[i]);
         tempQuatForControlQuats.get(tempAngularVelocity);
         controlAngularVelocities[i].set(tempAngularVelocity);
      }

      controlAngularVelocities[2].get(tempAngularVelocity);
      if (tempAngularVelocity.lengthSquared() > 1.0e-10)
      {
         tempAngularVelocity.normalize();
         tempAngularVelocity.scale(piInteger.getIntegerValue() * Math.PI);
         controlAngularVelocities[2].add(tempAngularVelocity);
      }

      for (int i = 0; i <= 3; i++)
      {
         controlQuaternions[i].set(tempControlQuaternions[i]);
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

   private final Vector4D qDot = new Vector4D();
   private final Vector4D qDDot = new Vector4D();

   @Override
   public void compute(double time)
   {
      currentTime.set(time);
      // The Hermite trajectory is defined with a time in [0, 1]
      time = time / trajectoryTime.getDoubleValue();

      if (isDone())
      {
         currentOrientation.set(finalOrientation);
         currentAngularVelocity.set(finalAngularVelocity);
         currentAngularAcceleration.setToZero();
         return;
      }
      else if (currentTime.getDoubleValue() <= 0.0)
      {
         currentOrientation.set(initialOrientation);
         currentAngularVelocity.set(initialAngularVelocity);
         currentAngularAcceleration.setToZero();
         return;
      }

      time = MathTools.clamp(time, 0.0, 1.0);
      double timePrevious = time - dtForFiniteDifference;
      double timeNext = time + dtForFiniteDifference;

      interpolateOrientation(timePrevious, qInterpolatedPrevious, angularVelocityInterpolatedPrevious);
      interpolateOrientation(timeNext, qInterpolatedNext, angularVelocityInterpolatedNext);
      interpolateOrientation(time, qInterpolated, angularVelocityInterpolated);

      quaternionCalculus.computeQDotByFiniteDifferenceCentral(qInterpolatedPrevious, qInterpolatedNext, dtForFiniteDifference, qDot);

      // This method of calculating qDDot will occasionally result in jerky behavior due to very small local minima/maxima in the quaternions
      quaternionCalculus.computeQDDotByFiniteDifferenceCentral(qInterpolatedPrevious, qInterpolated, qInterpolatedNext, dtForFiniteDifference, qDDot);

      quaternionCalculus.computeAngularAcceleration(qInterpolated, qDot, qDDot, tempAngularAcceleration);

      // Recomputing the velocity and acceleration given the actual trajectory time
      angularVelocityInterpolated.scale(1.0 / (trajectoryTime.getDoubleValue()));
      tempAngularAcceleration.scale(1.0 / (trajectoryTime.getDoubleValue() * trajectoryTime.getDoubleValue()));

      currentOrientation.set(qInterpolated);
      currentAngularVelocity.set(angularVelocityInterpolated);
      currentAngularAcceleration.set(tempAngularAcceleration);
   }

   private final Quaternion tempQuatForInterpolation = new Quaternion();

   private void interpolateOrientation(double time, Quaternion qInterpolated, Vector3D angularVelocity)
   {
      controlQuaternions[0].get(qInterpolated);

      updateBezierCoefficients(time);

      for (int i = 1; i <= 3; i++)
      {
         computeBezierQuaternionCurveTerm(i, tempQuatForInterpolation);
         qInterpolated.multiply(tempQuatForInterpolation);
      }

      computeBezierQuaternionCurveDerivative(qInterpolated, angularVelocity);
   }

   /**
    * I use the developed equations to compute the Bezier basis functions. It should be faster.
    *
    */
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
   }

   private void computeBezierQuaternionCurveTerm(int i, Quaternion resultToPack)
   {
      double cumulativeBernsteinCoefficient = cumulativeBeziers[i].getDoubleValue();

      controlAngularVelocities[i].get(tempAngularVelocity);
      tempAngularVelocity.scale(cumulativeBernsteinCoefficient);

      resultToPack.set(tempAngularVelocity);
   }

   private final Quaternion q0 = new Quaternion();

   private final Vector3D w1 = new Vector3D();
   private final Vector3D w2 = new Vector3D();
   private final Vector3D w3 = new Vector3D();
   
   private final Quaternion expW1B1 = new Quaternion();
   private final Quaternion expW2B2 = new Quaternion();
   private final Quaternion expW3B3 = new Quaternion();

   private final Vector4D w1B1Dot = new Vector4D();
   private final Vector4D w2B2Dot = new Vector4D();
   private final Vector4D w3B3Dot = new Vector4D();

   private final Vector3D rotationVector = new Vector3D();

   private final Vector4D qDot1 = new Vector4D();
   private final Vector4D qDot2 = new Vector4D();
   private final Vector4D qDot3 = new Vector4D();
   private final Vector4D angularVelocity4D = new Vector4D();

   private void computeBezierQuaternionCurveDerivative(QuaternionReadOnly qInterpolated, Vector3D angularVelocity)
   {
      controlQuaternions[0].get(q0);

      controlAngularVelocities[1].get(w1);
      controlAngularVelocities[2].get(w2);
      controlAngularVelocities[3].get(w3);

      w1B1Dot.set(w1);
      w2B2Dot.set(w2);
      w3B3Dot.set(w3);
      w1B1Dot.scale(cumulativeBeziersDot[1].getDoubleValue());
      w2B2Dot.scale(cumulativeBeziersDot[2].getDoubleValue());
      w3B3Dot.scale(cumulativeBeziersDot[3].getDoubleValue());

      rotationVector.setAndScale(cumulativeBeziers[1].getDoubleValue(), w1);
      expW1B1.set(rotationVector);
      rotationVector.setAndScale(cumulativeBeziers[2].getDoubleValue(), w2);
      expW2B2.set(rotationVector);
      rotationVector.setAndScale(cumulativeBeziers[3].getDoubleValue(), w3);
      expW3B3.set(rotationVector);

      QuaternionTools.multiply(q0, expW1B1, qDot1);
      QuaternionTools.multiply(qDot1, w1B1Dot, qDot1);
      QuaternionTools.multiply(qDot1, expW2B2, qDot1);
      QuaternionTools.multiply(qDot1, expW3B3, qDot1);
      
      QuaternionTools.multiply(q0, expW1B1, qDot2);
      QuaternionTools.multiply(qDot2, expW2B2, qDot2);
      QuaternionTools.multiply(qDot2, w2B2Dot, qDot2);
      QuaternionTools.multiply(qDot2, expW3B3, qDot2);
      
      QuaternionTools.multiply(q0, expW1B1, qDot3);
      QuaternionTools.multiply(qDot3, expW2B2, qDot3);
      QuaternionTools.multiply(qDot3, expW3B3, qDot3);
      QuaternionTools.multiply(qDot3, w3B3Dot, qDot3);

      qDot.add(qDot1, qDot2);
      qDot.add(qDot3);

      QuaternionTools.multiplyConjugateRight(qDot, qInterpolated, angularVelocity4D);
      angularVelocity.setX(angularVelocity4D.getX());
      angularVelocity.setY(angularVelocity4D.getY());
      angularVelocity.setZ(angularVelocity4D.getZ());
      
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
