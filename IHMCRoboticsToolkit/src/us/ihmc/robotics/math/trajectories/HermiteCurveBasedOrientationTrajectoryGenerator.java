package us.ihmc.robotics.math.trajectories;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.Vector4D;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
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

/**
 * This trajectory generator aims at interpolating between two orientations q0 and qf for given angular velocities at the limits w0 and wf.
 * The method used here differs from the trajectory generator implemented in {@link VelocityConstrainedOrientationTrajectoryGenerator}.
 * It seems that the approach used here is a better fit for interpolating between waypoints.
 *
 * I basically implemented the method called Hermite Quaternion Curve that is presented in the following paper:
 * <p> <a href="http://azrael.digipen.edu/MAT351/papers/Kim2.pdf"> Paper on quaternion interpolation (PDF link 1)</a> </p>
 * <p> <a href="http://graphics.cs.cmu.edu/nsp/course/15-464/Fall05/papers/kimKimShin.pdf"> Paper on quaternion interpolation (PDF link 2)</a> </p>
 * <p> <a href="https://www.researchgate.net/publication/2388093_A_General_Construction_Scheme_for_Unit_Quaternion_Curves_with_Simple_High_Order_Derivatives"> Paper on quaternion interpolation (ResearchGate link)</a> </p>
 *
 * Watch out for the typo in equation 9 when reading the paper though. The beta on the right hand side of the equation should have for subscripts {j,n} instead of {i,n}.
 * Also the statement that arbitrary high velocities can be assigned is false. Both endpoint velocities after scaling with the trajectory time have a maximum magnitude of 6 pi.
 *
 * @author Sylvain
 */
public class HermiteCurveBasedOrientationTrajectoryGenerator extends OrientationTrajectoryGeneratorInMultipleFrames
{
   private final YoVariableRegistry registry;
   private final DoubleYoVariable currentTime;
   private final DoubleYoVariable trajectoryTime;
   private final DoubleYoVariable trajectoryTimeScale;
   private final IntegerYoVariable piInteger;

   private final DoubleYoVariable[] cumulativeBeziers;

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
   private final double dtForFiniteDifference = 1.0e-4;

   public HermiteCurveBasedOrientationTrajectoryGenerator(String name, ReferenceFrame referenceFrame, YoVariableRegistry parentRegistry)
   {
      this(name, false, referenceFrame, parentRegistry);
   }

   public HermiteCurveBasedOrientationTrajectoryGenerator(String name, boolean allowMultipleFrames, ReferenceFrame referenceFrame,
         YoVariableRegistry parentRegistry)
   {
      super(allowMultipleFrames, referenceFrame);

      registry = new YoVariableRegistry(name);
      trajectoryTime = new DoubleYoVariable(name + "TrajectoryTime", registry);
      trajectoryTimeScale = new DoubleYoVariable(name + "TrajectoryTimeScale", registry);
      piInteger = new IntegerYoVariable(name + "PiInteger", registry);
      currentTime = new DoubleYoVariable(name + "Time", registry);
      trajectoryFrame = referenceFrame;

      cumulativeBeziers = new DoubleYoVariable[4];
      controlQuaternions = new YoFrameQuaternion[4];
      controlAngularVelocities = new YoFrameVector[4];

      for (int i = 1; i <= 3; i++)
      {
         cumulativeBeziers[i] = new DoubleYoVariable(name + "CumulativeBezier" + i, registry);
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

   /**
    * Sets an Integer n to add n/2 full revolutions to the trajectory.
    *
    */
   public void setPiInteger(int piInteger)
   {
      this.piInteger.set(piInteger);
      updateControlQuaternions();
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
    * Before Initializing the isSolvable() method should be called to confirm that the limit conditions are within the bounds
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
         quaternionCalculus.inverseMultiply(tempControlQuaternions[i - 1], tempControlQuaternions[i], tempQuatForControlQuats);
         quaternionCalculus.log(tempQuatForControlQuats, tempAngularVelocity);
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

   private final Vector4D qDot = new Vector4D();
   private final Vector4D qDDotOld = new Vector4D();
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
      else if (currentTime.getDoubleValue() <= 0.0)
      {
         currentOrientation.set(initialOrientation);
         currentAngularVelocity.set(initialAngularVelocity);
         currentAngularAcceleration.setToZero();
         return;
      }

      time = MathTools.clamp(time, 0.0, trajectoryTime.getDoubleValue());
      double timePrevious = time - dtForFiniteDifference;
      double timeNext = time + dtForFiniteDifference;

      interpolateOrientation(timePrevious, qInterpolatedPrevious);
      interpolateOrientation(timeNext, qInterpolatedNext);
      interpolateOrientation(time, qInterpolated);

      quaternionCalculus.computeQDotByFiniteDifferenceCentral(qInterpolatedPrevious, qInterpolatedNext, dtForFiniteDifference, qDot);

      // This method of calculating qDDot will occasionally result in jerky behavior due to very small local minima/maxima in the quaternions
      quaternionCalculus.computeQDDotByFiniteDifferenceCentral(qInterpolatedPrevious, qInterpolated, qInterpolatedNext, dtForFiniteDifference, qDDot);

      quaternionCalculus.computeAngularVelocityInWorldFrame(qInterpolated, qDot, tempAngularVelocity);
      quaternionCalculus.computeAngularAcceleration(qInterpolated, qDot, qDDot, tempAngularAcceleration);

      currentOrientation.set(qInterpolated);
      currentAngularVelocity.set(tempAngularVelocity);
      currentAngularAcceleration.set(tempAngularAcceleration);

      qDDotOld.set(qDDot);
   }

   private final Quaternion tempQuatForInterpolation = new Quaternion();

   private void interpolateOrientation(double time, Quaternion qInterpolated)
   {
      controlQuaternions[0].get(qInterpolated);

      updateBezierCoefficients(time);

      for (int i = 1; i <= 3; i++)
      {
         computeBezierQuaternionCurveTerm(i, tempQuatForInterpolation);
         qInterpolated.multiply(tempQuatForInterpolation);
      }
   }

   /**
    * I use the developed equations to compute the Bezier basis functions. It should be faster.
    *
    */
   private void updateBezierCoefficients(double time)
   {
      time *= trajectoryTimeScale.getDoubleValue();
      double timeSquare = time * time;
      double timeCube = timeSquare * time;

      cumulativeBeziers[1].set(1 - MathTools.powWithInteger(1 - time, 3));
      cumulativeBeziers[2].set(3.0 * timeSquare - 2 * timeCube);
      cumulativeBeziers[3].set(timeCube);
   }

   private void computeBezierQuaternionCurveTerm(int i, Quaternion resultToPack)
   {
      double cumulativeBernsteinCoefficient = cumulativeBeziers[i].getDoubleValue();
      controlAngularVelocities[i].get(tempAngularVelocity);
      tempAngularVelocity.scale(cumulativeBernsteinCoefficient);
      quaternionCalculus.exp(tempAngularVelocity, resultToPack);
   }

   public boolean isSolvable(double trajectoryTime, Vector3D omegaA, Vector3D omegaB)
   {
      omegaA.scale(trajectoryTime / 3);
      boolean omegaALength = omegaA.length() < 2 * Math.PI;

      omegaB.scale(trajectoryTime / 3);
      boolean omegaBLength = (omegaB.length() < 2 * Math.PI);

      omegaB.scale(-1);
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
