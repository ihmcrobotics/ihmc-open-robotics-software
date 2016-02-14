package us.ihmc.robotics.math.trajectories;

import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
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
 * The method used here differs from the trajectory generator implemented in {@link VelocityConstrainedOrientationTrajectoryGenerator}.
 * It seems that the approach used here is a better fit for interpolating between waypoints.
 * 
 * I basically implemented the method called Hermite Quaternion Curve that is presented in the following paper:
 * <p> <a href="http://azrael.digipen.edu/MAT351/papers/Kim2.pdf"> Paper on quaternion interpolation (PDF link 1)</a> </p>
 * <p> <a href="http://graphics.cs.cmu.edu/nsp/course/15-464/Fall05/papers/kimKimShin.pdf"> Paper on quaternion interpolation (PDF link 2)</a> </p>
 * <p> <a href="https://www.researchgate.net/publication/2388093_A_General_Construction_Scheme_for_Unit_Quaternion_Curves_with_Simple_High_Order_Derivatives"> Paper on quaternion interpolation (ResearchGate link)</a> </p>
 * 
 * Watch out for the typo in equation 9 when reading the paper though. The beta on the right hand side of the equation should have for subscripts {j,n} instead of {i,n}.
 * 
 * @author Sylvain 
 */
public class HermiteCurveBasedOrientationTrajectoryGenerator extends OrientationTrajectoryGeneratorInMultipleFrames
{
   private final YoVariableRegistry registry;
   private final DoubleYoVariable currentTime;
   private final DoubleYoVariable trajectoryTime;
   private final DoubleYoVariable trajectoryTimeScale;

   private final DoubleYoVariable[] beziers;
   private final DoubleYoVariable[] bezierDerivatives;
   private final DoubleYoVariable[] cumulativeBeziers;
   private final DoubleYoVariable[] cumulativeBezierDerivatives;

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
   private final double dtForFiniteDifference = 1.0e-3; // Weird jerkiness at 1.0e-4

   public HermiteCurveBasedOrientationTrajectoryGenerator(String namePrefix, ReferenceFrame referenceFrame, YoVariableRegistry parentRegistry)
   {
      this(namePrefix, false, referenceFrame, parentRegistry);
   }

   public HermiteCurveBasedOrientationTrajectoryGenerator(String namePrefix, boolean allowMultipleFrames, ReferenceFrame referenceFrame, YoVariableRegistry parentRegistry)
   {
      super(allowMultipleFrames, referenceFrame);

      registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());
      trajectoryTime = new DoubleYoVariable(namePrefix + "TrajectoryTime", registry);
      trajectoryTimeScale = new DoubleYoVariable(namePrefix + "TrajectoryTimeScale", registry);
      currentTime = new DoubleYoVariable(namePrefix + "Time", registry);
      trajectoryFrame = referenceFrame;

      beziers = new DoubleYoVariable[4];
      bezierDerivatives = new DoubleYoVariable[4];
      cumulativeBeziers = new DoubleYoVariable[4];
      cumulativeBezierDerivatives = new DoubleYoVariable[4];
      controlQuaternions = new YoFrameQuaternion[4];
      controlAngularVelocities = new YoFrameVector[4];

      for (int i = 1; i <= 3; i++)
      {
         beziers[i] = new DoubleYoVariable(namePrefix + "Bezier" + i, registry);
         bezierDerivatives[i] = new DoubleYoVariable(namePrefix + "BezierDerivative" + i, registry);
         cumulativeBeziers[i] = new DoubleYoVariable(namePrefix + "CumulativeBezier" + i, registry);
         cumulativeBezierDerivatives[i] = new DoubleYoVariable(namePrefix + "CumulativeBezierDerivative" + i, registry);
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
         YoFrameQuaternionInMultipleFrames initialOrientation = new YoFrameQuaternionInMultipleFrames(namePrefix + initialOrientationName, registry, trajectoryFrame);
         YoFrameVectorInMultipleFrames initialAngularVelocity = new YoFrameVectorInMultipleFrames(namePrefix + initialAngularVelocityName, registry, trajectoryFrame);
         YoFrameQuaternionInMultipleFrames finalOrientation = new YoFrameQuaternionInMultipleFrames(namePrefix + finalOrientationName, registry, trajectoryFrame);
         YoFrameVectorInMultipleFrames finalAngularVelocity = new YoFrameVectorInMultipleFrames(namePrefix + finalAngularVelocityName, registry, trajectoryFrame);

         YoFrameQuaternionInMultipleFrames currentOrientation = new YoFrameQuaternionInMultipleFrames(namePrefix + currentOrientationName, registry, trajectoryFrame);
         YoFrameVectorInMultipleFrames currentAngularVelocity = new YoFrameVectorInMultipleFrames(namePrefix + currentAngularVelocityName, registry, trajectoryFrame);
         YoFrameVectorInMultipleFrames currentAngularAcceleration = new YoFrameVectorInMultipleFrames(namePrefix + currentAngularAccelerationName, registry, trajectoryFrame);

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

         for (int i = 0; i <=3; i++)
         {
            YoFrameQuaternionInMultipleFrames controlQuaternion = new YoFrameQuaternionInMultipleFrames(namePrefix + controlQuaternionName + i, registry, trajectoryFrame);
            registerMultipleFramesHolders(controlQuaternion);
            controlQuaternions[i] = controlQuaternion;
         }

         for (int i = 1; i <= 3; i++)
         {
            YoFrameVectorInMultipleFrames controlAngularVelocity = new YoFrameVectorInMultipleFrames(namePrefix + controlAngularVelocityName + i, registry, trajectoryFrame);
            registerMultipleFramesHolders(controlAngularVelocity);
            controlAngularVelocities[i] = controlAngularVelocity;
         }
      }
      else
      {
         initialOrientation = new YoFrameQuaternion(namePrefix + initialOrientationName, trajectoryFrame, registry);
         initialAngularVelocity = new YoFrameVector(namePrefix + initialAngularVelocityName, trajectoryFrame, registry);
         finalOrientation = new YoFrameQuaternion(namePrefix + finalOrientationName, trajectoryFrame, registry);
         finalAngularVelocity = new YoFrameVector(namePrefix + finalAngularVelocityName, trajectoryFrame, registry);

         currentOrientation = new YoFrameQuaternion(namePrefix + currentOrientationName, trajectoryFrame, registry);
         currentAngularVelocity = new YoFrameVector(namePrefix + currentAngularVelocityName, trajectoryFrame, registry);
         currentAngularAcceleration = new YoFrameVector(namePrefix + currentAngularAccelerationName, trajectoryFrame, registry);

         for (int i = 0; i <=3; i++)
            controlQuaternions[i] = new YoFrameQuaternion(namePrefix + controlQuaternionName + i, trajectoryFrame, registry);

         for (int i = 1; i <= 3; i++)
            controlAngularVelocities[i] = new YoFrameVector(namePrefix + controlAngularVelocityName + i, trajectoryFrame, registry);
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

   public void setTrajectoryParameters(FrameSO3Waypoint initialFrameSO3Waypoint, FrameSO3Waypoint finalFrameSO3Waypoint)
   {
      setTrajectoryTime(finalFrameSO3Waypoint.getTime() - initialFrameSO3Waypoint.getTime());

      initialOrientation.set(initialFrameSO3Waypoint.getOrientation());
      initialAngularVelocity.set(initialFrameSO3Waypoint.getAngularVelocity());

      finalOrientation.set(finalFrameSO3Waypoint.getOrientation());
      finalAngularVelocity.set(finalFrameSO3Waypoint.getAngularVelocity());
   }

   public void setTrajectoryParameters(YoFrameSO3Waypoint initialYoFrameSO3Waypoint, YoFrameSO3Waypoint finalYoFrameSO3Waypoint)
   {
      setTrajectoryTime(finalYoFrameSO3Waypoint.getTime() - initialYoFrameSO3Waypoint.getTime());

      initialOrientation.set(initialYoFrameSO3Waypoint.getOrientation());
      initialAngularVelocity.set(initialYoFrameSO3Waypoint.getAngularVelocity());

      finalOrientation.set(finalYoFrameSO3Waypoint.getOrientation());
      finalAngularVelocity.set(finalYoFrameSO3Waypoint.getAngularVelocity());
   }

   @Override
   public void initialize()
   {
      currentTime.set(0.0);
      trajectoryTimeScale.set(1.0 / trajectoryTime.getDoubleValue());

      if (initialOrientation.dot(finalOrientation) < 0.0)
         finalOrientation.negate();

      updateControlQuaternions();      

      currentOrientation.set(initialOrientation);
      currentAngularVelocity.set(initialAngularVelocity);
      currentAngularAcceleration.setToZero();
   }

   private final Quat4d[] tempControlQuaternions = new Quat4d[]{new Quat4d(), new Quat4d(), new Quat4d(), new Quat4d()};
   private final Quat4d tempQuatForControlQuats = new Quat4d();

   private void updateControlQuaternions()
   {
      initialOrientation.get(tempControlQuaternions[0]);
      finalOrientation.get(tempControlQuaternions[3]);

      initialOrientation.get(tempControlQuaternions[1]);
      initialAngularVelocity.get(tempAngularVelocity);
      quaternionCalculus.invertTransform(tempControlQuaternions[0], tempAngularVelocity);
      tempAngularVelocity.scale(1.0 / 3.0 / trajectoryTimeScale.getDoubleValue());
      RotationTools.convertRotationVectorToQuaternion(tempAngularVelocity, tempQuatForControlQuats);
      tempControlQuaternions[1].mul(tempQuatForControlQuats);

      finalOrientation.get(tempControlQuaternions[2]);
      finalAngularVelocity.get(tempAngularVelocity);
      quaternionCalculus.invertTransform(tempControlQuaternions[3], tempAngularVelocity);
      tempAngularVelocity.scale(1.0 / 3.0 / trajectoryTimeScale.getDoubleValue());
      tempAngularVelocity.negate();
      RotationTools.convertRotationVectorToQuaternion(tempAngularVelocity, tempQuatForControlQuats);
      tempControlQuaternions[2].mul(tempQuatForControlQuats);

      for (int i = 1; i <= 3; i++)
      {
         quaternionCalculus.inverseMultiply(tempControlQuaternions[i-1], tempControlQuaternions[i], tempQuatForControlQuats);
         quaternionCalculus.log(tempQuatForControlQuats, tempAngularVelocity);
         controlAngularVelocities[i].set(tempAngularVelocity);
      }

      for (int i = 0; i <= 3; i++)
         controlQuaternions[i].set(tempControlQuaternions[i]);
   }

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

      time = MathTools.clipToMinMax(time, 0.0, trajectoryTime.getDoubleValue());
      double timePrevious = time - dtForFiniteDifference;
      double timeNext = time + dtForFiniteDifference;

      interpolateOrientation(timePrevious, qInterpolatedPrevious);
      interpolateOrientation(timeNext, qInterpolatedNext);
      interpolateOrientation(time, qInterpolated);

      quaternionCalculus.computeQDotByFiniteDifferenceCentral(qInterpolatedPrevious, qInterpolatedNext, dtForFiniteDifference, qDot);
      quaternionCalculus.computeQDDotByFiniteDifferenceCentral(qInterpolatedPrevious, qInterpolated, qInterpolatedNext, dtForFiniteDifference, qDDot);

      quaternionCalculus.computeAngularVelocity(qInterpolated, qDot, tempAngularVelocity);
      quaternionCalculus.computeAngularAcceleration(qInterpolated, qDot, qDDot, tempAngularAcceleration);

      currentOrientation.set(qInterpolated);
      currentAngularVelocity.set(tempAngularVelocity);
      currentAngularAcceleration.set(tempAngularAcceleration);
   }

   private final Quat4d tempQuatForInterpolation = new Quat4d();

   private void interpolateOrientation(double time, Quat4d qInterpolated)
   {
      controlQuaternions[0].get(qInterpolated);

      updateBezierCoefficients(time);

      for (int i = 1; i <= 3; i++)
      {
         computeBezierQuaternionCurveTerm(i, tempQuatForInterpolation);
         qInterpolated.mul(tempQuatForInterpolation);
      }
   }

   /**
    * I use the developed equations to compute the Bezier basis functions. It should be faster.
    * @param time
    */
   private void updateBezierCoefficients(double time)
   {
      time *= trajectoryTimeScale.getDoubleValue();
      double timeSquare = time * time;
      double timeCube = timeSquare * time;
      beziers[1].set(3.0 * MathTools.square(1.0 - time) * time);
      beziers[2].set(3.0 * (timeSquare - timeCube));
      beziers[3].set(timeCube);

      bezierDerivatives[1].set(3.0 * (1.0 - 4.0 * time - 3.0 * timeSquare));
      bezierDerivatives[2].set(3.0 * (2.0 * time - 3.0 * timeSquare));
      bezierDerivatives[3].set(3.0 * timeSquare);

      for (int i = 1; i <= 3; i++)
      {
         cumulativeBeziers[i].set(0.0);
         cumulativeBezierDerivatives[i].set(0.0);
         for (int j = i; j <= 3; j++)
         {
            cumulativeBeziers[i].add(beziers[j].getDoubleValue());
            cumulativeBezierDerivatives[i].add(bezierDerivatives[j].getDoubleValue());
         }
      }
   }

   private void computeBezierQuaternionCurveTerm(int i, Quat4d resultToPack)
   {
      double cumulativeBernsteinCoefficient = cumulativeBeziers[i].getDoubleValue();
      controlAngularVelocities[i].get(tempAngularVelocity);
      tempAngularVelocity.scale(cumulativeBernsteinCoefficient);
      quaternionCalculus.exp(tempAngularVelocity, resultToPack);
   }

   @Override
   public boolean isDone()
   {
      return currentTime.getDoubleValue() >= trajectoryTime.getDoubleValue();
   }

   @Override
   public void get(FrameOrientation orientationToPack)
   {
      currentOrientation.getFrameOrientationIncludingFrame(orientationToPack);
   }

   @Override
   public void packAngularVelocity(FrameVector velocityToPack)
   {
      currentAngularVelocity.getFrameTupleIncludingFrame(velocityToPack);
   }

   @Override
   public void packAngularAcceleration(FrameVector accelerationToPack)
   {
      currentAngularAcceleration.getFrameTupleIncludingFrame(accelerationToPack);
   }

   @Override
   public void packAngularData(FrameOrientation orientationToPack, FrameVector angularVelocityToPack, FrameVector angularAccelerationToPack)
   {
      get(orientationToPack);
      packAngularVelocity(angularVelocityToPack);
      packAngularAcceleration(angularAccelerationToPack);
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
