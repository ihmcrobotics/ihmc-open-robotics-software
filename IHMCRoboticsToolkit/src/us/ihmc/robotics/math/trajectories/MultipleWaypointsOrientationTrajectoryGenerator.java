package us.ihmc.robotics.math.trajectories;

import java.util.ArrayList;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Matrix3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.math.frames.YoFrameQuaternion;
import us.ihmc.robotics.math.frames.YoFrameQuaternionInMultipleFrames;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.frames.YoFrameVectorInMultipleFrames;

public class MultipleWaypointsOrientationTrajectoryGenerator extends OrientationTrajectoryGeneratorInMultipleFrames
{
   private final ReferenceFrame frameAtWaypointOrientation;

   private final int maximumNumberOfWaypoints;
   private final boolean doVelocityAtWaypoints;

   private final YoVariableRegistry registry;

   private final DoubleYoVariable currentTrajectoryTime;
   private final YoFrameQuaternion currentOrientation;
   private final YoFrameVector currentAngularVelocity;
   private final YoFrameVector currentAngularAcceleration;

   private final IntegerYoVariable numberOfWaypoints;
   private final ArrayList<DoubleYoVariable> timeAtWaypoints;
   private final ArrayList<YoFrameQuaternion> orientationAtWaypoints;
   private final ArrayList<YoFrameVector> angularVelocityAtWaypoints;
   private final ArrayList<YoPolynomial[]> subTrajectories;
   private final ArrayList<YoFrameVector> deltaOrientionBetweenWaypoints;

   private final FrameOrientation initialOrientationOfCurrentSubTrajectory = new FrameOrientation();

   private final AxisAngle4d tempAxisAngle = new AxisAngle4d();
   private final Vector3d tempCurrentDeltaOrientation = new Vector3d();

   private final FrameOrientation tempFrameOrientation = new FrameOrientation();
   private final FrameVector tempFrameVector = new FrameVector();
   private final FrameVector initialAngularVelocityOfSubTrajectory = new FrameVector();
   private final FrameVector finalAngularVelocityOfSubTrajectory = new FrameVector();

   public MultipleWaypointsOrientationTrajectoryGenerator(String namePrefix, int maximumNumberOfWaypoints, boolean doVelocityAtWaypoints, boolean allowMultipleFrames,
         ReferenceFrame referenceFrame, YoVariableRegistry parentRegistry)
   {
      super(allowMultipleFrames, referenceFrame);

      this.maximumNumberOfWaypoints = maximumNumberOfWaypoints;
      this.doVelocityAtWaypoints = doVelocityAtWaypoints;

      frameAtWaypointOrientation = new ReferenceFrame("OrientationTrajectoryFrame", referenceFrame)
      {
         private static final long serialVersionUID = 8840430180170220404L;
         private final Matrix3d localMatrix = new Matrix3d();

         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            initialOrientationOfCurrentSubTrajectory.changeFrame(parentFrame);
            initialOrientationOfCurrentSubTrajectory.getMatrix3d(localMatrix);
            transformToParent.setRotation(localMatrix);
         }
      };

      registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());

      numberOfWaypoints = new IntegerYoVariable(namePrefix + "NumberOfWaypoints", registry);
      numberOfWaypoints.set(0);

      timeAtWaypoints = new ArrayList<>(maximumNumberOfWaypoints);
      orientationAtWaypoints = new ArrayList<>(maximumNumberOfWaypoints);
      angularVelocityAtWaypoints = new ArrayList<>(maximumNumberOfWaypoints);
      subTrajectories = new ArrayList<>(maximumNumberOfWaypoints - 1);
      deltaOrientionBetweenWaypoints = new ArrayList<>(maximumNumberOfWaypoints - 1);

      currentTrajectoryTime = new DoubleYoVariable(namePrefix + "CurrentTrajectoryTime", registry);

      String currentOrientationName = namePrefix + "CurrentOrientation";
      String currentAngularVelocityName = namePrefix + "CurrentAngularVelocity";
      String currentAngularAccelerationName = namePrefix + "CurrentAngularAcceleration";

      if (allowMultipleFrames)
      {
         YoFrameQuaternionInMultipleFrames currentOrientation = new YoFrameQuaternionInMultipleFrames(currentOrientationName, registry, referenceFrame);
         YoFrameVectorInMultipleFrames currentAngularVelocity = new YoFrameVectorInMultipleFrames(currentAngularVelocityName, registry, referenceFrame);
         YoFrameVectorInMultipleFrames currentAngularAcceleration = new YoFrameVectorInMultipleFrames(currentAngularAccelerationName, registry, referenceFrame);
         registerMultipleFramesHolders(currentOrientation, currentAngularVelocity, currentAngularAcceleration);

         this.currentOrientation = currentOrientation;
         this.currentAngularVelocity = currentAngularVelocity;
         this.currentAngularAcceleration = currentAngularAcceleration;
      }
      else
      {
         currentOrientation = new YoFrameQuaternion(currentOrientationName, referenceFrame, registry);
         currentAngularVelocity = new YoFrameVector(currentAngularVelocityName, referenceFrame, registry);
         currentAngularAcceleration = new YoFrameVector(currentAngularAccelerationName, referenceFrame, registry);
      }

      String orientationAtWayPointName = namePrefix + "OrientationAtWaypoint";
      String angularVelocityAtWaypointName = namePrefix + "ArngularVelocityAtWaypoint";
      String deltaOrientationBetweenWaypointsName = namePrefix + "DeltaOrientionBetweenWaypoints";

      for (int i = 0; i < maximumNumberOfWaypoints; i++)
      {
         if (allowMultipleFrames)
         {
            YoFrameQuaternionInMultipleFrames orientationAtWaypoint = new YoFrameQuaternionInMultipleFrames(orientationAtWayPointName + i, registry, referenceFrame);
            orientationAtWaypoints.add(orientationAtWaypoint);

            YoFrameVectorInMultipleFrames angularVelocityAtWaypoint = new YoFrameVectorInMultipleFrames(angularVelocityAtWaypointName + i, registry, referenceFrame);
            angularVelocityAtWaypoints.add(angularVelocityAtWaypoint);

            registerMultipleFramesHolders(orientationAtWaypoint, angularVelocityAtWaypoint);
         }
         else
         {
            YoFrameQuaternion orientationAtWaypoint = new YoFrameQuaternion(orientationAtWayPointName + i, referenceFrame, registry);
            orientationAtWaypoints.add(orientationAtWaypoint);

            YoFrameVector angularVelocityAtWaypoint = new YoFrameVector(angularVelocityAtWaypointName + i, referenceFrame, registry);
            angularVelocityAtWaypoints.add(angularVelocityAtWaypoint);
         }
      }

      for (int i = 0; i < maximumNumberOfWaypoints - 1; i++)
      {
         // Variables to store the difference of orientation between waypoints.
         // Should maybe use one frame per waypoint, just did not want to flood with reference frames.
         YoFrameVector deltaOrientionBetweenWaypoint = new YoFrameVector(deltaOrientationBetweenWaypointsName + i + "And" + (i + 1), frameAtWaypointOrientation, registry);
         deltaOrientionBetweenWaypoints.add(deltaOrientionBetweenWaypoint);
      }

      for (int i = 0; i < maximumNumberOfWaypoints; i++)
      {
         DoubleYoVariable timeAtWaypoint = new DoubleYoVariable(namePrefix + "TimeAtWaypoint" + i, registry);
         timeAtWaypoints.add(timeAtWaypoint);
      }

      int numberOfCoefficients = doVelocityAtWaypoints ? 4 : 2;
      for (int i = 0; i < maximumNumberOfWaypoints - 1; i++)
      {
         YoPolynomial subTrajectoryX = new YoPolynomial(namePrefix + "PolynomialX" + i, numberOfCoefficients, registry);
         YoPolynomial subTrajectoryY = new YoPolynomial(namePrefix + "PolynomialY" + i, numberOfCoefficients, registry);
         YoPolynomial subTrajectoryZ = new YoPolynomial(namePrefix + "PolynomialZ" + i, numberOfCoefficients, registry);
         subTrajectories.add(new YoPolynomial[] { subTrajectoryX, subTrajectoryY, subTrajectoryZ });
      }

      clear();

      parentRegistry.addChild(registry);
   }

   public void clear()
   {
      numberOfWaypoints.set(0);

      for (int i = 0; i < maximumNumberOfWaypoints; i++)
      {
         timeAtWaypoints.get(i).set(Double.NaN);
         orientationAtWaypoints.get(i).setToNaN();
         angularVelocityAtWaypoints.get(i).setToNaN();
      }

      for (int i = 0; i < numberOfWaypoints.getIntegerValue() - 1; i++)
      {
         deltaOrientionBetweenWaypoints.get(i).setToNaN();
      }
   }

   public void appendWaypoint(double timeAtWaypoint, Quat4d orientation)
   {
      if (doVelocityAtWaypoints)
         throw new RuntimeException("Need to provide angular velocity");
      checkNumberOfWaypoints(numberOfWaypoints.getIntegerValue() + 1);

      timeAtWaypoints.get(numberOfWaypoints.getIntegerValue()).set(timeAtWaypoint);
      orientationAtWaypoints.get(numberOfWaypoints.getIntegerValue()).set(orientation);

      numberOfWaypoints.increment();
   }

   public void appendWaypoint(double timeAtWaypoint, FrameOrientation orientation)
   {
      if (doVelocityAtWaypoints)
         throw new RuntimeException("Need to provide angular velocity");
      checkNumberOfWaypoints(numberOfWaypoints.getIntegerValue() + 1);

      timeAtWaypoints.get(numberOfWaypoints.getIntegerValue()).set(timeAtWaypoint);
      orientationAtWaypoints.get(numberOfWaypoints.getIntegerValue()).set(orientation);

      numberOfWaypoints.increment();
   }

   public void appendWaypoints(double[] timeAtWaypoints, Quat4d[] orientations)
   {
      if (timeAtWaypoints.length != orientations.length)
         throw new RuntimeException("Arguments are inconsistent.");

      checkNumberOfWaypoints(numberOfWaypoints.getIntegerValue() + timeAtWaypoints.length);

      for (int i = 0; i < timeAtWaypoints.length; i++)
      {
         appendWaypoint(timeAtWaypoints[i], orientations[i]);
      }
   }

   public void appendWaypoints(double[] timeAtWaypoints, FrameOrientation[] orientations)
   {
      if (timeAtWaypoints.length != orientations.length)
         throw new RuntimeException("Arguments are inconsistent.");

      checkNumberOfWaypoints(numberOfWaypoints.getIntegerValue() + timeAtWaypoints.length);

      for (int i = 0; i < timeAtWaypoints.length; i++)
      {
         appendWaypoint(timeAtWaypoints[i], orientations[i]);
      }
   }

   public void appendWaypoint(double timeAtWaypoint, Quat4d orientation, Vector3d angularVelocity)
   {
      checkNumberOfWaypoints(numberOfWaypoints.getIntegerValue() + 1);

      timeAtWaypoints.get(numberOfWaypoints.getIntegerValue()).set(timeAtWaypoint);
      orientationAtWaypoints.get(numberOfWaypoints.getIntegerValue()).set(orientation);
      if (doVelocityAtWaypoints)
         angularVelocityAtWaypoints.get(numberOfWaypoints.getIntegerValue()).set(angularVelocity);

      numberOfWaypoints.increment();
   }

   public void appendWaypoint(double timeAtWaypoint, FrameOrientation orientation, FrameVector angularVelocity)
   {
      checkNumberOfWaypoints(numberOfWaypoints.getIntegerValue() + 1);

      timeAtWaypoints.get(numberOfWaypoints.getIntegerValue()).set(timeAtWaypoint);
      orientationAtWaypoints.get(numberOfWaypoints.getIntegerValue()).set(orientation);
      if (doVelocityAtWaypoints)
         angularVelocityAtWaypoints.get(numberOfWaypoints.getIntegerValue()).set(angularVelocity);

      numberOfWaypoints.increment();
   }

   public void appendWaypoints(double[] timeAtWaypoints, Quat4d[] orientations, Vector3d[] angularVelocities)
   {
      if (doVelocityAtWaypoints && angularVelocities == null)
         throw new RuntimeException("Need to provide angular velocity");
      if (timeAtWaypoints.length != orientations.length || (angularVelocities != null && orientations.length != angularVelocities.length))
         throw new RuntimeException("Arguments are inconsistent.");

      checkNumberOfWaypoints(numberOfWaypoints.getIntegerValue() + timeAtWaypoints.length);

      for (int i = 0; i < timeAtWaypoints.length; i++)
      {
         if (doVelocityAtWaypoints)
            appendWaypoint(timeAtWaypoints[i], orientations[i], angularVelocities[i]);
         else
            appendWaypoint(timeAtWaypoints[i], orientations[i]);
      }
   }

   public void appendWaypoints(double[] timeAtWaypoints, FrameOrientation[] orientations, FrameVector[] angularVelocities)
   {
      if (timeAtWaypoints.length != orientations.length || orientations.length != angularVelocities.length)
         throw new RuntimeException("Arguments are inconsistent.");

      checkNumberOfWaypoints(numberOfWaypoints.getIntegerValue() + timeAtWaypoints.length);

      for (int i = 0; i < timeAtWaypoints.length; i++)
      {
         appendWaypoint(timeAtWaypoints[i], orientations[i], angularVelocities[i]);
      }
   }

   public void appendWaypoints(WaypointOrientationTrajectoryData trajectoryData)
   {
      trajectoryData.checkReferenceFrameMatch(currentOrientation.getReferenceFrame());
      appendWaypoints(trajectoryData.getTimeAtWaypoints(), trajectoryData.getOrientations(), trajectoryData.getAngularVelocities());
   }

   private void checkNumberOfWaypoints(int length)
   {
      if (length > maximumNumberOfWaypoints)
         throw new RuntimeException("Cannot exceed the maximum number of waypoints. Number of waypoints provided: " + length);
   }

   @Override
   public void initialize()
   {
      if (numberOfWaypoints.getIntegerValue() == 0)
      {
         throw new RuntimeException("Trajectory has no waypoints.");
      }

      double timeAtFirstWaypoint = timeAtWaypoints.get(0).getDoubleValue();
      for (int i = 0; i < numberOfWaypoints.getIntegerValue(); i++)
      {
         timeAtWaypoints.get(i).sub(timeAtFirstWaypoint);
      }

      for (int i = 0; i < numberOfWaypoints.getIntegerValue() - 1; i++)
      {
         YoFrameQuaternion initialOrientation = orientationAtWaypoints.get(i);
         YoFrameQuaternion finalOrientation = orientationAtWaypoints.get(i + 1);
         YoFrameVector initialAngularVelocity = angularVelocityAtWaypoints.get(i);
         YoFrameVector finalAngularVelocity = angularVelocityAtWaypoints.get(i + 1);

         initialOrientation.getFrameOrientationIncludingFrame(initialOrientationOfCurrentSubTrajectory);
         frameAtWaypointOrientation.update();
         finalOrientation.getFrameOrientationIncludingFrame(tempFrameOrientation);
         tempFrameOrientation.changeFrame(frameAtWaypointOrientation);
         tempFrameOrientation.getAxisAngle(tempAxisAngle);
         YoFrameVector deltaOrientation = deltaOrientionBetweenWaypoints.get(i);
         deltaOrientation.set(tempAxisAngle.getX(), tempAxisAngle.getY(), tempAxisAngle.getZ());
         deltaOrientation.scale(tempAxisAngle.getAngle());

         initialAngularVelocity.getFrameTupleIncludingFrame(initialAngularVelocityOfSubTrajectory);
         finalAngularVelocity.getFrameTupleIncludingFrame(finalAngularVelocityOfSubTrajectory);
         initialAngularVelocityOfSubTrajectory.changeFrame(frameAtWaypointOrientation);
         finalAngularVelocityOfSubTrajectory.changeFrame(frameAtWaypointOrientation);

         double t0 = timeAtWaypoints.get(i).getDoubleValue();
         double tf = timeAtWaypoints.get(i + 1).getDoubleValue();

         YoPolynomial[] subTrajectory = subTrajectories.get(i);
         if (doVelocityAtWaypoints)
         {
            subTrajectory[0].setCubic(t0, tf, 0.0, initialAngularVelocityOfSubTrajectory.getX(), deltaOrientation.getX(), finalAngularVelocityOfSubTrajectory.getX());
            subTrajectory[1].setCubic(t0, tf, 0.0, initialAngularVelocityOfSubTrajectory.getY(), deltaOrientation.getY(), finalAngularVelocityOfSubTrajectory.getY());
            subTrajectory[2].setCubic(t0, tf, 0.0, initialAngularVelocityOfSubTrajectory.getZ(), deltaOrientation.getZ(), finalAngularVelocityOfSubTrajectory.getZ());
         }
         else
         {
            subTrajectory[0].setLinear(t0, tf, 0.0, deltaOrientation.getX());
            subTrajectory[1].setLinear(t0, tf, 0.0, deltaOrientation.getY());
            subTrajectory[2].setLinear(t0, tf, 0.0, deltaOrientation.getZ());
         }
      }
   }

   @Override
   public void compute(double time)
   {
      if (numberOfWaypoints.getIntegerValue() == 0)
      {
         throw new RuntimeException("Trajectory has no waypoints.");
      }

      if (numberOfWaypoints.getIntegerValue() == 1)
      {
         currentOrientation.set(orientationAtWaypoints.get(0));
         currentAngularVelocity.setToZero();
         currentAngularAcceleration.setToZero();
         return;
      }

      double firstT = timeAtWaypoints.get(0).getDoubleValue();
      double lastT = timeAtWaypoints.get(numberOfWaypoints.getIntegerValue() - 1).getDoubleValue();
      int indexOfLastTrajectory = numberOfWaypoints.getIntegerValue() - 2;

      if (time <= firstT)
      {
         currentOrientation.set(orientationAtWaypoints.get(0));
         currentAngularVelocity.setToZero();
         currentAngularAcceleration.setToZero();
      }
      else if (time >= lastT)
      {
         currentOrientation.set(orientationAtWaypoints.get(numberOfWaypoints.getIntegerValue() - 1));
         currentAngularVelocity.setToZero();
         currentAngularAcceleration.setToZero();
      }
      else
      {
         for (int i = 0; i <= indexOfLastTrajectory; i++)
         {
            if (time < timeAtWaypoints.get(i + 1).getDoubleValue())
            {
               updateCurrent(i, time);
               break;
            }
         }
      }

      this.currentTrajectoryTime.set(time);

      if (!doVelocityAtWaypoints)
      {
         currentAngularVelocity.setToZero();
         currentAngularAcceleration.setToZero();
      }
   }

   private void updateCurrent(int currentSubTrajectoryIndex, double time)
   {
      YoFrameQuaternion initialOrientation = orientationAtWaypoints.get(currentSubTrajectoryIndex);
      initialOrientation.getFrameOrientationIncludingFrame(initialOrientationOfCurrentSubTrajectory);
      frameAtWaypointOrientation.update();

      YoPolynomial[] currentSubTrajectory = subTrajectories.get(currentSubTrajectoryIndex);
      currentSubTrajectory[0].compute(time);
      currentSubTrajectory[1].compute(time);
      currentSubTrajectory[2].compute(time);

      tempCurrentDeltaOrientation.setX(currentSubTrajectory[0].getPosition());
      tempCurrentDeltaOrientation.setY(currentSubTrajectory[1].getPosition());
      tempCurrentDeltaOrientation.setZ(currentSubTrajectory[2].getPosition());

      double angle = tempCurrentDeltaOrientation.length();
      if (Math.abs(angle) < 1e-5)
         tempCurrentDeltaOrientation.set(1.0, 0.0, 0.0);
      else
         tempCurrentDeltaOrientation.scale(1.0 / angle);
      tempAxisAngle.set(tempCurrentDeltaOrientation, angle);
      tempFrameOrientation.setIncludingFrame(frameAtWaypointOrientation, tempAxisAngle);
      tempFrameOrientation.changeFrame(currentOrientation.getReferenceFrame());
      currentOrientation.set(tempFrameOrientation);

      tempFrameVector.setToZero(frameAtWaypointOrientation);
      tempFrameVector.setX(currentSubTrajectory[0].getVelocity());
      tempFrameVector.setY(currentSubTrajectory[1].getVelocity());
      tempFrameVector.setZ(currentSubTrajectory[2].getVelocity());
      tempFrameVector.changeFrame(currentAngularVelocity.getReferenceFrame());
      currentAngularVelocity.set(tempFrameVector);

      tempFrameVector.setToZero(frameAtWaypointOrientation);
      tempFrameVector.setX(currentSubTrajectory[0].getAcceleration());
      tempFrameVector.setY(currentSubTrajectory[1].getAcceleration());
      tempFrameVector.setZ(currentSubTrajectory[2].getAcceleration());
      tempFrameVector.changeFrame(currentAngularAcceleration.getReferenceFrame());
      currentAngularAcceleration.set(tempFrameVector);
   }

   @Override
   public boolean isDone()
   {
      if (numberOfWaypoints.getIntegerValue() == 0)
         return true;

      double tFinal = timeAtWaypoints.get(numberOfWaypoints.getIntegerValue() - 1).getDoubleValue();
      return currentTrajectoryTime.getDoubleValue() > tFinal;
   }

   @Override
   public void get(FrameOrientation orientationToPack)
   {
      currentOrientation.getFrameOrientationIncludingFrame(orientationToPack);
   }

   @Override
   public void packAngularVelocity(FrameVector angularVelocityToPack)
   {
      currentAngularVelocity.getFrameTupleIncludingFrame(angularVelocityToPack);
   }

   @Override
   public void packAngularAcceleration(FrameVector angularAccelerationToPack)
   {
      currentAngularAcceleration.getFrameTupleIncludingFrame(angularAccelerationToPack);
   }

   @Override
   public void packAngularData(FrameOrientation orientationToPack, FrameVector angularVelocityToPack, FrameVector angularAccelerationToPack)
   {
      get(orientationToPack);
      packAngularVelocity(angularVelocityToPack);
      packAngularAcceleration(angularAccelerationToPack);
   }
}
