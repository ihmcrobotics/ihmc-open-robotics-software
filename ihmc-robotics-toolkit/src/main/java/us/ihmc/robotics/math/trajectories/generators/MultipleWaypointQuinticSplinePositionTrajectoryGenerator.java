package us.ihmc.robotics.math.trajectories.generators;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.math.interpolators.QuinticSplineInterpolator;
import us.ihmc.robotics.math.trajectories.PositionTrajectoryGenerator;
import us.ihmc.yoVariables.registry.YoRegistry;

/**
 * Helper class to create a position trajectory using the quintic spline interpolator Not realtime
 * safe.
 */
public class MultipleWaypointQuinticSplinePositionTrajectoryGenerator implements PositionTrajectoryGenerator
{
   private final QuinticSplineInterpolator interpolator;
   private ReferenceFrame trajectoryFrame;

   private int numberOfPoints;
   private final double[] time;
   private final double[] x;
   private final double[] y;
   private final double[] z;

   private final FrameVector3D v0;
   private final FrameVector3D a0;
   private final FrameVector3D vf;
   private final FrameVector3D af;

   private final FramePoint3D tempPosition;

   public MultipleWaypointQuinticSplinePositionTrajectoryGenerator(String name, ReferenceFrame trajectoryFrame, int maximumNumberOfPoints,
                                                                   YoRegistry parentRegistry)
   {
      this.interpolator = new QuinticSplineInterpolator(name, maximumNumberOfPoints, 3, parentRegistry);
      this.trajectoryFrame = trajectoryFrame;
      this.time = new double[maximumNumberOfPoints];
      this.x = new double[maximumNumberOfPoints];
      this.y = new double[maximumNumberOfPoints];
      this.z = new double[maximumNumberOfPoints];

      this.tempPosition = new FramePoint3D(trajectoryFrame);

      this.v0 = new FrameVector3D(trajectoryFrame);
      this.a0 = new FrameVector3D(trajectoryFrame);
      this.vf = new FrameVector3D(trajectoryFrame);
      this.af = new FrameVector3D(trajectoryFrame);
   }

   /**
    * Clear all waypoints out of this trajectory
    */
   public void clear()
   {
      numberOfPoints = 0;
      v0.setToZero();
      a0.setToZero();
      vf.setToZero();
      af.setToZero();
   }

   public void clearAndSetTrajectoryFrame(ReferenceFrame trajectoryFrame)
   {
      this.trajectoryFrame = trajectoryFrame;
      
      numberOfPoints = 0;
      v0.setToZero(trajectoryFrame);
      a0.setToZero(trajectoryFrame);
      vf.setToZero(trajectoryFrame);
      af.setToZero(trajectoryFrame);
      tempPosition.setToZero(trajectoryFrame);
   }

   public void addWaypoint(double time, FrameTuple3DReadOnly position)
   {
      if (this.numberOfPoints >= this.interpolator.getMaximumNumberOfWaypoints())
      {
         throw new RuntimeException("Number of waypoints exceeds maximum number of waypoints");
      }

      this.tempPosition.setMatchingFrame(position);

      this.time[this.numberOfPoints] = time;
      this.x[this.numberOfPoints] = tempPosition.getX();
      this.y[this.numberOfPoints] = tempPosition.getY();
      this.z[this.numberOfPoints] = tempPosition.getZ();

      this.numberOfPoints++;

   }

   public void setInitialConditions(FrameTuple3DReadOnly initialVelocity, FrameTuple3DReadOnly initialAcceleration)
   {
      this.v0.setMatchingFrame(initialVelocity);
      this.a0.setMatchingFrame(initialAcceleration);
   }

   public void setFinalConditions(FrameTuple3DReadOnly finalVelocity, FrameTuple3DReadOnly finalAcceleration)
   {
      this.vf.setMatchingFrame(finalVelocity);
      this.af.setMatchingFrame(finalAcceleration);
   }

   public void initialize()
   {
      this.interpolator.initialize(this.numberOfPoints, time);

      this.interpolator.determineCoefficients(0, x, v0.getX(), vf.getX(), a0.getX(), af.getX());
      this.interpolator.determineCoefficients(1, y, v0.getY(), vf.getY(), a0.getY(), af.getY());
      this.interpolator.determineCoefficients(2, z, v0.getZ(), vf.getZ(), a0.getZ(), af.getZ());
   }

   public void compute(double time)
   {
      this.interpolator.compute(time);
   }

   @Override
   public boolean isDone()
   {
      return this.interpolator.isDone();
   }

   @Override
   public void getPosition(FramePoint3D positionToPack)
   {
      trajectoryFrame.checkReferenceFrameMatch(positionToPack);

      positionToPack.setX(this.interpolator.getPosition(0));
      positionToPack.setY(this.interpolator.getPosition(1));
      positionToPack.setZ(this.interpolator.getPosition(2));
   }

   @Override
   public void getVelocity(FrameVector3D velocityToPack)
   {
      trajectoryFrame.checkReferenceFrameMatch(velocityToPack);

      velocityToPack.setX(this.interpolator.getVelocity(0));
      velocityToPack.setY(this.interpolator.getVelocity(1));
      velocityToPack.setZ(this.interpolator.getVelocity(2));
   }

   @Override
   public void getAcceleration(FrameVector3D accelerationToPack)
   {
      trajectoryFrame.checkReferenceFrameMatch(accelerationToPack);

      accelerationToPack.setX(this.interpolator.getAcceleration(0));
      accelerationToPack.setY(this.interpolator.getAcceleration(1));
      accelerationToPack.setZ(this.interpolator.getAcceleration(2));
   }

   @Override
   public void showVisualization()
   {

   }

   @Override
   public void hideVisualization()
   {

   }
}
