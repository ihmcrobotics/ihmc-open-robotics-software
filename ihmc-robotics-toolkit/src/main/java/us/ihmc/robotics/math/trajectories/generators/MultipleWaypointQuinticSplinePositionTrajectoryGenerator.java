package us.ihmc.robotics.math.trajectories.generators;


import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.math.interpolators.QuinticSplineInterpolator;
import us.ihmc.robotics.math.trajectories.PositionTrajectoryGenerator;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

/**
 * Helper class to create a position trajectory using the quintic spline interpolator
 * 
 * Not realtime safe. 
 */
public class MultipleWaypointQuinticSplinePositionTrajectoryGenerator implements PositionTrajectoryGenerator
{
   private final ReferenceFrame trajectoryFrame;
   private final QuinticSplineInterpolator interpolator;

   private int numberOfPoints;
   private final double[] time;
   private final double[] x;
   private final double[] y;
   private final double[] z;
   
   private final Vector3D v0 = new Vector3D();
   private final Vector3D a0 = new Vector3D();
   private final Vector3D vf = new Vector3D();
   private final Vector3D af = new Vector3D();
   

   public MultipleWaypointQuinticSplinePositionTrajectoryGenerator(String name, ReferenceFrame trajectoryFrame, int maximumNumberOfPoints, YoVariableRegistry parentRegistry)
   {
      this.trajectoryFrame = trajectoryFrame;
      this.interpolator = new QuinticSplineInterpolator(name, maximumNumberOfPoints, 3, parentRegistry);
      this.time = new double[maximumNumberOfPoints];
      this.x = new double[maximumNumberOfPoints];
      this.y = new double[maximumNumberOfPoints];
      this.z = new double[maximumNumberOfPoints];
      
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
   
   
   public void addWaypoint(double time, FrameTuple3DReadOnly position)
   {
      if(this.numberOfPoints >= this.interpolator.getMaximumNumberOfWaypoints())
      {
         throw new RuntimeException("Number of waypoints exceeds maximum number of waypoints");
      }
      
      trajectoryFrame.checkReferenceFrameMatch(position);
      
      this.time[this.numberOfPoints] = time;
      this.x[this.numberOfPoints] = position.getX();
      this.y[this.numberOfPoints] = position.getY();
      this.z[this.numberOfPoints] = position.getZ();
      
      this.numberOfPoints++;
      
   }
   
   public void setInitialConditions(FrameTuple3DReadOnly initialVelocity, FrameTuple3DReadOnly initialAcceleration)
   {
      trajectoryFrame.checkReferenceFrameMatch(initialVelocity);
      trajectoryFrame.checkReferenceFrameMatch(initialAcceleration);

      
      this.v0.set(initialVelocity);
      this.a0.set(initialAcceleration);
   }
   
   public void setFinalConditions(FrameTuple3DReadOnly finalVelocity, FrameTuple3DReadOnly finalAcceleration)
   {
      this.vf.set(finalVelocity);
      this.af.set(finalAcceleration);
   }

   public void initialize()
   {
      
      
      this.interpolator.initialize(this.numberOfPoints, time);

      this.interpolator.determineCoefficients(0, x, v0.getX(), vf.getX(), a0.getX(), af.getX());
      this.interpolator.determineCoefficients(0, y, v0.getY(), vf.getY(), a0.getY(), af.getY());
      this.interpolator.determineCoefficients(0, z, v0.getZ(), vf.getZ(), a0.getZ(), af.getZ());
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
      positionToPack.setX(this.interpolator.getPosition(0));
      positionToPack.setY(this.interpolator.getPosition(1));
      positionToPack.setZ(this.interpolator.getPosition(2));
   }

   @Override
   public void getVelocity(FrameVector3D velocityToPack)
   {
      velocityToPack.setX(this.interpolator.getVelocity(0));
      velocityToPack.setY(this.interpolator.getVelocity(1));
      velocityToPack.setZ(this.interpolator.getVelocity(2));
   }

   @Override
   public void getAcceleration(FrameVector3D accelerationToPack)
   {
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
