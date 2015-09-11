package us.ihmc.robotics.math.trajectories;

import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.ReferenceFrameHolder;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class WaypointOrientationTrajectoryData extends ReferenceFrameHolder
{
   private ReferenceFrame referenceFrame;
   private final int numberOfWaypoints;
   private final double[] timeAtWaypoints;
   private final Quat4d[] orientations;
   private final Vector3d[] angularVelocities;

   // Temporary variables to change frame easily
   private final FrameOrientation tempFrameOrientation = new FrameOrientation();
   private final FrameVector tempFrameVector = new FrameVector();

   public WaypointOrientationTrajectoryData(ReferenceFrame referenceFrame, double[] timeAtWaypoints, Quat4d[] orientations, Vector3d[] angularVelocities)
   {
      if (timeAtWaypoints.length != orientations.length || (angularVelocities != null && orientations.length != angularVelocities.length))
         throw new RuntimeException("Arguments are inconsistent");

      this.referenceFrame = referenceFrame;
      numberOfWaypoints = timeAtWaypoints.length;
      this.timeAtWaypoints = new double[numberOfWaypoints];
      this.orientations = new Quat4d[numberOfWaypoints];

      System.arraycopy(timeAtWaypoints, 0, this.timeAtWaypoints, 0, numberOfWaypoints);
      System.arraycopy(orientations, 0, this.orientations, 0, numberOfWaypoints);

      if (angularVelocities != null)
      {
         this.angularVelocities = new Vector3d[numberOfWaypoints];
         System.arraycopy(angularVelocities, 0, this.angularVelocities, 0, numberOfWaypoints);
      }
      else
      {
         this.angularVelocities = null;
      }
   }

   public double[] getTimeAtWaypoints()
   {
      return timeAtWaypoints;
   }

   public Quat4d[] getOrientations()
   {
      return orientations;
   }

   public Vector3d[] getAngularVelocities()
   {
      return angularVelocities;
   }

   public void changeFrame(ReferenceFrame desiredFrame)
   {
      for (int i = 0; i < numberOfWaypoints; i++)
      {
         Quat4d orientation = orientations[i];
         tempFrameOrientation.setIncludingFrame(referenceFrame, orientation);
         tempFrameOrientation.changeFrame(desiredFrame);
         tempFrameOrientation.getQuaternion(orientation);

         if (angularVelocities != null)
         {
            Vector3d velocity = angularVelocities[i];
            tempFrameVector.setIncludingFrame(referenceFrame, velocity);
            tempFrameVector.changeFrame(referenceFrame);
            tempFrameVector.get(velocity);
         }
      }

      referenceFrame = desiredFrame;
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }
}
