package us.ihmc.robotics.math.trajectories;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.ReferenceFrameHolder;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class WaypointPositionTrajectoryData extends ReferenceFrameHolder
{
   private ReferenceFrame referenceFrame;
   private final int numberOfWaypoints;
   private final double[] timeAtWaypoints;
   private final Point3d[] positions;
   private final Vector3d[] velocities;

   // Temporary variables to change frame easily
   private final FramePoint tempFramePoint = new FramePoint();
   private final FrameVector tempFrameVector = new FrameVector();

   public WaypointPositionTrajectoryData(ReferenceFrame referenceFrame, double[] timeAtWaypoints, Point3d[] positions, Vector3d[] velocities)
   {
      if (timeAtWaypoints.length != positions.length || positions.length != velocities.length)
         throw new RuntimeException("Arguments are inconsistent");

      this.referenceFrame = referenceFrame;
      numberOfWaypoints = timeAtWaypoints.length;
      this.timeAtWaypoints = new double[numberOfWaypoints];
      this.positions = new Point3d[numberOfWaypoints];
      this.velocities = new Vector3d[numberOfWaypoints];

      System.arraycopy(timeAtWaypoints, 0, this.timeAtWaypoints, 0, numberOfWaypoints);
      System.arraycopy(positions, 0, this.positions, 0, numberOfWaypoints);
      System.arraycopy(velocities, 0, this.velocities, 0, numberOfWaypoints);
   }

   public double[] getTimeAtWaypoints()
   {
      return timeAtWaypoints;
   }

   public Point3d[] getPositions()
   {
      return positions;
   }

   public Vector3d[] getVelocities()
   {
      return velocities;
   }

   public void changeFrame(ReferenceFrame desiredFrame)
   {
      for (int i = 0; i < numberOfWaypoints; i++)
      {
         Point3d position = positions[i];
         tempFramePoint.setIncludingFrame(referenceFrame, position);
         tempFramePoint.changeFrame(desiredFrame);
         tempFramePoint.get(position);
         
         Vector3d velocity = velocities[i];
         tempFrameVector.setIncludingFrame(referenceFrame, velocity);
         tempFrameVector.changeFrame(referenceFrame);
         tempFrameVector.get(velocity);
      }

      referenceFrame = desiredFrame;
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }
}
