package us.ihmc.robotics.math.trajectories;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.trajectories.providers.PositionProvider;

/*
 * Note: this class can be used to interpolate N variables simultaneously.
 * You don't have a provider for the waypoint, therefore you should use method initialize(
 */
public class MultipleWaypointsPositionTrajectoryGenerator implements PositionTrajectoryGenerator
{
   private final MultipleWaypointsTrajectoryGenerator[] trajectories = new MultipleWaypointsTrajectoryGenerator[3];
   private final ReferenceFrame referenceFrame;
   private final PositionProvider currentPelvisPosition;

   public MultipleWaypointsPositionTrajectoryGenerator(String namePrefix, ReferenceFrame trajectoryFrame,
         PositionProvider currentPelvisPosition,  YoVariableRegistry parentRegistry)
   {
      this.referenceFrame = trajectoryFrame;
      this.currentPelvisPosition = currentPelvisPosition;

      trajectories[0] = new MultipleWaypointsTrajectoryGenerator(namePrefix + "_X_", parentRegistry);
      trajectories[1] = new MultipleWaypointsTrajectoryGenerator(namePrefix + "_Y_", parentRegistry);
      trajectories[2] = new MultipleWaypointsTrajectoryGenerator(namePrefix + "_Z_", parentRegistry);
   }

   @Override
   public void get(FramePoint positionToPack)
   {
      positionToPack.setIncludingFrame(referenceFrame, trajectories[0].getValue(), trajectories[1].getValue(), trajectories[2].getValue());
   }

   @Override
   public void packVelocity(FrameVector velocityToPack)
   {
      velocityToPack.setIncludingFrame(referenceFrame, trajectories[0].getVelocity(), trajectories[1].getVelocity(), trajectories[2].getVelocity());
   }

   @Override
   public void packAcceleration(FrameVector accelerationToPack)
   {
      accelerationToPack.setIncludingFrame(referenceFrame, trajectories[0].getAcceleration(), trajectories[1].getAcceleration(),
            trajectories[2].getAcceleration());
   }

   @Override
   public void packLinearData(FramePoint positionToPack, FrameVector velocityToPack, FrameVector accelerationToPack)
   {
      get(positionToPack);
      packVelocity(velocityToPack);
      packAcceleration(accelerationToPack);
   }

   public void clear()
   {
      for (int i = 0; i < trajectories.length; i++)
      {
         trajectories[i].clear();
      }
   }

   public boolean appendWaypoint(double timeAtWaypoint, Point3d position, Vector3d velocity)
   {
      boolean success = trajectories[0].appendWaypoint(timeAtWaypoint, position.getX(), velocity.getX());
      success &= trajectories[1].appendWaypoint(timeAtWaypoint, position.getY(), velocity.getY());
      success &= trajectories[2].appendWaypoint(timeAtWaypoint, position.getZ(), velocity.getZ());
      
      if (!success)
      {
         clear();
      }
      return success;
   }

   public void appendWaypoint(double timeAtWaypoint, FramePoint position, FrameVector velocity)
   {
      position.checkReferenceFrameMatch(referenceFrame);
      velocity.checkReferenceFrameMatch(referenceFrame);
      
      Point3d pos = new Point3d();
      Vector3d vel = new Vector3d();
      position.get(pos);
      velocity.get(vel);
      
      appendWaypoint(timeAtWaypoint, pos, vel);
   }

   public void appendWaypoints(double[] timeAtWaypoints, FramePoint[] positions, FrameVector[] velocities)
   {
      if (timeAtWaypoints.length != positions.length || positions.length != velocities.length)
         throw new RuntimeException("Arguments are inconsistent");

      for (int i = 0; i < timeAtWaypoints.length; i++)
      {
         referenceFrame.checkReferenceFrameMatch(positions[i]);
         referenceFrame.checkReferenceFrameMatch(velocities[i]);
         appendWaypoint(timeAtWaypoints[i], positions[i], velocities[i]);
      }
   }

   public void appendWaypoints(double[] timeAtWaypoints, Point3d[] positions, Vector3d[] velocities)
   {
      if (timeAtWaypoints.length != positions.length || positions.length != velocities.length)
         throw new RuntimeException("Arguments are inconsistent");

      for (int i = 0; i < timeAtWaypoints.length; i++)
      {
         appendWaypoint(timeAtWaypoints[i], positions[i], velocities[i]);
      }
   }

   public void appendWaypoints(WaypointPositionTrajectoryData trajectoryData)
   {
      trajectoryData.checkReferenceFrameMatch(referenceFrame);
      appendWaypoints(trajectoryData.getTimeAtWaypoints(), trajectoryData.getPositions(), trajectoryData.getVelocities());
   }

   public void setWaypoints(WaypointPositionTrajectoryData trajectoryData)
   {
      clear();
      appendWaypoints(trajectoryData);
   }

   @Override
   public void initialize()
   {
      FramePoint positionToPack = new FramePoint(ReferenceFrame.getWorldFrame());
      currentPelvisPosition.get(positionToPack);
      positionToPack.changeFrame( ReferenceFrame.getWorldFrame() );
      
      trajectories[0].setInitialCondition(positionToPack.getX(), 0.0);
      trajectories[1].setInitialCondition(positionToPack.getY(), 0.0);
      trajectories[2].setInitialCondition(positionToPack.getZ(), 0.0);
   }
   
   public void initialize(FramePoint position, FrameVector velocity)
   {
      position.changeFrame( ReferenceFrame.getWorldFrame() );
      velocity.changeFrame( ReferenceFrame.getWorldFrame() );
      
      trajectories[0].setInitialCondition(position.getX(), velocity.getX());
      trajectories[1].setInitialCondition(position.getY(), velocity.getY());
      trajectories[2].setInitialCondition(position.getZ(), velocity.getZ());
   }

   @Override
   public void compute(double time)
   {
      trajectories[0].compute(time);
      trajectories[1].compute(time);
      trajectories[2].compute(time);
   }

   @Override
   public boolean isDone()
   {
      return trajectories[0].isDone() && trajectories[1].isDone() && trajectories[2].isDone();
   }

   @Override
   public void showVisualization()
   {
      // TODO Auto-generated method stub
   }

   @Override
   public void hideVisualization()
   {
      // TODO Auto-generated method stub
   }
}
