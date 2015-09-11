package us.ihmc.robotics.math.trajectories;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.screwTheory.OneDoFJoint;

public class MultipleWaypointsOneDoFJointTrajectoryGenerator extends MultipleWaypointsTrajectoryGenerator implements OneDoFJointTrajectoryGenerator
{
   private final OneDoFJoint joint;

   public MultipleWaypointsOneDoFJointTrajectoryGenerator(String namePrefix, OneDoFJoint joint, YoVariableRegistry parentRegistry)
   {
      super(namePrefix, parentRegistry);
      this.joint = joint;

      clear();
   }

   @Override
   public void initialize(double initialPosition, double initialVelocity)
   { 
      super.setInitialCondition(initialPosition, initialVelocity);
   }

   @Override
   public void initialize()
   {
      super.initialize();
   }

   @Override
   public void clear()
   {
      super.clear();
   }

   @Override
   public boolean appendWaypoint(double timeSincePreviousWaypoint, double position, double velocity)
   {
      return super.appendWaypoint(timeSincePreviousWaypoint, position, velocity);
   }

   @Override
   public void setWaypoints(double[] timeAtWaypoints, double[] positions, double[] velocities)
   {
      clear();
      appendWaypoints(timeAtWaypoints, positions, velocities);
   }

   public OneDoFJoint getJoint()
   {
      return joint;
   }
}
