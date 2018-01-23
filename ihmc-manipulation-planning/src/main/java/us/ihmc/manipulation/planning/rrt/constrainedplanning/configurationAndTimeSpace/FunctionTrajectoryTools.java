package us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.WholeBodyTrajectoryToolboxMessageTools.FunctionTrajectory;

public class FunctionTrajectoryTools
{
   /**
    * Returns an implementation of {@code FunctionTrajectory} that implements a circle trajectory.
    * <p>
    * Together {@code circleCenter} and {@code circleOrientation} describe the frame in which the
    * circle is drawn. In this local frame, the circle is drawn on the XY-plane.
    * </p>
    * 
    * @param circleCenter the center of the circle. Not modified.
    * @param circleOrientation the orientation of the circle. Not modified.
    * @param outputOrientation the constant orientation the this trajectory should output. Not
    *           modified.
    * @param radius the circle's radius.
    * @param angleStart the angle from the local x-axis that the circle should start at.
    * @param clockwise whether the circle is to be drawn in a clockwise or counter-clockwise manner.
    * @param t0 the start time of the trajectory.
    * @param tf the end time of the trajectory.
    * @return the function trajectory of a circle.
    */
   public static FunctionTrajectory circleTrajectory(Point3DReadOnly circleCenter, QuaternionReadOnly circleOrientation, QuaternionReadOnly outputOrientation,
                                                     double radius, double angleStart, boolean clockwise, double t0, double tf)
   {
      return new FunctionTrajectory()
      {
         @Override
         public Pose3D compute(double time)
         {
            RigidBodyTransform circlePose = new RigidBodyTransform(circleOrientation, circleCenter);

            time = MathTools.clamp(time, t0, tf);
            double angle = angleStart + 2.0 * Math.PI * (time - t0) / (tf - t0);

            if (clockwise)
               angle = -angle;

            double xLocal = radius * Math.cos(angle);
            double yLocal = radius * Math.sin(angle);

            Point3D point = new Point3D(xLocal, yLocal, 0.0);
            circlePose.transform(point);

            return new Pose3D(point, outputOrientation);
         }
      };
   }
}
