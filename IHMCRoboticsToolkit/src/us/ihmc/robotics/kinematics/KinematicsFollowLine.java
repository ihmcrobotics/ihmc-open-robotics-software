package us.ihmc.robotics.kinematics;

import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.screwTheory.GeometricJacobian;

/**
 * Interface which will plan and update the path of the endeffector along a line while maintaining the same orientation
 *
 * @author Peter Abeles
 */
public interface KinematicsFollowLine
{
   public void setWorkModel(GeometricJacobian model);

   public GeometricJacobian getModel();

   /**
    * Specifies the robot math.
    * @param start Initial location
    * @param stop Final location
    * @param direction Orientation that the end effector must maintain
    * @return true if possible and false if impossible
    */
   public boolean setPath(FramePoint start, FramePoint stop, FrameVector direction);

   /**
    * Checks to see if the desired robot joint angles need to be changed.  If they do, update them and return the appropriate status
    */
   public Status update();

   public static enum Status {
      NO_CHANGE,
      CHANGED,
      FATAL,
      FINISHED
   }

}
