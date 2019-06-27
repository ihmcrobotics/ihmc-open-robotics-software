package us.ihmc.avatar.initialSetup;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.wholeBodyController.DRCRobotJointMap;

public interface DRCRobotInitialSetup<T extends Robot>
{
   public abstract void initializeRobot(T robot, DRCRobotJointMap jointMap);

   public abstract void setInitialYaw(double yaw);
   public abstract double getInitialYaw();

   public abstract void setInitialGroundHeight(double groundHeight);
   public abstract double getInitialGroundHeight();

   public abstract void setOffset(Vector3D additionalOffset);
   public abstract void getOffset(Vector3D offsetToPack);

   /**
    * Indicates whether the robot can be reset to its initial configuration with the implementation of this initial
    * setup. This is deprecated since all initial setups should support this eventually. An example of an initial setup
    * that can be reused is the Atlas one.
    *
    * @return whether this initial setup will work exactly once.
    */
   @Deprecated
   public default boolean supportsReset()
   {
      return false;
   }
}
