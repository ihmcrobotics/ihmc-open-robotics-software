package us.ihmc.avatar.initialSetup;

import javax.vecmath.Vector3d;

import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.wholeBodyController.DRCRobotJointMap;

public interface DRCRobotInitialSetup<T extends Robot>
{
   public abstract void initializeRobot(T robot, DRCRobotJointMap jointMap);

   public abstract void setInitialYaw(double yaw);
   public abstract double getInitialYaw();

   public abstract void setInitialGroundHeight(double groundHeight);
   public abstract double getInitialGroundHeight();

   public abstract void setOffset(Vector3d additionalOffset);
   public abstract void getOffset(Vector3d offsetToPack);
}
