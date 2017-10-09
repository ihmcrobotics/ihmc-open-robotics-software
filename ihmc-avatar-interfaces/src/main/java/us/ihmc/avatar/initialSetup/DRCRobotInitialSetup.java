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
}
