package us.ihmc.commonWalkingControlModules.partNamesAndTorques;

import us.ihmc.robotSide.RobotSide;

public interface LegTorquesInterface
{
   public abstract void setKneeTorque(double kneeTorque);
   public abstract void addKneeTorque(double kneeTorque);
   public abstract RobotSide getRobotSide();
}
