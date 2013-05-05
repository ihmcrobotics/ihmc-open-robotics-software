package us.ihmc.darpaRoboticsChallenge.handControl;

import java.util.ArrayList;

import us.ihmc.robotSide.RobotSide;

public interface HandControllerDispatcherInterface
{
   public void setHandController(RobotSide robotSide, SimulatedUnderactuatedSandiaHandController simulatedUnderactuatedSandiaHandController);
   public void addHandJoints(RobotSide robotSide, ArrayList<FingerJoint> handJoints);
}
