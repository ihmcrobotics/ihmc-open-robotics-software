package us.ihmc.darpaRoboticsChallenge.handControl;

import java.util.ArrayList;

import us.ihmc.commonWalkingControlModules.controllers.HandControllerInterface;
import us.ihmc.darpaRoboticsChallenge.networking.dataProducers.JointConfigurationGathererAndProducer;
import us.ihmc.robotSide.RobotSide;

public interface HandControllerDispatcherInterface
{
   public void setHandController(RobotSide robotSide, HandControllerWithYoVariables simulatedUnderactuatedSandiaHandController);
   public void addHandJoints(RobotSide robotSide, ArrayList<FingerJoint> handJoints);
   public void setJointConfigurationGathererAndProducer(JointConfigurationGathererAndProducer jointConfigurationGathererAndProducer);
}
