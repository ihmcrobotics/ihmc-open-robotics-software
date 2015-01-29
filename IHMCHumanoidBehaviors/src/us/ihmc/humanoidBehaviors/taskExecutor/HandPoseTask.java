package us.ihmc.humanoidBehaviors.taskExecutor;

import us.ihmc.communication.packets.manipulation.HandPosePacket;
import us.ihmc.communication.packets.manipulation.HandPosePacket.Frame;
import us.ihmc.communication.util.PacketControllerTools;
import us.ihmc.humanoidBehaviors.behaviors.primitives.HandPoseBehavior;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.taskExecutor.Task;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class HandPoseTask implements Task
{
   private static final boolean DEBUG = false;
   private final HandPosePacket handPosePacket;
   private final HandPoseBehavior handPoseBehavior;

   private final DoubleYoVariable yoTime;
   private double behaviorDoneTime = Double.NaN;
   private final double sleepTime;

   public HandPoseTask(RobotSide robotSide, double[] desiredArmJointAngles, DoubleYoVariable yoTime, HandPoseBehavior handPoseBehavior, double trajectoryTime)
   {
      this(robotSide, desiredArmJointAngles, yoTime, handPoseBehavior, trajectoryTime, 0.0);
   }

   public HandPoseTask(RobotSide robotSide, double[] desiredArmJointAngles, DoubleYoVariable yoTime, HandPoseBehavior handPoseBehavior, double trajectoryTime,
         double sleepTime)
   {
      this.handPoseBehavior = handPoseBehavior;
      this.yoTime = yoTime;
      this.sleepTime = sleepTime;

      handPosePacket = new HandPosePacket(robotSide, trajectoryTime, desiredArmJointAngles);

   }

   public HandPoseTask(RobotSide robotSide,DoubleYoVariable yoTime, HandPoseBehavior handPoseBehavior, Frame frame ,RigidBodyTransform pose, double trajectoryTime)
   {
      this.handPoseBehavior = handPoseBehavior;
      this.yoTime = yoTime;
      this.sleepTime = 0.0;
      
      handPosePacket = PacketControllerTools.createHandPosePacket(frame, pose, robotSide, trajectoryTime);
   }
   
   public HandPoseTask(RobotSide robotSide,DoubleYoVariable yoTime, HandPoseBehavior handPoseBehavior, Frame frame ,FramePose pose, double trajectoryTime)
   {
      this.handPoseBehavior = handPoseBehavior;
      this.yoTime = yoTime;
      this.sleepTime = 0.0;
      
      RigidBodyTransform rigidBodyPose = new RigidBodyTransform();
      pose.getPose(rigidBodyPose);
      
      handPosePacket = PacketControllerTools.createHandPosePacket(frame, rigidBodyPose , robotSide, trajectoryTime);
   }
   
   public HandPoseTask(HandPosePacket goToHomePacket, HandPoseBehavior handPoseBehavior, DoubleYoVariable yoTime)
   {
      this.handPoseBehavior = handPoseBehavior;
      this.yoTime = yoTime;
      this.sleepTime = 0.0;
      this.handPosePacket = goToHomePacket;
   }
   
   @Override
   public void doTransitionIntoAction()
   {
      if (DEBUG)
         System.out.println("Started arm task");
      handPoseBehavior.initialize();
      handPoseBehavior.setInput(handPosePacket);
   }

   @Override
   public void doAction()
   {
      handPoseBehavior.doControl();
      if (Double.isNaN(behaviorDoneTime) && handPoseBehavior.isDone())
      {
         behaviorDoneTime = yoTime.getDoubleValue();
      }
   }

   @Override
   public void doTransitionOutOfAction()
   {
      if (DEBUG)
         System.out.println("Finished " + handPosePacket.robotSide.getCamelCaseNameForMiddleOfExpression() + " arm pose");
      handPoseBehavior.finalize();
   }

   @Override
   public boolean isDone()
   {
      boolean sleepTimeAchieved = yoTime.getDoubleValue() > behaviorDoneTime + sleepTime;
      return handPoseBehavior.isDone() && sleepTimeAchieved;
   }

}
