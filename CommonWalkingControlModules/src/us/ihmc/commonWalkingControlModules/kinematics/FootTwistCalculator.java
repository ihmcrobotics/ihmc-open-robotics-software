package us.ihmc.commonWalkingControlModules.kinematics;

import us.ihmc.sensorProcessing.ProcessedSensorsInterface;
import us.ihmc.humanoidRobotics.model.FullRobotModel;
import us.ihmc.humanoidRobotics.partNames.LegJointName;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.Twist;

public class FootTwistCalculator
{
   private final ProcessedSensorsInterface processedSensors;
   private final RobotSide robotSide;
   private final FullRobotModel fullRobotModel;
   private final Twist tempTwist = new Twist();
   private final RigidBody pelvis;
   private final ReferenceFrame footFrame;

   public FootTwistCalculator(RobotSide robotSide, ProcessedSensorsInterface processedSensors)
   {
      this.robotSide = robotSide;
      this.processedSensors = processedSensors;
      this.fullRobotModel = processedSensors.getFullRobotModel();
      this.pelvis = fullRobotModel.getPelvis();
      this.footFrame = fullRobotModel.getFoot(robotSide).getParentJoint().getFrameAfterJoint();
   }

   public Twist computeFootTwist()
   {
      Twist ret = processedSensors.getTwistOfPelvisWithRespectToWorld();
      ret.changeBodyFrameNoRelativeTwist(pelvis.getBodyFixedFrame());

      for (LegJointName legJointName : fullRobotModel.getRobotSpecificJointNames().getLegJointNames())
      {
         OneDoFJoint joint = fullRobotModel.getLegJoint(robotSide, legJointName);
         joint.packSuccessorTwist(tempTwist);
         ret.changeFrame(tempTwist.getExpressedInFrame());
         ret.add(tempTwist);
      }

      ret.changeBodyFrameNoRelativeTwist(footFrame);
      ret.changeFrame(footFrame);

      return ret;
   }
}
