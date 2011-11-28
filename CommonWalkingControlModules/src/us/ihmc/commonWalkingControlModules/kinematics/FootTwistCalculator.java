package us.ihmc.commonWalkingControlModules.kinematics;

import java.util.ArrayList;

import us.ihmc.commonWalkingControlModules.sensors.ProcessedSensorsInterface;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.RevoluteJoint;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.Twist;

public class FootTwistCalculator
{
   private final ProcessedSensorsInterface processedSensors;
   private final ArrayList<RevoluteJoint> legJointList;
   private final Twist tempTwist = new Twist();
   private final RigidBody pelvis;
   private final ReferenceFrame footFrame;

   public FootTwistCalculator(RobotSide robotSide, ProcessedSensorsInterface processedSensors)
   {
      this.processedSensors = processedSensors;
      this.legJointList = processedSensors.getFullRobotModel().getLegJointList(robotSide);
      this.pelvis = processedSensors.getFullRobotModel().getPelvis();
      this.footFrame = legJointList.get(legJointList.size() - 1).getFrameAfterJoint();
   }

   public Twist computeFootTwist()
   {
      Twist ret = processedSensors.getTwistOfPelvisWithRespectToWorld();
      ret.changeBodyFrameNoRelativeTwist(pelvis.getBodyFixedFrame());

      for (RevoluteJoint joint : legJointList)
      {
         joint.packSuccessorTwist(tempTwist);
         ret.changeFrame(tempTwist.getExpressedInFrame());
         ret.add(tempTwist);
      }

      ret.changeBodyFrameNoRelativeTwist(footFrame);
      ret.changeFrame(footFrame);

      return ret;
   }
}
