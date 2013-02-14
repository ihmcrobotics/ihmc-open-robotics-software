package us.ihmc.darpaRoboticsChallenge.handControl;

import java.util.EnumMap;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.screwTheory.OneDoFJoint;

public class SandiaHandModel
{
   public enum SandiaFingerName
   {
      THUMB,
      INDEX,
      MIDDLE,
      PINKY
   }
   
   private final EnumMap<SandiaFingerName, OneDoFJoint> fingerJoints = new EnumMap<SandiaFingerName, OneDoFJoint>(SandiaFingerName.class);
   private final RobotSide robotSide;

   public SandiaHandModel(SDFFullRobotModel fullRobotModel, RobotSide robotSide)
   {
      String prefix = robotSide == RobotSide.LEFT ? "left_" : "right_";

      this.robotSide = robotSide;
      
      fingerJoints.put(SandiaFingerName.THUMB, fullRobotModel.getOneDoFJointByName(prefix + "f3_j0"));
      fingerJoints.put(SandiaFingerName.INDEX, fullRobotModel.getOneDoFJointByName(prefix + "f0_j0"));
      fingerJoints.put(SandiaFingerName.MIDDLE, fullRobotModel.getOneDoFJointByName(prefix + "f1_j0"));
      fingerJoints.put(SandiaFingerName.PINKY, fullRobotModel.getOneDoFJointByName(prefix + "f2_j0"));
   }

   public OneDoFJoint getBaseJoint(SandiaFingerName name)
   {
      return fingerJoints.get(name);
   }
   
   public RobotSide getRobotSide()
   {
      return robotSide;
   }

}
