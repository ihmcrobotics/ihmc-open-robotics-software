package us.ihmc.valkyrie.parameters;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class ValkyriePhysicalProperties
{
   public static final double footsizeReduction = 0.01;

   public static final double ankleHeight = 0.09; // Should be 0.075 + 0.015 (sole thickness)
   public static final double footLength = 0.25 - footsizeReduction;
   public static final double footBack = 0.073 - footsizeReduction / 2.0;
   public static final double footForward = footLength - footBack;
   public static final double footWidth = 0.15 - footsizeReduction;

   public static final double thighLength = 0.431;
   public static final double shinLength = 0.406;

   public static final SideDependentList<RigidBodyTransform> soleToAnkleFrameTransforms = new SideDependentList<>();
   public static final SideDependentList<RigidBodyTransform> handControlFrameToWristTransforms = new SideDependentList<RigidBodyTransform>();
   public static final SideDependentList<RigidBodyTransform> handControlFrameToArmMassSimTransforms = new SideDependentList<RigidBodyTransform>();

   static
   {
      for (RobotSide side : RobotSide.values)
      {
         RigidBodyTransform soleToAnkleFrame = new RigidBodyTransform();
         //         soleToAnkleFrame.setEuler(new Vector3d(0.0, +0.13, 0.0));
         soleToAnkleFrame.setTranslation(new Vector3D(footLength / 2.0 - footBack, 0.0, -ValkyriePhysicalProperties.ankleHeight));
         soleToAnkleFrameTransforms.put(side, soleToAnkleFrame);
      }
   }

   static
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBodyTransform controlFrameToWristTransform = new RigidBodyTransform();
         controlFrameToWristTransform.setTranslation(0.025, robotSide.negateIfRightSide(0.07), 0.0);
         controlFrameToWristTransform.appendYawRotation(robotSide.negateIfRightSide(Math.PI * 0.5));
         handControlFrameToWristTransforms.put(robotSide, controlFrameToWristTransform);

         RigidBodyTransform controlFrameToArmMassSimTransform = new RigidBodyTransform();
         controlFrameToArmMassSimTransform.setTranslation(-0.0275, robotSide.negateIfRightSide(0.4), 0.0);
         controlFrameToArmMassSimTransform.appendYawRotation(robotSide.negateIfRightSide(0.5 * Math.PI));
         controlFrameToArmMassSimTransform.appendRollRotation(robotSide.negateIfRightSide(0.5 * Math.PI));
         handControlFrameToArmMassSimTransforms.put(robotSide, controlFrameToArmMassSimTransform);
      }
   }

   public static RigidBodyTransform getSoleToAnkleFrameTransform(RobotSide side)
   {
      return soleToAnkleFrameTransforms.get(side);
   }

   public static double getLegLength()
   {
      return thighLength + shinLength;
   }
}
