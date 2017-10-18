package us.ihmc.escher.parameters;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class EscherPhysicalProperties
{
   public static final double footsizeReduction = 0.05;

   public static final double ankleHeight = 0.0725;

   public static final double actualFootLength = 0.25;
   public static final double actualFootWidth = 0.15;
   public static final double footBack = 0.10; // // FIXME: 11/19/16
   public static final double footForward = actualFootLength - footBack; // // FIXME: 11/19/16

   public static final double footWidthForControl = actualFootWidth - footsizeReduction;
   public static final double footLengthForControl = actualFootLength - footsizeReduction;
   public static final double footBackForControl = footBack - footsizeReduction/2.0;
   public static final double footForwardForControl = footForward - footsizeReduction/2.0;

   public static final double thighLength = 0.41;
   public static final double shinLength = 0.41;

   public static final SideDependentList<RigidBodyTransform> soleToAnkleFrameTransforms = new SideDependentList<>();
   public static final SideDependentList<RigidBodyTransform> handControlFrameToWristTransforms = new SideDependentList<RigidBodyTransform>();

   static
   {
      for (RobotSide side : RobotSide.values)
      {
         RigidBodyTransform soleToAnkleFrame = new RigidBodyTransform();
//         soleToAnkleFrame.setEuler(new Vector3d(0.0, +0.13, 0.0));
         soleToAnkleFrame.setTranslation(new Vector3D(actualFootLength / 2.0 - footBack, 0.0, -EscherPhysicalProperties.ankleHeight));
         soleToAnkleFrameTransforms.put(side, soleToAnkleFrame);
      }
   }

   static
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBodyTransform controlFrameToWristTransform = new RigidBodyTransform();
         controlFrameToWristTransform.setTranslation(0.0, robotSide.negateIfRightSide(0.10), 0.0);
         handControlFrameToWristTransforms.put(robotSide, controlFrameToWristTransform);
      }
   }

   public static RigidBodyTransform getSoleToAnkleFrameTransform(RobotSide side)
   {
      return soleToAnkleFrameTransforms.get(side);
   }
}
