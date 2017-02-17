package us.ihmc.steppr.parameters;

import us.ihmc.avatar.drcRobot.DRCRobotPhysicalProperties;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.geometry.TransformTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class BonoPhysicalProperties implements DRCRobotPhysicalProperties
{
   /* Original Ankle
   public static final double ankleHeight = 2.0 * 0.0254;
   public static final double shiftFootForward = 0.000;
   public static final double shortenFoot = 0.0;//0.025;
   public static final double footForward = 0.202 + shiftFootForward - shortenFoot;
   public static final double heelExtension = 2*0.0254;
   public static final double footBack = 0.05 + heelExtension - shiftFootForward;
   public static final double footLength = footForward + footBack;
   public static final double toeWidth = 0.152;
   public static final double footWidth = toeWidth - 0.022;
   public static final double thighLength = 0.37694;
   public static final double shinLength = 0.42164;
   public static final double legLength = 1.01 * (thighLength + shinLength);
   public static final double pelvisToFoot = 0.887;
   */
   
   /* Spring Ankle */
   public static final double ankleHeight = 3.0 * 0.0254;
   public static final double shiftFootForward = 0.001;
   public static final double shortenFoot = 0.0;//0.025;
   public static final double footForward = 0.202 + shiftFootForward - shortenFoot - 0.005;
   public static final double heelExtension = 2*0.0254;
   public static final double footBack = 0.092 - 0.005;//0.05 + heelExtension - shiftFootForward;
   public static final double footLength = footForward + footBack;
   public static final double toeWidth = 0.152 - 0.02;
   public static final double footWidth = toeWidth - 0.022 - 0.02;
   public static final double thighLength = 0.37694;
   public static final double shinLength = 0.42164-0.6*0.0254;
   public static final double legLength = 1.01 * (thighLength + shinLength);
   public static final double pelvisToFoot = 0.869;
   

   public static final SideDependentList<RigidBodyTransform> soleToAnkleFrameTransforms = new SideDependentList<RigidBodyTransform>();
   static
   {
      for (RobotSide robotSide : RobotSide.values())
      {
         RigidBodyTransform soleToAnkleFrame = TransformTools.yawPitchDegreesTransform(new Vector3D(footLength / 2.0 - footBack, 0.0, -ankleHeight), 0.0, Math.toDegrees(0.0 * 0.18704));
         soleToAnkleFrameTransforms.put(robotSide, soleToAnkleFrame);
      }
   }

   @Override
   public double getAnkleHeight()
   {
      return ankleHeight;
   }

   public static RigidBodyTransform getSoleToAnkleFrameTransform(RobotSide side)
   {
      return soleToAnkleFrameTransforms.get(side);
   }
}
