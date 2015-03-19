package us.ihmc.valkyrie.parameters;

import javax.vecmath.Vector3d;

import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotPhysicalProperties;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.robotSide.SideDependentList;

public class ValkyriePhysicalProperties extends DRCRobotPhysicalProperties
{
   public static final double scale = 1.0;
   
   public static final double ankleHeight = 0.09; // Should be 0.075 + 0.015 (sole thickness)
   public static final double footLength = 0.254;
   public static final double footBack = 0.058;
   public static final double footWidth = 0.152;
   public static final double footForward = footLength - footBack;
   
   public static final double footBackForControl = footBack - 0.038;
   public static final double footWidthForControl = footWidth - 0.042;
   public static final double footLengthForControl = footLength - 0.04; 
   public static final double footForwardForControl = footLengthForControl - footBackForControl;
   public static final double thighLength = 0.431;
   public static final double shinLength = 0.406;
   
   public static final double footChamferX = 0.15 * footLength;
   public static final double footChamferY = 0.15 * footWidth;

   public static final SideDependentList<RigidBodyTransform> soleToAnkleFrameTransforms = new SideDependentList<>();
   public static final SideDependentList<RigidBodyTransform> handControlFrameToWristTransforms = new SideDependentList<RigidBodyTransform>(new RigidBodyTransform(), new RigidBodyTransform());

   static
   {
      for (RobotSide side : RobotSide.values)
      {
         RigidBodyTransform soleToAnkleFrame = new RigidBodyTransform();
//         soleToAnkleFrame.setEuler(new Vector3d(0.0, +0.13, 0.0));
         soleToAnkleFrame.setTranslation(new Vector3d(footLength / 2.0 - footBack, 0.0, -ValkyriePhysicalProperties.ankleHeight));
         soleToAnkleFrameTransforms.put(side, soleToAnkleFrame);
      }
   }

   @Override
   public double getAnkleHeight()
   {
      return ankleHeight;
   }

   public static RigidBodyTransform getAnkleToSoleFrameTransform(RobotSide side)
   {
      return soleToAnkleFrameTransforms.get(side);
   }
}
