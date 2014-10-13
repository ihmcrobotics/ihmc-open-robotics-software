package us.ihmc.valkyrie.paramaters;

import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import javax.vecmath.Vector3d;

import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotPhysicalProperties;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;

public class ValkyriePhysicalProperties extends DRCRobotPhysicalProperties
{
   public static final double footsizeReduction = 0.0;
   
   public static final double ankleHeight = 0.082;
   public static final double footLength = 0.25 - footsizeReduction;
   public static final double footBack = 0.058;
   public static final double footForward = footLength - footBack;
   public static final double footWidth = 0.15 - footsizeReduction;
   public static final double thighLength = 0.431;
   public static final double shinLength = 0.406;
   public static final double pelvisToFoot = 0.887 + 0.3;
   
   public static final double footChamferX = 0.2 * footLength;
   public static final double footChamferY = 0.2 * footWidth;

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
