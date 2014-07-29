package us.ihmc.valkyrie.paramaters;

import javax.media.j3d.Transform3D;
import javax.vecmath.Vector3d;

import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotPhysicalProperties;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;

public class ValkyriePhysicalProperties extends DRCRobotPhysicalProperties
{
   public static final double ankleHeight = 0.082;
   public static final double footLength = 0.25;
   public static final double footBack = 0.058;
   public static final double footForward = footLength - footBack;
   public static final double footWidth = 0.15;
   public static final double thighLength = 0.431;
   public static final double shinLength = 0.406;
   public static final double pelvisToFoot = 0.887 + 0.3;

   public static final SideDependentList<Transform3D> soleToAnkleFrameTransforms = new SideDependentList<>();
   public static final SideDependentList<Transform3D> handControlFrameToWristTransforms = new SideDependentList<Transform3D>(new Transform3D(), new Transform3D());

   static
   {
      for (RobotSide side : RobotSide.values)
      {
         Transform3D soleToAnkleFrame = new Transform3D();
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

   public static Transform3D getAnkleToSoleFrameTransform(RobotSide side)
   {
      return soleToAnkleFrameTransforms.get(side);
   }
}
