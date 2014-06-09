package us.ihmc.atlas.parameters;

import javax.media.j3d.Transform3D;
import javax.vecmath.Vector3d;

import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotPhysicalProperties;
import us.ihmc.utilities.math.geometry.TransformTools;

public class AtlasPhysicalProperties extends DRCRobotPhysicalProperties
{
   public static final double ankleHeight = 0.084;
   public static final double pelvisToFoot = 0.887;

   // this transform is probably in the wrong direction.  The ankle is above the sole,  so the origin of the ankle should have a positive z value
   // from the point of view of the sole.  With this transform it will have a negative value -- PA
   public static final double footWidth = 0.12;    // 0.08;   //0.124887;
   public static final double toeWidth = 0.095;    // 0.07;   //0.05;   //
   public static final double footLength = 0.255;
   public static final double footBack = 0.09;    // 0.06;   //0.082;    // 0.07;
   public static final double footStartToetaperFromBack = 0.195;
   public static final double footForward = footLength - footBack;    // 0.16;   //0.178;    // 0.18;
   public static final Transform3D ankleToSoleFrameTransform = TransformTools.createTranslationTransform(new Vector3d(footLength / 2.0 - footBack, 0.0, -ankleHeight));
   public static final double shinLength = 0.374;
   public static final double thighLength = 0.422;

   @Override
   public double getAnkleHeight()
   {
      return ankleHeight;
   }
}
