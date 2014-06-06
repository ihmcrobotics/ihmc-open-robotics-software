package us.ihmc.acsell.parameters;

import javax.media.j3d.Transform3D;
import javax.vecmath.Vector3d;

import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotPhysicalProperties;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.math.geometry.TransformTools;


public class BonoPhysicalProperties extends DRCRobotPhysicalProperties
{
   public static final double ankleHeight = 0.00;
   public static final double footForward = 0.202;
   public static final double footBack = 0.05;
   public static final double footLength = footForward + footBack;
   public static final double footWidth = 0.152;
   public static final double thighLength = 0.37694;
   public static final double shinLength = 0.42164;
   public static final double legLength = thighLength + shinLength;
   public static final double pelvisToFoot = 0.887;

   public static final SideDependentList<Transform3D> ankleToSoleFrameTransforms = new SideDependentList<Transform3D>();
   static {
      for(RobotSide robotSide : RobotSide.values())
      {
         Transform3D ankleToSoleFrameTransform = TransformTools.yawPitchDegreesTransform(new Vector3d(footLength / 2.0 - footBack, 0.0, -ankleHeight), 0.0, Math.toDegrees(0.0*0.18704));
         ankleToSoleFrameTransforms.put(robotSide, ankleToSoleFrameTransform);
      }
   }
   
   @Override
   public double getAnkleHeight()
   {
      return ankleHeight;
   }
   
   public static Transform3D getAnkleToSoleFrameTransform(RobotSide side)
   {
      return ankleToSoleFrameTransforms.get(side);
   }

}
