package us.ihmc.atlas.parameters;

import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotPhysicalProperties;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.geometry.TransformTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class AtlasPhysicalProperties extends DRCRobotPhysicalProperties
{
   public static final double scale = 1.0;
   public static final double ankleHeight = 0.084;
   public static final double pelvisToFoot = 0.887;

   public static final double footWidthForControl = scale * 0.11; //0.12; // 0.08;   //0.124887;
   public static final double toeWidthForControl = scale * 0.085; //0.095; // 0.07;   //0.05;   //
   public static final double footLengthForControl = scale * 0.22; //0.255;
   public static final double footBackForControl = scale * 0.085; // 0.09; // 0.06;   //0.082;    // 0.07;
   
   public static final double actualFootWidth = scale * 0.138;
   public static final double actualFootLength = scale * 0.26;
   
   public static final double footStartToetaperFromBack = 0.195;
   public static final double footForward = footLengthForControl - footBackForControl; // 0.16;   //0.178;    // 0.18;
   public static final double shinLength = 0.374;
   public static final double thighLength = 0.422;

   public static final SideDependentList<RigidBodyTransform> soleToAnkleFrameTransforms = new SideDependentList<>();
   public static final SideDependentList<RigidBodyTransform> handAttachmentPlateToWristTransforms = new SideDependentList<RigidBodyTransform>();

   static
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBodyTransform soleToAnkleFrame = TransformTools.createTranslationTransform(footLengthForControl / 2.0 - footBackForControl, 0.0, -ankleHeight);
         soleToAnkleFrameTransforms.put(robotSide, soleToAnkleFrame);

         double y = robotSide.negateIfRightSide(0.1);
         double yaw = robotSide.negateIfRightSide(Math.PI / 2.0);
         RigidBodyTransform handControlFrameToWristTransform = TransformTools.createTransformFromTranslationAndEulerAngles(0.0, y, 0.0, 0.0, 0.0, yaw);
         handAttachmentPlateToWristTransforms.put(robotSide, handControlFrameToWristTransform);
      }
   }

   @Override
   public double getAnkleHeight()
   {
      return ankleHeight;
   }
}
