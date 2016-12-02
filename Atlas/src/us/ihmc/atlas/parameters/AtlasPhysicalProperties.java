package us.ihmc.atlas.parameters;

import us.ihmc.avatar.drcRobot.DRCRobotPhysicalProperties;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.geometry.TransformTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class AtlasPhysicalProperties extends DRCRobotPhysicalProperties
{
   private double modelScale = 1.0;
   private double massScalePower = 3.0;
   
   private final double ankleHeight ;
   private final double pelvisToFoot;
   private final double footWidthForControl ;
   private final double toeWidthForControl ;
   private final double footLengthForControl;
   private final double footBackForControl ;
   private final double actualFootWidth ;
   private final double actualFootLength;
   private final double footStartToetaperFromBack;
   private final double footForward; // 0.16;   //0.178;    // 0.18;
   private final double shinLength;
   private final double thighLength;

   private final SideDependentList<RigidBodyTransform> soleToAnkleFrameTransforms = new SideDependentList<>();
   private final SideDependentList<RigidBodyTransform> handAttachmentPlateToWristTransforms = new SideDependentList<RigidBodyTransform>();

   public AtlasPhysicalProperties()
   {
      ankleHeight = modelScale * 0.084;                                                                      
      pelvisToFoot = modelScale * 0.887;                                                                     
      footWidthForControl = modelScale * 0.11; //0.12; // 0.08;   //0.124887;                                
      toeWidthForControl = modelScale * 0.085; //0.095; // 0.07;   //0.05;   //                              
      footLengthForControl = modelScale * 0.22; //0.255;                                                     
      footBackForControl = modelScale * 0.085; // 0.09; // 0.06;   //0.082;    // 0.07;                      
      actualFootWidth = modelScale * 0.138;                                                                  
      actualFootLength = modelScale * 0.26;                                                                  
      footStartToetaperFromBack = modelScale * 0.195;                                                        
      footForward = footLengthForControl - footBackForControl; // 0.16;   //0.178;    // 0.18;  
      shinLength = modelScale * 0.374;                                                                       
      thighLength = modelScale * 0.422;                                                                      
                                                                                                
      // TODO Auto-generated constructor stub
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

   public double getPelvisToFoot()
   {
      return pelvisToFoot;
   }

   public double getFootWidthForControl()
   {
      return footWidthForControl;
   }

   public double getToeWidthForControl()
   {
      return toeWidthForControl;
   }

   public double getFootLengthForControl()
   {
      return footLengthForControl;
   }

   public double getFootBackForControl()
   {
      return footBackForControl;
   }

   public double getActualFootWidth()
   {
      return actualFootWidth;
   }

   public double getActualFootLength()
   {
      return actualFootLength;
   }

   public double getFootStartToetaperFromBack()
   {
      return footStartToetaperFromBack;
   }

   public double getFootForward()
   {
      return footForward;
   }

   public double getShinLength()
   {
      return shinLength;
   }

   public double getThighLength()
   {
      return thighLength;
   }

   public SideDependentList<RigidBodyTransform> getSoleToAnkleFrameTransforms()
   {
      return soleToAnkleFrameTransforms;
   }

   public SideDependentList<RigidBodyTransform> getHandAttachmentPlateToWristTransforms()
   {
      return handAttachmentPlateToWristTransforms;
   }

   public double getModelScale()
   {
      return modelScale;
   }
   
   public double getMassScalePower()
   {
      return massScalePower;
   }

   public void setModelScale(double modelScale)
   {
      this.modelScale = modelScale;
   }

   public void setMassScalePower(double massScalePower)
   {
      this.massScalePower = massScalePower;
   }
   
   
}
