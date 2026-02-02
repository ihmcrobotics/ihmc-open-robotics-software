package us.ihmc.zulu.parameters.model;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class ZuluPhysicalProperties
{
   private final double ANKLE_HEIGHT = 0.06;

   private final double FOOT_WIDTH_REDUCTION = 0.015;
   private final double FOOT_LENGTH_REDUCTION = 0.01;

   private final double ACTUAL_FOOT_LENGTH = 0.197;
   private final double ACTUAL_FOOT_WIDTH = 0.09;
   private final double FOOT_BACK = 0.05;
   private final double THIGH_LENGTH = 0.431;
   private final double SHIN_LENGTH = 0.406;

   private final double FOOT_FORWARD = ACTUAL_FOOT_LENGTH - FOOT_BACK;
   private final double FOOT_FORWARD_FOR_CONTROL = FOOT_FORWARD - FOOT_LENGTH_REDUCTION / 2.0;
   private final double FOOT_BACK_FOR_CONTROL = FOOT_BACK - FOOT_LENGTH_REDUCTION / 2.0;
   private final double FOOT_WIDTH_FOR_CONTROL = ACTUAL_FOOT_WIDTH - FOOT_WIDTH_REDUCTION;
   private final double FOOT_LENGTH_FOR_CONTROL = ACTUAL_FOOT_LENGTH - FOOT_LENGTH_REDUCTION;


   private final double modelSizeScale;
   private final double modelMassScalePower;
   private final double ankleHeight, footLength, footBack, footForward, footWidth, thighLength, shinLength;
   private final double footLengthForControl, footWidthForControl, footForwardForControl, footBackForControl;

   private final SideDependentList<RigidBodyTransform> soleToAnkleFrameTransforms = new SideDependentList<>();
   private final SideDependentList<RigidBodyTransform> handControlFrameToWristTransforms = new SideDependentList<>();

   public ZuluPhysicalProperties(boolean hasArms)
   {
      this(1.0, 1.0, hasArms);
   }
   public ZuluPhysicalProperties()
   {
      this(1.0, 1.0, true);
   }

   public ZuluPhysicalProperties(double modelSizeScale, double modelMassScale, boolean hasArms)
   {
      this.modelSizeScale = modelSizeScale;
      modelMassScalePower = modelSizeScale != 1.0 ? Math.log(modelMassScale) / Math.log(modelSizeScale) : 1.0;

      ankleHeight = modelSizeScale * ANKLE_HEIGHT;
      footLength = modelSizeScale * ACTUAL_FOOT_LENGTH;
      footBack = modelSizeScale * FOOT_BACK;
      footForward = modelSizeScale * FOOT_FORWARD;
      footWidth = modelSizeScale * ACTUAL_FOOT_WIDTH;
      thighLength = modelSizeScale * THIGH_LENGTH;
      shinLength = modelSizeScale * SHIN_LENGTH;
      footLengthForControl = modelSizeScale * FOOT_LENGTH_FOR_CONTROL;
      footWidthForControl = modelSizeScale * FOOT_WIDTH_FOR_CONTROL;
      footForwardForControl = modelSizeScale * FOOT_FORWARD_FOR_CONTROL;
      footBackForControl = modelSizeScale * FOOT_BACK_FOR_CONTROL;

      for (RobotSide side : RobotSide.values)
      {
         RigidBodyTransform soleToAnkleFrame = new RigidBodyTransform();
         //         soleToAnkleFrame.setEuler(new Vector3d(0.0, +0.13, 0.0));
         soleToAnkleFrame.getTranslation().set(new Vector3D(footLength / 2.0 - footBack, 0.0, -ankleHeight));
         soleToAnkleFrameTransforms.put(side, soleToAnkleFrame);
      }

      if (hasArms)
      {
         for (RobotSide robotSide : RobotSide.values)
         {
            RigidBodyTransform controlFrameToWristTransform = new RigidBodyTransform();
            controlFrameToWristTransform.getTranslation().set(0.0, 0.0, -0.075);
            controlFrameToWristTransform.getTranslation().scale(modelSizeScale);
            handControlFrameToWristTransforms.put(robotSide, controlFrameToWristTransform);
         }
      }
   }

   public double getModelSizeScale()
   {
      return modelSizeScale;
   }

   public double getModelMassScalePower()
   {
      return modelMassScalePower;
   }

   public double getAnkleHeight()
   {
      return ankleHeight;
   }

   public double getFootLengthReductionForControl()
   {
      return modelSizeScale * FOOT_LENGTH_REDUCTION;
   }

   public double getFootWidthReductionForControl()
   {
      return modelSizeScale * FOOT_WIDTH_REDUCTION;
   }

   public double getActualFootLength()
   {
      return footLength;
   }

   public double getActualFootWidth()
   {
      return footWidth;
   }

   public double getActualFootBack()
   {
      return footBack;
   }

   public double getFootLengthForControl()
   {
      return footLengthForControl;
   }

   public double getToeWidthForControl()
   {
      return footBackForControl;
   }

   public double getFootForwardForControl()
   {
      return footForwardForControl;
   }

   public double getFootBack()
   {
      return footBack;
   }

   public double getFootForward()
   {
      return footForward;
   }

   public double getFootWidthForControl()
   {
      return footWidthForControl;
   }

   public double getThighLength()
   {
      return thighLength;
   }

   public double getShinLength()
   {
      return shinLength;
   }

   public double getLegLength()
   {
      return thighLength + shinLength;
   }

   public SideDependentList<RigidBodyTransform> getSoleToAnkleFrameTransforms()
   {
      return soleToAnkleFrameTransforms;
   }

   public RigidBodyTransform getSoleToAnkleFrameTransform(RobotSide side)
   {
      return soleToAnkleFrameTransforms.get(side);
   }

   public RigidBodyTransform getHandControlFrameToWristTransform(RobotSide side)
   {
      return handControlFrameToWristTransforms.get(side);
   }
}
