package us.ihmc.valkyrie.parameters;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class ValkyriePhysicalProperties
{
   private static final double defaultFootsizeReduction = 0.01;

   private static final double defaultAnkleHeight = 0.09; // Should be 0.075 + 0.015 (sole thickness)
   private static final double defaultFootLength = 0.25 - defaultFootsizeReduction;
   private static final double defaultFootBack = 0.073 - defaultFootsizeReduction / 2.0;
   private static final double defaultFootForward = defaultFootLength - defaultFootBack;
   private static final double defaultFootWidth = 0.15 - defaultFootsizeReduction;

   private static final double defaultThighLength = 0.431;
   private static final double defaultShinLength = 0.406;

   private final double ankleHeight, footLength, footBack, footForward, footWidth, thighLength, shinLength;

   private final SideDependentList<RigidBodyTransform> soleToAnkleFrameTransforms = new SideDependentList<>();
   private final SideDependentList<RigidBodyTransform> handControlFrameToWristTransforms = new SideDependentList<>();
   private final SideDependentList<RigidBodyTransform> handControlFrameToArmMassSimTransforms = new SideDependentList<>();

   private final double modelSizeScale;
   private final double modelMassScalePower;

   public ValkyriePhysicalProperties()
   {
      this(1.0, 1.0);
   }

   public ValkyriePhysicalProperties(double modelSizeScale, double modelMassScale)
   {
      this.modelSizeScale = modelSizeScale;
      modelMassScalePower = modelSizeScale != 1.0 ? Math.log(modelMassScale) / Math.log(modelSizeScale) : 1.0;

      ankleHeight = modelSizeScale * defaultAnkleHeight;
      footLength = modelSizeScale * defaultFootLength;
      footBack = modelSizeScale * defaultFootBack;
      footForward = modelSizeScale * defaultFootForward;
      footWidth = modelSizeScale * defaultFootWidth;
      thighLength = modelSizeScale * defaultThighLength;
      shinLength = modelSizeScale * defaultShinLength;

      for (RobotSide side : RobotSide.values)
      {
         RigidBodyTransform soleToAnkleFrame = new RigidBodyTransform();
         //         soleToAnkleFrame.setEuler(new Vector3d(0.0, +0.13, 0.0));
         soleToAnkleFrame.getTranslation().set(new Vector3D(footLength / 2.0 - footBack, 0.0, -ankleHeight));
         soleToAnkleFrameTransforms.put(side, soleToAnkleFrame);
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBodyTransform controlFrameToWristTransform = new RigidBodyTransform();
         controlFrameToWristTransform.getTranslation().set(0.025, robotSide.negateIfRightSide(0.07), 0.0);
         controlFrameToWristTransform.getTranslation().scale(modelSizeScale);
         controlFrameToWristTransform.appendYawRotation(robotSide.negateIfRightSide(Math.PI * 0.5));
         handControlFrameToWristTransforms.put(robotSide, controlFrameToWristTransform);

         RigidBodyTransform controlFrameToArmMassSimTransform = new RigidBodyTransform();
         controlFrameToArmMassSimTransform.getTranslation().set(-0.0275, robotSide.negateIfRightSide(0.4), 0.0);
         controlFrameToArmMassSimTransform.getTranslation().scale(modelSizeScale);
         controlFrameToArmMassSimTransform.appendYawRotation(robotSide.negateIfRightSide(0.5 * Math.PI));
         controlFrameToArmMassSimTransform.appendRollRotation(robotSide.negateIfRightSide(0.5 * Math.PI));
         handControlFrameToArmMassSimTransforms.put(robotSide, controlFrameToArmMassSimTransform);
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

   public double getFootLength()
   {
      return footLength;
   }

   public double getFootBack()
   {
      return footBack;
   }

   public double getFootForward()
   {
      return footForward;
   }

   public double getFootWidth()
   {
      return footWidth;
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

   public RigidBodyTransform getHandControlFrameToArmMassSimTransform(RobotSide side)
   {
      return handControlFrameToArmMassSimTransforms.get(side);
   }
}
