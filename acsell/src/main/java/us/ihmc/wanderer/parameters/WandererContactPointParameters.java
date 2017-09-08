package us.ihmc.wanderer.parameters;

import static us.ihmc.wanderer.parameters.WandererPhysicalProperties.footLength;
import static us.ihmc.wanderer.parameters.WandererPhysicalProperties.footWidth;
import static us.ihmc.wanderer.parameters.WandererPhysicalProperties.toeWidth;

import us.ihmc.simulationconstructionset.util.LinearGroundContactModel;
import us.ihmc.wholeBodyController.DRCRobotJointMap;
import us.ihmc.wholeBodyController.RobotContactPointParameters;

public class WandererContactPointParameters extends RobotContactPointParameters
{
   public WandererContactPointParameters(DRCRobotJointMap jointMap)
   {
      super(jointMap, toeWidth, footWidth, footLength, WandererPhysicalProperties.soleToAnkleFrameTransforms);

      createDefaultFootContactPoints();
   }

   @Override
   public void setupGroundContactModelParameters(LinearGroundContactModel linearGroundContactModel)
   {
      linearGroundContactModel.setZStiffness(1500.0);
      linearGroundContactModel.setZDamping(750.0);
      linearGroundContactModel.setXYStiffness(25000.0);
      linearGroundContactModel.setXYDamping(750.0);
   }
}
