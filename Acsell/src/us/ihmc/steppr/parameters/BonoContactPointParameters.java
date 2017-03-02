package us.ihmc.steppr.parameters;

import static us.ihmc.steppr.parameters.BonoPhysicalProperties.footLength;
import static us.ihmc.steppr.parameters.BonoPhysicalProperties.footWidth;
import static us.ihmc.steppr.parameters.BonoPhysicalProperties.soleToAnkleFrameTransforms;
import static us.ihmc.steppr.parameters.BonoPhysicalProperties.toeWidth;

import us.ihmc.simulationconstructionset.util.LinearGroundContactModel;
import us.ihmc.wholeBodyController.DRCRobotJointMap;
import us.ihmc.wholeBodyController.RobotContactPointParameters;

public class BonoContactPointParameters extends RobotContactPointParameters
{
   public BonoContactPointParameters(DRCRobotJointMap jointMap)
   {
      super(jointMap, toeWidth, footWidth, footLength, soleToAnkleFrameTransforms);

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
