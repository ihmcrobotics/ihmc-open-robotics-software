package us.ihmc.valkyrie.torquespeedcurve;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.valkyrie.parameters.ValkyriePhysicalProperties;
import us.ihmc.wholeBodyController.DRCRobotJointMap;
import us.ihmc.wholeBodyController.RobotContactPointParameters;

public class ValkyrieMultiContactPointParameters extends RobotContactPointParameters<RobotSide>
{
   public ValkyrieMultiContactPointParameters(DRCRobotJointMap jointMap, ValkyriePhysicalProperties physicalProperties)
   {
      super(jointMap, physicalProperties.getFootWidth(), physicalProperties.getFootLength(), physicalProperties.getSoleToAnkleFrameTransforms());
   }

   public void addSingleContactPoint(String parentJointName, String bodyName, String contactName, RigidBodyTransform contactPoseInParentJoint)
   {
      additionalContactRigidBodyNames.add(bodyName);
      additionalContactNames.add(contactName);
      additionalContactTransforms.add(contactPoseInParentJoint);
      addSimulationContactPoint(parentJointName, contactPoseInParentJoint.getTranslation());
   }
}
