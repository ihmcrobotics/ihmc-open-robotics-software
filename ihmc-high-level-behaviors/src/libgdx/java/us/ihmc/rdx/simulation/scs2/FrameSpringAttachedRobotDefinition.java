package us.ihmc.rdx.simulation.scs2;

import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.robot.SixDoFJointDefinition;

/**
 * WIP
 * @deprecated Might not need this!
 */
public class FrameSpringAttachedRobotDefinition extends RobotDefinition
{
   public FrameSpringAttachedRobotDefinition()
   {
      RigidBodyDefinition rootBodyDefinition = new RigidBodyDefinition("rootBody");
      SixDoFJointDefinition rootJointDefinition = new SixDoFJointDefinition("rootJoint");
      rootBodyDefinition.addChildJoint(rootJointDefinition);

      setRootBodyDefinition(rootBodyDefinition);
   }
}
