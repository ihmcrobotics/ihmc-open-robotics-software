package us.ihmc.gdx.simulation.environment.object.objects.door;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.robot.SixDoFJointDefinition;
import us.ihmc.scs2.definition.state.SixDoFJointState;

public class DoorDefinition
{
   public DoorDefinition()
   {
      RobotDefinition robotDefinition = new RobotDefinition("door");
      RigidBodyDefinition rootBodyDefinition = new RigidBodyDefinition("rootBody");

      SixDoFJointDefinition rootJointDefinition = new SixDoFJointDefinition("rootJoint");
      rootBodyDefinition.addChildJoint(rootJointDefinition);



      robotDefinition.setRootBodyDefinition(rootBodyDefinition);

      SixDoFJointState initialRootJointState = new SixDoFJointState(new YawPitchRoll(), new Point3D(0.0, 0.0, 0.0));
      rootJointDefinition.setInitialJointState(initialRootJointState);
   }
}
