package us.ihmc.gdx.simulation.environment.object.objects.door;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.scs2.definition.robot.RevoluteJointDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.robot.SixDoFJointDefinition;
import us.ihmc.scs2.definition.state.OneDoFJointState;
import us.ihmc.scs2.definition.state.SixDoFJointState;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.scs2.simulation.robot.multiBodySystem.SimRevoluteJoint;

public class DoorDefinition extends RobotDefinition
{
   private final SixDoFJointState initialSixDoFState;
   private final OneDoFJointState initialHingeState;
   private final OneDoFJointState initialLeverState;

   public DoorDefinition()
   {
      super("door");
      RigidBodyDefinition rootBodyDefinition = new RigidBodyDefinition("doorRootBody");

      SixDoFJointDefinition rootJointDefinition = new SixDoFJointDefinition("doorRootJoint");
      rootBodyDefinition.addChildJoint(rootJointDefinition);
      initialSixDoFState = new SixDoFJointState(new YawPitchRoll(), new Point3D());
      initialSixDoFState.setVelocity(new Vector3D(), new Vector3D());
      rootJointDefinition.setInitialJointState(initialSixDoFState);

      DoorFrameDefinition doorFrameDefinition = new DoorFrameDefinition();
      rootJointDefinition.setSuccessor(doorFrameDefinition);

      RevoluteJointDefinition doorHingeJoint = new RevoluteJointDefinition("doorHingeJoint");
      doorHingeJoint.setAxis(Axis3D.Z);
      doorFrameDefinition.addChildJoint(doorHingeJoint);
      doorHingeJoint.getTransformToParent().getTranslation().add(0.0, 0.005, 0.02);
      initialHingeState = new OneDoFJointState();
      doorHingeJoint.setInitialJointState(initialHingeState);

      DoorPanelDefinition doorPanelDefinition = new DoorPanelDefinition();
      doorHingeJoint.setSuccessor(doorPanelDefinition);

      RevoluteJointDefinition doorLeverJoint = new RevoluteJointDefinition("doorLeverJoint");
      doorLeverJoint.setAxis(Axis3D.X);
      doorPanelDefinition.addChildJoint(doorLeverJoint);
      doorLeverJoint.getTransformToParent().getTranslation().add(-0.0508 / 2.0, 0.9144 - 0.05, (2.0447 / 2.0) - 0.13);
      initialLeverState = new OneDoFJointState();
      doorLeverJoint.setInitialJointState(initialLeverState);

      DoorLeverHandleDefinition doorLeverHandleDefinition = new DoorLeverHandleDefinition();
      doorLeverJoint.setSuccessor(doorLeverHandleDefinition);

      setRootBodyDefinition(rootBodyDefinition);
   }

   public void applyPDController(Robot robot)
   {
      SimRevoluteJoint doorLeverJoint = (SimRevoluteJoint) robot.getJoint("doorLeverJoint");
      robot.getControllerManager().addController(() ->
      {
         double p = 2.0;
         double d = 1.0;

         double errorQ = doorLeverJoint.getQ();
         double errorQd = doorLeverJoint.getQd();

         doorLeverJoint.setTau(-p * errorQ - d * errorQd);
      });
   }

   public SixDoFJointState getInitialSixDoFState()
   {
      return initialSixDoFState;
   }

   public OneDoFJointState getInitialHingeState()
   {
      return initialHingeState;
   }
}
