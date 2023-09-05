package us.ihmc.perception.sceneGraph.multiBodies.door;

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

/**
 * SCS 2 definition of the door we have in the lab.
 *
 * It is a push and pull door depending on the side you're on.
 *
 * TODO: It's decently accurate, but still missing some features like:
 *   - Accurate hinge axis
 *   - Lever handle latch
 *   - Dimensions stored as JSON for different varieties
 *   - Dimensions of actual door double checked to match a sim door
 */
public class DoorDefinition extends RobotDefinition
{
   private SixDoFJointState initialSixDoFState;
   private OneDoFJointState initialHingeState;
   private OneDoFJointState initialLeverState;

   private final DoorPanelDefinition doorPanelDefinition = new DoorPanelDefinition();

   public DoorDefinition()
   {
      super("door");
   }

   public DoorPanelDefinition getDoorPanelDefinition()
   {
      return doorPanelDefinition;
   }

   public void build()
   {
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
      doorHingeJoint.getTransformToParent()
                    .getTranslation()
                    .add(0.0, DoorModelParameters.DOOR_PANEL_HINGE_OFFSET, DoorModelParameters.DOOR_PANEL_GROUND_GAP_HEIGHT);
      initialHingeState = new OneDoFJointState();
      doorHingeJoint.setInitialJointState(initialHingeState);
      doorHingeJoint.setPositionLowerLimit(-1.7);
      doorHingeJoint.setPositionUpperLimit(1.7);

      doorPanelDefinition.build();
      doorHingeJoint.setSuccessor(doorPanelDefinition);

      RevoluteJointDefinition doorLeverJoint = new RevoluteJointDefinition("doorLeverJoint");
      doorLeverJoint.setAxis(Axis3D.X);
      doorPanelDefinition.addChildJoint(doorLeverJoint);
      doorLeverJoint.getTransformToParent()
                    .getTranslation()
                    .add(-DoorModelParameters.DOOR_PANEL_THICKNESS / 2.0,
                         DoorModelParameters.DOOR_PANEL_WIDTH - DoorModelParameters.DOOR_LEVER_HANDLE_INSET,
                         DoorModelParameters.DOOR_LEVER_HANDLE_FROM_BOTTOM_OF_PANEL);
      initialLeverState = new OneDoFJointState();
      doorLeverJoint.setInitialJointState(initialLeverState);

      DoorLeverHandleDefinition doorLeverHandleDefinition = new DoorLeverHandleDefinition();
      doorLeverJoint.setSuccessor(doorLeverHandleDefinition);

      setRootBodyDefinition(rootBodyDefinition);
   }

   public static void applyPDController(Robot robot)
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
