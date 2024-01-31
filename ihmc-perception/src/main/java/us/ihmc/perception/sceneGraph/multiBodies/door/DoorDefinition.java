package us.ihmc.perception.sceneGraph.multiBodies.door;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.scs2.definition.robot.*;
import us.ihmc.scs2.definition.state.OneDoFJointState;
import us.ihmc.scs2.definition.state.SixDoFJointState;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.scs2.simulation.robot.multiBodySystem.SimPrismaticJoint;
import us.ihmc.scs2.simulation.robot.multiBodySystem.SimRevoluteJoint;

import static us.ihmc.perception.sceneGraph.multiBodies.door.DoorModelParameters.*;

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
   private OneDoFJointState initialBoltState;

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
                    .add(-DOOR_PANEL_THICKNESS, DOOR_PANEL_HINGE_OFFSET, DOOR_PANEL_GROUND_GAP_HEIGHT);
      initialHingeState = new OneDoFJointState();
      doorHingeJoint.setInitialJointState(initialHingeState);
      double hingeJointLimit = 1.7;
      doorHingeJoint.setPositionLowerLimit(-hingeJointLimit);
      doorHingeJoint.setPositionUpperLimit(hingeJointLimit);

      doorPanelDefinition.build();
      doorHingeJoint.setSuccessor(doorPanelDefinition);

      RevoluteJointDefinition doorLeverJoint = new RevoluteJointDefinition("doorLeverJoint");
      doorLeverJoint.setAxis(Axis3D.X);
      doorPanelDefinition.addChildJoint(doorLeverJoint);
      doorLeverJoint.getTransformToParent().getTranslation().add(0.0,
                                                                 DOOR_PANEL_WIDTH - DOOR_OPENER_INSET,
                                                                 DOOR_OPENER_FROM_BOTTOM_OF_PANEL);
      initialLeverState = new OneDoFJointState();
      doorLeverJoint.setInitialJointState(initialLeverState);

      DoorLeverHandleDefinition doorLeverHandleDefinition = new DoorLeverHandleDefinition();
      doorLeverJoint.setSuccessor(doorLeverHandleDefinition);

      PrismaticJointDefinition doorBoltJoint = new PrismaticJointDefinition("doorBoltJoint");
      doorBoltJoint.setAxis(Axis3D.Y);
      doorBoltJoint.setPositionLimits(-DOOR_BOLT_TRAVEL, 0.0);
      doorPanelDefinition.addChildJoint(doorBoltJoint);
      doorBoltJoint.getTransformToParent().getTranslation().add(0.01,
                                                                DOOR_PANEL_WIDTH + DOOR_BOLT_TRAVEL / 2.0,
                                                                DOOR_OPENER_FROM_BOTTOM_OF_PANEL);
      initialBoltState = new OneDoFJointState();
      doorBoltJoint.setInitialJointState(initialBoltState);

      DoorBoltDefinition doorBoltDefinition = new DoorBoltDefinition();
      doorBoltJoint.setSuccessor(doorBoltDefinition);

      setRootBodyDefinition(rootBodyDefinition);
   }

   public static void applyPDController(Robot robot)
   {
      SimRevoluteJoint doorLeverJoint = (SimRevoluteJoint) robot.getJoint("doorLeverJoint");
      SimPrismaticJoint doorBoltJoint = (SimPrismaticJoint) robot.getJoint("doorBoltJoint");

      // TODO: Make this happen at simulation rate instead of control rate
      robot.getControllerManager().addController(() ->
      {
         double p = 2.0;
         double d = 1.0;

         double errorQ = doorLeverJoint.getQ();
         double errorQd = doorLeverJoint.getQd();

         doorLeverJoint.setTau(-p * errorQ - d * errorQd);

         double angleOfFullPull = 0.4 * Math.PI / 2.0;
         double pullAmountQ = -Math.abs(doorLeverJoint.getQ()) * DOOR_BOLT_TRAVEL / angleOfFullPull;

         double boltFromPullAmountQ = doorBoltJoint.getQ() - pullAmountQ;
         if (boltFromPullAmountQ < 0.0)
            boltFromPullAmountQ = 0.0;

         d = 5.0;

         double springK = 8.0;
         double mechanismK = 50.0;
         errorQ = springK * doorBoltJoint.getQ() + mechanismK * boltFromPullAmountQ;
         errorQd = doorBoltJoint.getQd();

         doorBoltJoint.setTau(-errorQ - d * errorQd);
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
