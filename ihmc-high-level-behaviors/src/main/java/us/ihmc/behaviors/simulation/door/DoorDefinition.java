package us.ihmc.behaviors.simulation.door;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.scs2.definition.robot.PrismaticJointDefinition;
import us.ihmc.scs2.definition.robot.RevoluteJointDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.robot.SixDoFJointDefinition;
import us.ihmc.scs2.definition.state.OneDoFJointState;
import us.ihmc.scs2.definition.state.SixDoFJointState;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.bullet.physicsEngine.BulletMultiBodyLinkCollider;
import us.ihmc.scs2.simulation.bullet.physicsEngine.BulletPhysicsEngine;
import us.ihmc.scs2.simulation.bullet.physicsEngine.BulletRobot;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.scs2.simulation.robot.multiBodySystem.SimPrismaticJoint;
import us.ihmc.scs2.simulation.robot.multiBodySystem.SimRevoluteJoint;

import static us.ihmc.perception.sceneGraph.rigidBody.doors.DoorModelParameters.*;

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

   private final DoorFrameDefinition doorFrameDefinition;
   private final DoorPanelDefinition doorPanelDefinition;
   private final DoorBoltDefinition doorBoltDefinition;
   private final DoorLeverHandleDefinition doorLeverHandleDefinition;

   public DoorDefinition(String name)
   {
      super(name);

      doorFrameDefinition = new DoorFrameDefinition(getName() + "_doorFrame");
      doorPanelDefinition = new DoorPanelDefinition(getName() + "_doorPanel");
      doorBoltDefinition = new DoorBoltDefinition(getName() + "_doorBolt");
      doorLeverHandleDefinition = new DoorLeverHandleDefinition(getName() + "_doorLever");
   }

   public DoorFrameDefinition getDoorFrameDefinition()
   {
      return doorFrameDefinition;
   }

   public DoorPanelDefinition getDoorPanelDefinition()
   {
      return doorPanelDefinition;
   }

   public DoorBoltDefinition getDoorBoltDefinition()
   {
      return doorBoltDefinition;
   }

   public DoorLeverHandleDefinition getDoorLeverHandleDefinition()
   {
      return doorLeverHandleDefinition;
   }

   public void build()
   {
      RigidBodyDefinition rootBodyDefinition = new RigidBodyDefinition(getName() + "_doorRootBody");

      SixDoFJointDefinition rootJointDefinition = new SixDoFJointDefinition(getName() + "_doorRootJoint");
      rootBodyDefinition.addChildJoint(rootJointDefinition);
      initialSixDoFState = new SixDoFJointState(new YawPitchRoll(), new Point3D());
      initialSixDoFState.setVelocity(new Vector3D(), new Vector3D());
      rootJointDefinition.setInitialJointState(initialSixDoFState);

      rootJointDefinition.setSuccessor(doorFrameDefinition);

      RevoluteJointDefinition doorHingeJoint = new RevoluteJointDefinition(getName() + "_doorHingeJoint");
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

      RevoluteJointDefinition doorLeverJoint = new RevoluteJointDefinition(getName() + "_doorLeverJoint");
      doorLeverJoint.setAxis(Axis3D.X);
      doorPanelDefinition.addChildJoint(doorLeverJoint);
      doorLeverJoint.getTransformToParent().getTranslation().add(0.0,
                                                                 DOOR_PANEL_WIDTH - DOOR_OPENER_INSET,
                                                                 DOOR_OPENER_FROM_BOTTOM_OF_PANEL);
      initialLeverState = new OneDoFJointState();
      doorLeverJoint.setInitialJointState(initialLeverState);
      doorLeverJoint.setSuccessor(doorLeverHandleDefinition);

      PrismaticJointDefinition doorBoltJoint = new PrismaticJointDefinition(getName() + "_doorBoltJoint");
      doorBoltJoint.setAxis(Axis3D.Y);
      doorBoltJoint.setPositionLimits(-DOOR_BOLT_TRAVEL, 0.0);
      doorPanelDefinition.addChildJoint(doorBoltJoint);
      doorBoltJoint.getTransformToParent().getTranslation().add(0.01,
                                                                DOOR_PANEL_WIDTH + DOOR_BOLT_TRAVEL / 2.0,
                                                                DOOR_OPENER_FROM_BOTTOM_OF_PANEL);
      initialBoltState = new OneDoFJointState();
      doorBoltJoint.setInitialJointState(initialBoltState);

      doorBoltJoint.setSuccessor(doorBoltDefinition);

      setRootBodyDefinition(rootBodyDefinition);
   }

   public static void setupPhysics(Robot robot, SimulationSession simulationSession)
   {
      SimRevoluteJoint doorLeverJoint = (SimRevoluteJoint) robot.getJoint(robot.getName() + "_doorLeverJoint");
      SimPrismaticJoint doorBoltJoint = (SimPrismaticJoint) robot.getJoint(robot.getName() + "_doorBoltJoint");

      simulationSession.addBeforePhysicsCallback(time ->
      {
         double leverSpringK = DOOR_LEVER_MAX_TORQUE / DOOR_LEVER_MAX_TURN_ANGLE;
         double leverDamping = 1.0;

         double leverErrorQ = doorLeverJoint.getQ();
         double leverErrorQd = doorLeverJoint.getQd();

         doorLeverJoint.setTau(-leverSpringK * leverErrorQ - leverDamping * leverErrorQd);

         // Compute the bolt desired position as a proportion of the max bolt travel, the lever end stop,
         // and current lever position
         double boltDesiredQ = -Math.abs(doorLeverJoint.getQ()) * DOOR_BOLT_TRAVEL / DOOR_LEVER_MAX_TURN_ANGLE;

         // Compute the error for the hard mechanism, which we simulated as a spring damper,
         // but in real life it's a physical constraint. We remove any negative error, as this
         // strong component should only ever serve to pull the latch in the opening direction.
         double boltMechanismErrorQ = doorBoltJoint.getQ() - boltDesiredQ;
         if (boltMechanismErrorQ < 0.0)
            boltMechanismErrorQ = 0.0;

         //
         double boltSpringK = 8.0; // Light spring that tried to restore the bolt to zero position (fully extended)
         double mechanismK = 1000.0; // Strong spring that simulates a hard mechanism
         double boltKTerm = boltSpringK * doorBoltJoint.getQ() + mechanismK * boltMechanismErrorQ;
         double boltDamping = 5.0;
         double boltErrorQd = doorBoltJoint.getQd();

         doorBoltJoint.setTau(-boltKTerm - boltDamping * boltErrorQd);
      });

      if (simulationSession.getPhysicsEngine() instanceof BulletPhysicsEngine bulletPhysicsEngine)
      {
         for (BulletRobot bulletRobot : bulletPhysicsEngine.getBulletRobots())
         {
            if (bulletRobot.getName().equals(robot.getName()))
            {
               // The bolt should be slippery
               BulletMultiBodyLinkCollider boltLinkCollider = bulletRobot.getBulletMultiBodyRobot().getBulletMultiBodyLinkCollider(3);
               boltLinkCollider.setFriction(0.001);
            }
         }
      }
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
