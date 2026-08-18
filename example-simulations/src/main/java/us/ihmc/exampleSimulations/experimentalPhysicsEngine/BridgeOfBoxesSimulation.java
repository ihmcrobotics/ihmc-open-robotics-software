package us.ihmc.exampleSimulations.experimentalPhysicsEngine;

import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.AffineTransform;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemBasics;
import us.ihmc.mecano.spatial.SpatialVector;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.scs2.SimulationConstructionSet2;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.controller.ControllerInput;
import us.ihmc.scs2.definition.controller.ControllerOutput;
import us.ihmc.scs2.definition.controller.interfaces.Controller;
import us.ihmc.scs2.definition.geometry.Box3DDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.state.SixDoFJointState;
import us.ihmc.scs2.definition.terrain.TerrainObjectDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;
import us.ihmc.scs2.simulation.impulseBased.physicsEngine.parameters.ContactParameters;
import us.ihmc.scs2.simulation.PhysicsEngineFactories;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoseUsingYawPitchRoll;
import us.ihmc.yoVariables.registry.YoRegistry;

import us.ihmc.scs2.examples.simulations.ExampleExperimentalSimulationTools;


import java.util.ArrayList;
import java.util.List;

import static us.ihmc.scs2.examples.simulations.ExampleExperimentalSimulationTools.newBoxRobot;

public class BridgeOfBoxesSimulation
{
   private static final String BRIDGE_NAME = "Bridge";
   private static final String RIGHT_SUPPORT = "RightSupport";
   private static final String LEFT_SUPPORT = "LeftSupport";
   private static final boolean SUPPORT_STATIC = false;

   public BridgeOfBoxesSimulation()
   {
      ContactParameters contactParameters = new ContactParameters();
      contactParameters.setMinimumPenetration(5.0e-5);
      contactParameters.setCoefficientOfFriction(0.7);
      contactParameters.setCoefficientOfRestitution(0.0);
      contactParameters.setRestitutionThreshold(0.0);
      contactParameters.setErrorReductionParameter(0.1);

      List<RobotDefinition> robots = new ArrayList<>();

      Vector3D bridgeSize = new Vector3D(3.0, 0.3, 0.2);
      double bridgeMass = 5.0;
      Vector3D supportSize = new Vector3D(0.5, 0.5, 0.5);
      double supportMass = 5.0;
      RobotDefinition bridgeRobot = newBoxRobot(BRIDGE_NAME, bridgeSize, bridgeMass, 0.8, ColorDefinitions.Aquamarine());
      robots.add(bridgeRobot);

      double bridgeHeight = 1.0;

      SixDoFJointState initialJointState = new SixDoFJointState(null, new Point3D(0, 0, bridgeHeight + 0.5 * bridgeSize.getZ() - 5.0e-4));
      bridgeRobot.getRootJointDefinitions().get(0).setInitialJointState(initialJointState);

      bridgeRobot.getRigidBodyDefinition(BRIDGE_NAME + "RigidBody").addCollisionShapeDefinition(new CollisionShapeDefinition(new Box3DDefinition(bridgeSize)));

      YoRegistry registry = new YoRegistry("controllerInputs");

      YoFramePoseUsingYawPitchRoll leftSupportPose = createSupportPose(true, registry, bridgeSize, bridgeHeight, supportSize);
      YoFramePoseUsingYawPitchRoll rightSupportPose = createSupportPose(false, registry, bridgeSize, bridgeHeight, supportSize);

      TerrainObjectDefinition terrainObjectDefinition = new TerrainObjectDefinition();

      if (SUPPORT_STATIC)
      {
         terrainObjectDefinition.addCollisionShapeDefinition(new CollisionShapeDefinition(new RigidBodyTransform(leftSupportPose.getOrientation(),
                                                                                                                 leftSupportPose.getPosition()),
                                                                                          new Box3DDefinition(supportSize)));
         terrainObjectDefinition.addVisualDefinition(new VisualDefinition(new AffineTransform(leftSupportPose.getOrientation(), leftSupportPose.getPosition()),
                                                                          new Box3DDefinition(supportSize),
                                                                          new MaterialDefinition(ColorDefinitions.Thistle())));
         terrainObjectDefinition.addCollisionShapeDefinition(new CollisionShapeDefinition(new RigidBodyTransform(rightSupportPose.getOrientation(),
                                                                                                                 rightSupportPose.getPosition()),
                                                                                          new Box3DDefinition(supportSize)));
         terrainObjectDefinition.addVisualDefinition(new VisualDefinition(new AffineTransform(rightSupportPose.getOrientation(),
                                                                                              rightSupportPose.getPosition()),
                                                                          new Box3DDefinition(supportSize),
                                                                          new MaterialDefinition(ColorDefinitions.Thistle())));
      }
      else
      {
         // Left support
         String supportName = LEFT_SUPPORT;
         RobotDefinition leftSupportRobot = newBoxRobot(supportName, supportSize, supportMass, 0.8, ColorDefinitions.Crimson());
         leftSupportRobot.getRootJointDefinitions().get(0)
                         .setInitialJointState(new SixDoFJointState(leftSupportPose.getOrientation(), leftSupportPose.getPosition()));
         leftSupportRobot.getRigidBodyDefinition(supportName + "RigidBody")
                         .addCollisionShapeDefinition(new CollisionShapeDefinition(new Box3DDefinition(supportSize)));
         robots.add(leftSupportRobot);

         // Right support
         supportName = RIGHT_SUPPORT;
         RobotDefinition rightSupportRobot = newBoxRobot(supportName, supportSize, supportMass, 0.8, ColorDefinitions.Crimson());
         rightSupportRobot.getRootJointDefinitions().get(0)
                          .setInitialJointState(new SixDoFJointState(rightSupportPose.getOrientation(), rightSupportPose.getPosition()));
         rightSupportRobot.getRigidBodyDefinition(supportName + "RigidBody")
                          .addCollisionShapeDefinition(new CollisionShapeDefinition(new Box3DDefinition(supportSize)));
         robots.add(rightSupportRobot);

         // controllers
         leftSupportRobot.addControllerDefinition((in, out) -> newSupportController(true, leftSupportPose, in, out, supportMass, bridgeMass));
         rightSupportRobot.addControllerDefinition((in, out) -> newSupportController(false, rightSupportPose, in, out, supportMass, bridgeMass));
      }
      double simDT = 0.0001;

      SimulationConstructionSet2 scs = new SimulationConstructionSet2("simulation", PhysicsEngineFactories.newImpulseBasedPhysicsEngineFactory(contactParameters));
      robots.forEach(scs::addRobot);
      scs.addTerrainObject(terrainObjectDefinition);
      scs.getRootRegistry().addChild(registry);
      scs.setDT(simDT);
      scs.startSimulationThread();
      scs.simulate(2.0);

   }

   private YoFramePoseUsingYawPitchRoll createSupportPose(boolean leftSide, YoRegistry registry, Tuple3DReadOnly bridgeSize, double bridgeHeight,
                                                          Tuple3DReadOnly supportSize)
   {
      String prefix = leftSide ? "left" : "right";
      YoFramePoseUsingYawPitchRoll pose = new YoFramePoseUsingYawPitchRoll(prefix + "SupportPose", ReferenceFrame.getWorldFrame(), registry);
      double sign = leftSide ? -1.0 : 1.0;
      pose.setPitch(sign * (Math.toRadians(45.0)));
      pose.getPosition().set(sign * (0.3 * bridgeSize.getX()), 0.0, bridgeHeight);
      pose.appendTranslation(0.5 * sign * (supportSize.getX()), 0.0, -0.5 * supportSize.getZ());
      return pose;
   }

   private Controller newSupportController(boolean leftSide, Pose3DReadOnly desiredSupportPose, ControllerInput controllerInput,
                                           ControllerOutput controllerOutput, double supportMass, double bridgeMass)
   {
      MultiBodySystemBasics controllerRobot = controllerInput.createCopy(ReferenceFrame.getWorldFrame());
      FloatingJointBasics joint = (FloatingJointBasics) controllerRobot.getJointsToConsider().get(0);

      double kp = 1000.0;
      double kd = 200.0;
      double frequency = 1.0;
      double amplitude = 0.20;

      SpatialVector proportionalTerm = new SpatialVector();
      SpatialVector derivativeTerm = new SpatialVector();
      SpatialVector gravityTerm = new SpatialVector();
      FramePose3D errorPose = new FramePose3D();

      return () ->
      {
         controllerInput.readState(controllerRobot);
         controllerRobot.getRootBody().updateFramesRecursively();

         errorPose.setIncludingFrame(ReferenceFrame.getWorldFrame(), desiredSupportPose);

         double offset = amplitude * Math.sin(2.0 * Math.PI * frequency * controllerInput.getTime());
         errorPose.getPosition().addX(leftSide ? -offset : offset);

         errorPose.changeFrame(joint.getFrameAfterJoint());

         proportionalTerm.setToZero(joint.getFrameAfterJoint());
         proportionalTerm.getLinearPart().set(errorPose.getPosition());
         errorPose.getOrientation().getRotationVector(proportionalTerm.getAngularPart());
         proportionalTerm.scale(kp);

         derivativeTerm.setIncludingFrame(joint.getJointTwist());
         derivativeTerm.scale(-kd);

         gravityTerm.setToZero(ReferenceFrame.getWorldFrame());
         gravityTerm.setLinearPartZ(9.81 * (supportMass + 0.5 * bridgeMass));
         gravityTerm.changeFrame(joint.getFrameAfterJoint());

         joint.getJointWrench().set(proportionalTerm);
         joint.getJointWrench().add(derivativeTerm);
         joint.getJointWrench().add(gravityTerm);

         controllerOutput.getJointOutput(joint).setEffort(joint);
      };
   }

   public static void main(String[] args)
   {
      new BridgeOfBoxesSimulation();
   }
}
