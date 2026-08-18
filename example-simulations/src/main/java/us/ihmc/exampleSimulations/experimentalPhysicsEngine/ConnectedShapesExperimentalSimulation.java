package us.ihmc.exampleSimulations.experimentalPhysicsEngine;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.scs2.SimulationConstructionSet2;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.geometry.Box3DDefinition;
import us.ihmc.scs2.definition.robot.RevoluteJointDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.robot.SixDoFJointDefinition;
import us.ihmc.scs2.definition.state.OneDoFJointState;
import us.ihmc.scs2.definition.state.SixDoFJointState;
import us.ihmc.scs2.definition.terrain.TerrainObjectDefinition;
import us.ihmc.scs2.definition.visual.*;
import us.ihmc.scs2.examples.simulations.ExampleExperimentalSimulationTools;
import us.ihmc.scs2.simulation.impulseBased.physicsEngine.parameters.ContactParameters;
import us.ihmc.scs2.simulation.PhysicsEngineFactories;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class ConnectedShapesExperimentalSimulation
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   public ConnectedShapesExperimentalSimulation()
   {
      ContactParameters contactParameters = new ContactParameters();
      contactParameters.setMinimumPenetration(5.0e-5);
      contactParameters.setCoefficientOfFriction(0.7);
      contactParameters.setErrorReductionParameter(0.001);
      contactParameters.setCoefficientOfRestitution(0.5);
      contactParameters.setRestitutionThreshold(0.15);

      Vector3D boxSize1 = new Vector3D(0.5, 0.3, 0.3);
      double boxMass1 = 1.0;
      double radiusOfGyrationPercent = 0.8;
      ColorDefinition boxApp1 = ColorDefinitions.LightSeaGreen();

      Vector3D boxSize2 = new Vector3D(0.3, 0.3, 0.3);
      double boxMass2 = 1.0;
      ColorDefinition boxApp2 = ColorDefinitions.Teal();

      Vector3D connectionOffset = new Vector3D(0.9, 0.0, 0.0);

      RobotDefinition robotDefinition = new RobotDefinition("ConnectedShapes");
      RigidBodyDefinition rootBodyDefinition = new RigidBodyDefinition("rootBody");
      SixDoFJointDefinition rootJointDefinition = new SixDoFJointDefinition("rootJoint");
      rootBodyDefinition.addChildJoint(rootJointDefinition);
      RigidBodyDefinition rigidBody1 = ExampleExperimentalSimulationTools.newBoxRigidBody("box1", boxSize1, boxMass1, radiusOfGyrationPercent, boxApp1);
      rootJointDefinition.setSuccessor(rigidBody1);

      RevoluteJointDefinition pinJoint = new RevoluteJointDefinition("pin");
      pinJoint.setAxis(Axis3D.Y);
      RigidBodyDefinition rigidBody2 = ExampleExperimentalSimulationTools.newBoxRigidBody("box2", boxSize2, boxMass2, radiusOfGyrationPercent, boxApp2);
      rigidBody2.setCenterOfMassOffset(connectionOffset);
      VisualDefinitionFactory factory2 = new VisualDefinitionFactory();
      factory2.appendTranslation(0.5 * connectionOffset.getX(), 0, 0);
      factory2.appendRotation(0.5 * Math.PI, Axis3D.Y);
      factory2.addCylinder(connectionOffset.getX(), 0.02, ColorDefinitions.Chocolate());
      rigidBody2.getVisualDefinitions().forEach(visual -> visual.getOriginPose().prependTranslation(connectionOffset));
      rigidBody2.addVisualDefinitions(factory2.getVisualDefinitions());
      pinJoint.setSuccessor(rigidBody2);
      rigidBody1.addChildJoint(pinJoint);

      robotDefinition.setRootBodyDefinition(rootBodyDefinition);

      SixDoFJointState initialRootJointState = new SixDoFJointState(null, new Point3D(0.0, 0.0, boxSize1.getZ()));
      rootJointDefinition.setInitialJointState(initialRootJointState);
      OneDoFJointState initialPinJointState = new OneDoFJointState();
      initialPinJointState.setEffort(3.0);
      pinJoint.setInitialJointState(initialPinJointState);

      rigidBody1.addCollisionShapeDefinition(new CollisionShapeDefinition(new Box3DDefinition(boxSize1)));
      rigidBody2.addCollisionShapeDefinition(new CollisionShapeDefinition(new RigidBodyTransform(new Quaternion(), connectionOffset),
                                                                          new Box3DDefinition(boxSize2)));

      RigidBodyTransform terrainPose = new RigidBodyTransform();
      terrainPose.getTranslation().subZ(0.05);
      Box3DDefinition terrainGeometry = new Box3DDefinition(5.0, 5.0, 0.1);
      TerrainObjectDefinition terrain = new TerrainObjectDefinition(new VisualDefinition(terrainPose,
                                                                                         terrainGeometry,
                                                                                         new MaterialDefinition(ColorDefinitions.DarkKhaki())),
                                                                    new CollisionShapeDefinition(terrainPose, terrainGeometry));

      SimulationConstructionSet2 scs = new SimulationConstructionSet2("simulation", PhysicsEngineFactories.newImpulseBasedPhysicsEngineFactory(contactParameters));
      scs.addTerrainObject(terrain);
      scs.setDT(1e-6);
      scs.setBufferRecordTickPeriod(100);
      scs.addRobot(robotDefinition);
      scs.startSimulationThread();
   }

   public static void main(String[] args)
   {
      new ConnectedShapesExperimentalSimulation();
   }
}
