package us.ihmc.exampleSimulations.experimentalPhysicsEngine;

import us.ihmc.euclid.referenceFrame.FrameBox3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.scs2.SimulationConstructionSet2;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.geometry.Box3DDefinition;
import us.ihmc.scs2.definition.geometry.GeometryDefinition;
import us.ihmc.scs2.definition.robot.JointDefinition;
import us.ihmc.scs2.definition.state.JointState;
import us.ihmc.scs2.definition.state.SixDoFJointState;
import us.ihmc.scs2.definition.state.interfaces.JointStateBasics;
import us.ihmc.scs2.definition.terrain.TerrainObjectDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;
import us.ihmc.scs2.examples.simulations.BoxRobotDefinition;
import us.ihmc.scs2.simulation.collision.Collidable;
import us.ihmc.scs2.simulation.collision.CollidableHelper;
import us.ihmc.scs2.simulation.impulseBased.physicsEngine.parameters.ContactParameters;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.simulation.impulseBased.physicsEngine.MultiBodySystemStateWriter;
import us.ihmc.scs2.simulation.PhysicsEngineFactories;
import us.ihmc.scs2.examples.simulations.ExampleExperimentalSimulationTools;
import us.ihmc.simulationConstructionSetTools.tools.TerrainObjectDefinitionTools;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;

public class BoxTeeteringEdgeToEdgeExperimentalSimulation
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final ContactParameters contactParameters = new ContactParameters();

   public BoxTeeteringEdgeToEdgeExperimentalSimulation()
   {
      contactParameters.setMinimumPenetration(5.0e-5);
      contactParameters.setCoefficientOfFriction(0.7);
      contactParameters.setErrorReductionParameter(0.001);

      double boxXLength = 0.2;
      double boxYWidth = 0.12;
      double boxZHeight = 0.4;
      double boxMass = 1.0;
      double boxRadiusOfGyrationPercent = 0.8;

      double initialBoxRoll = -Math.PI / 64.0;
      double initialVelocity = 0.0;

      double groundWidth = 1.0;
      double groundLength = 1.0;

      RobotDefinition boxRobot = ExampleExperimentalSimulationTools.newBoxRobot("box",
                                                                                boxXLength,
                                                                                boxYWidth,
                                                                                boxZHeight,
                                                                                boxMass,
                                                                                boxRadiusOfGyrationPercent,
                                                                                ColorDefinitions.DarkCyan());
      boxRobot.getRigidBodyDefinition("boxRigidBody")
              .addCollisionShapeDefinition(new CollisionShapeDefinition(new Box3DDefinition(boxXLength, boxYWidth, boxZHeight)));

      SixDoFJointState initialJointState = new SixDoFJointState(new YawPitchRoll(0, 0, initialBoxRoll),
                                                                new Point3D(0.0,
                                                                            groundWidth / 2.0 - 0.002,
                                                                            boxZHeight / 2.0 * 1.05 + boxYWidth / 2.0 * Math.sin(Math.abs(initialBoxRoll))));
      initialJointState.setVelocity(null, new Vector3D(initialVelocity, 0, 0));
      boxRobot.getRootJointDefinitions().get(0).setInitialJointState(initialJointState);

      GeometryDefinition terrainGeometry = new Box3DDefinition(groundLength, groundWidth, 0.1);
      RigidBodyTransform terrainPose = new RigidBodyTransform(new Quaternion(), new Vector3D(0.0, 0.0, -0.05));
      TerrainObjectDefinition terrain = new TerrainObjectDefinition(new VisualDefinition(terrainPose,
                                                                                         terrainGeometry,
                                                                                         new MaterialDefinition(ColorDefinitions.DarkKhaki())),
                                                                    new CollisionShapeDefinition(terrainPose, terrainGeometry));

      double simDT = 0.0001;

      SimulationConstructionSet2 scs = new SimulationConstructionSet2("simulation", PhysicsEngineFactories.newImpulseBasedPhysicsEngineFactory(contactParameters));
      scs.getSimulationSession().setGravity(0.0, 0.0, -9.81);
      scs.addTerrainObject(terrain);
      scs.addRobot(boxRobot);
      scs.setDT(simDT);
      scs.startSimulationThread();
      scs.simulate(1.5);
   }

   public static void main(String[] args)
   {
      new BoxTeeteringEdgeToEdgeExperimentalSimulation();
   }
}
