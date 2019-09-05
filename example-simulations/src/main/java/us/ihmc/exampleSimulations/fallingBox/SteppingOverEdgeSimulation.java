package us.ihmc.exampleSimulations.fallingBox;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.simulationConstructionSetTools.util.environments.EdgeEnvironment;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.RobotFromDescription;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.physics.collision.HybridImpulseSpringDamperCollisionHandler;
import us.ihmc.simulationconstructionset.physics.collision.simple.CollisionManager;

public class SteppingOverEdgeSimulation
{
   private static final Box3D box = new Box3D(0.7, 0.12, 0.05);
   private static final double mass = 5.0;

   private static final double initialPitchAngle = -Math.PI * 0.45;
   private static final double initialHeight = 0.35;
   private static final Vector3D initialVelocity = new Vector3D(0.0, 0.0, 0.0);

   public SteppingOverEdgeSimulation()
   {
      double dt = 0.001;

      EdgeEnvironment environment = new EdgeEnvironment();

      BoxRobotDescription robotDescription = new BoxRobotDescription("robot", box, mass, false);
      BoxRobotCollisionMeshDefinitionDataHolder collisionMeshData = new BoxRobotCollisionMeshDefinitionDataHolder(box);
      robotDescription.addCollisionMeshDefinitionData(collisionMeshData);

      Robot robot = new RobotFromDescription(robotDescription);

      FloatingJoint floatingJoint = (FloatingJoint) robot.getRootJoints().get(0);
      floatingJoint.setPosition(-0.05, 0.0, initialHeight);
      floatingJoint.setYawPitchRoll(0.0, initialPitchAngle, 0.0);
      floatingJoint.setVelocity(initialVelocity);

      List<Robot> allSimulatedRobotList = new ArrayList<Robot>();
      allSimulatedRobotList.add(robot);

      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setCreateGUI(true);
      parameters.setDataBufferSize(8000);

      SimulationConstructionSet scs = new SimulationConstructionSet(allSimulatedRobotList.toArray(new Robot[0]), parameters);
      Graphics3DObject staticLinkGraphics = new Graphics3DObject();
      staticLinkGraphics.addCoordinateSystem(0.1);
      scs.addStaticLinkGraphics(staticLinkGraphics);
      scs.addStaticLinkGraphics(environment.getTerrainObject3D().getLinkGraphics());

      double coefficientOfRestitution = 0.0;
      double coefficientOfFriction = 0.7;
      HybridImpulseSpringDamperCollisionHandler collisionHandler = new HybridImpulseSpringDamperCollisionHandler(coefficientOfRestitution,
                                                                                                                 coefficientOfFriction, scs.getRootRegistry(),
                                                                                                                 new YoGraphicsListRegistry());
      collisionHandler.setKp(100000.0);
      collisionHandler.setKd(50.0);

      CollisionManager collisionManager = new CollisionManager(environment.getTerrainObject3D(), collisionHandler);
      scs.initializeShapeCollision(collisionManager);

      scs.setDT(dt, 1);
      scs.setFastSimulate(true);
      scs.setGroundVisible(false);

      scs.setCameraFix(0.0, 0.0, 0.8);
      scs.setCameraPosition(0.0, -8.0, 0.4);
      scs.startOnAThread();
   }

   public static void main(String[] args)
   {
      new SteppingOverEdgeSimulation();
   }
}
