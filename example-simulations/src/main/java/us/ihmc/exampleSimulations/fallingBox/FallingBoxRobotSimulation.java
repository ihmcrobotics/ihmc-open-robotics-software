package us.ihmc.exampleSimulations.fallingBox;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.GroundContactModel;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.RobotFromDescription;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.physics.collision.HybridImpulseSpringDamperCollisionHandler;
import us.ihmc.simulationconstructionset.physics.collision.simple.CollisionManager;
import us.ihmc.simulationconstructionset.util.LinearGroundContactModel;

public class FallingBoxRobotSimulation
{
   private static final Box3D box = new Box3D(0.5, 0.3, 0.8);
   private static final double mass = 10.0;

   private static final double height = 0.5;
   private static final double width = 1.0;

   private static final boolean initialVelocity = false;
   private static final boolean initialOrientation = true;
   
   private static final boolean testTorque = false;

   public FallingBoxRobotSimulation()
   {
      double dt = 0.001;

      FlatGroundEnvironment environment = new FlatGroundEnvironment();
      
      BoxRobotDescription gcpRobotDescription = new BoxRobotDescription("gcpRobot", box, mass, true);
      BoxRobotDescription csRobotDescription = new BoxRobotDescription("csRobot", box, mass, false);
      BoxRobotCollisionMeshDefinitionDataHolder collisionMeshData = new BoxRobotCollisionMeshDefinitionDataHolder(box);
      csRobotDescription.addCollisionMeshDefinitionData(collisionMeshData);

      Robot gcpRobot = new RobotFromDescription(gcpRobotDescription);
      Robot csRobot = new RobotFromDescription(csRobotDescription);

      GroundContactModel groundModel = new LinearGroundContactModel(gcpRobot, 1422, 15.6, 125.0, 300.0, gcpRobot.getRobotsYoVariableRegistry()); // same value with DRCSCSInitialSetup.
      groundModel.setGroundProfile3D(environment.getTerrainObject3D());
      gcpRobot.setGroundContactModel(groundModel);

      FloatingJoint gcpFloatingJoint = (FloatingJoint) gcpRobot.getRootJoints().get(0);
      gcpFloatingJoint.setPosition(0.0, 0.5 * width, height);

      FloatingJoint csFloatingJoint = (FloatingJoint) csRobot.getRootJoints().get(0);
      csFloatingJoint.setPosition(0.0, -0.5 * width, height);

      if (initialVelocity)
      {
         gcpFloatingJoint.setVelocity(1.0, 0.0, 0.0);
         csFloatingJoint.setVelocity(1.0, 0.0, 0.0);
      }

      if (initialOrientation)
      {
         gcpFloatingJoint.setYawPitchRoll(0.0, 0.05 * Math.PI, 0.0);
         csFloatingJoint.setYawPitchRoll(0.0, 0.05 * Math.PI, 0.0);
      }

      GroundContactPointBasedEstimator gcpEstimator = new GroundContactPointBasedEstimator(gcpRobot);
      gcpRobot.setController(gcpEstimator);
      CollisionShapeBasedEstimator csEstimator = new CollisionShapeBasedEstimator(csRobot);
      csRobot.setController(csEstimator);
      
      List<Robot> allSimulatedRobotList = new ArrayList<Robot>();
      allSimulatedRobotList.add(gcpRobot);
      allSimulatedRobotList.add(csRobot);

      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setCreateGUI(true);
      parameters.setDataBufferSize(8000);

      SimulationConstructionSet scs = new SimulationConstructionSet(allSimulatedRobotList.toArray(new Robot[0]), parameters);
      Graphics3DObject staticLinkGraphics = new Graphics3DObject();
      staticLinkGraphics.addCoordinateSystem(0.1);
      scs.addStaticLinkGraphics(staticLinkGraphics);
      scs.addStaticLinkGraphics(environment.getTerrainObject3D().getLinkGraphics());

      double coefficientOfRestitution = 0.2;
      double coefficientOfFriction = 0.7;
      HybridImpulseSpringDamperCollisionHandler collisionHandler = new HybridImpulseSpringDamperCollisionHandler(coefficientOfRestitution,
                                                                                                                 coefficientOfFriction, scs.getRootRegistry(),
                                                                                                                 new YoGraphicsListRegistry());

      CollisionManager collisionManager = new CollisionManager(environment.getTerrainObject3D(), collisionHandler);
      scs.initializeShapeCollision(collisionManager);

      scs.setDT(dt, 1);
      scs.setFastSimulate(true);
      scs.setGroundVisible(false);

      scs.setCameraFix(0.0, 0.0, 0.8);
      scs.setCameraPosition(0.0, -8.0, 1.4);
      scs.startOnAThread();
   }

   public static void main(String[] args)
   {
      new FallingBoxRobotSimulation();
   }
}
