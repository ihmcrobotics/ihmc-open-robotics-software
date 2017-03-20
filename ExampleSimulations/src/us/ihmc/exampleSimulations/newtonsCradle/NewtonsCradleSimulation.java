package us.ihmc.exampleSimulations.newtonsCradle;

import java.util.ArrayList;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.exampleSimulations.collidingArms.SingleBallRobotDescription;
import us.ihmc.exampleSimulations.collidingArms.SingleBoxRobotDescription;
import us.ihmc.exampleSimulations.collidingArms.SingleCapsuleRobotDescription;
import us.ihmc.exampleSimulations.collidingArms.SingleCylinderRobotDescription;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.shapes.Box3d;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.RobotFromDescription;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.physics.CollisionHandler;
import us.ihmc.simulationconstructionset.physics.collision.DefaultCollisionVisualizer;
import us.ihmc.simulationconstructionset.physics.collision.HybridImpulseSpringDamperCollisionHandler;
import us.ihmc.simulationconstructionset.util.LinearStickSlipGroundContactModel;
import us.ihmc.simulationconstructionset.util.ground.CombinedTerrainObject3D;
import us.ihmc.tools.thread.ThreadTools;

public class NewtonsCradleSimulation
{
   private static CollisionHandler createCollisionHandler(double coefficientOfRestitution, double coefficientOfFriction, YoVariableRegistry registry,
                                                          YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      //      CollisionHandler collisionHandler =  new DefaultCollisionHandler(coefficientOfRestitution, coefficientOfFriction);
      CollisionHandler collisionHandler = new HybridImpulseSpringDamperCollisionHandler(coefficientOfRestitution, coefficientOfFriction, registry,
                                                                                        yoGraphicsListRegistry);

      //      CollisionHandler collisionHandler = new SpringCollisionHandler(2.0, 1.1, 1.1, robot.getRobotsYoVariableRegistry());
      //      CollisionHandler collisionHandler = new SpringCollisionHandler(1, 1000, 10.0, robot.getRobotsYoVariableRegistry());

      return collisionHandler;
   }

   public static void createNewtonsCradleSimulation()
   {
      NewtonsCradleRobot robot = new NewtonsCradleRobot();

      SimulationConstructionSet scs = new SimulationConstructionSet(robot);
      scs.setDT(0.0001, 100);

      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      CollisionHandler collisionHandler = createCollisionHandler(0.99, 0.15, scs.getRootRegistry(), yoGraphicsListRegistry);

      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);
      DefaultCollisionVisualizer collisionVisualizer = new DefaultCollisionVisualizer(4.0, 4.0, 0.01, scs, 100);

      scs.initializeCollisionDetectionAndHandling(collisionVisualizer, collisionHandler);
      scs.startOnAThread();
   }

   public static void createSpinningCoinSimulation()
   {
      SpinningCoinRobot spinningCoinRobot = new SpinningCoinRobot();
      ArrayList<Robot> robots = spinningCoinRobot.getRobots();

      int estimatedNumberOfContactPoints = 32;
      GroundAsABoxRobot groundAsABoxRobot = new GroundAsABoxRobot();
      groundAsABoxRobot.setEstimatedNumberOfContactPoints(estimatedNumberOfContactPoints);
      groundAsABoxRobot.setAddWalls(false);
      robots.add(groundAsABoxRobot.createRobot());

      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters(10000);

      Robot[] robotArray = new Robot[robots.size()];
      robots.toArray(robotArray);

      SimulationConstructionSet scs = new SimulationConstructionSet(robotArray, parameters);
      scs.setDT(0.0001, 1);
      scs.setGroundVisible(false);

      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

      double epsilon = 0.3;
      double mu = 0.7;
      CollisionHandler collisionHandler = createCollisionHandler(epsilon, mu, scs.getRootRegistry(), yoGraphicsListRegistry);
      DefaultCollisionVisualizer collisionVisualizer = new DefaultCollisionVisualizer(10.0, 10.0, 0.01, scs, 100);

      scs.initializeCollisionDetectionAndHandling(collisionVisualizer, collisionHandler);
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);

      scs.startOnAThread();

      scs.setSimulateDuration(60.0);
      scs.simulate();
   }

   public static void createStackOfBouncyBallsSimulation()
   {
      ArrayList<Robot> robots = new ArrayList<>();

      // Set to true for a large stack of same sized balls for object stacking evaluations.
      boolean stackOfSameSizedBalls = false;

      int numberOfBalls = 4;
      double radiusScaleFactor = 0.6;
      double massScaleFactor = 0.2;

      if (stackOfSameSizedBalls)
      {
         numberOfBalls = 40;
         radiusScaleFactor = 1.0;
         massScaleFactor = 1.0;
      }

      StackOfBouncyBallsRobot robot = new StackOfBouncyBallsRobot(numberOfBalls, radiusScaleFactor, massScaleFactor);
      robots.add(robot);

      int estimatedNumberOfContactPoints = 4;
      GroundAsABoxRobot groundAsABoxRobot = new GroundAsABoxRobot();
      groundAsABoxRobot.setEstimatedNumberOfContactPoints(estimatedNumberOfContactPoints);
      groundAsABoxRobot.setAddWalls(false);
      robots.add(groundAsABoxRobot.createRobot());

      Robot[] robotArray = new Robot[robots.size()];
      robots.toArray(robotArray);

      SimulationConstructionSet scs = new SimulationConstructionSet(robotArray);
      scs.setDT(0.0001, 100);
      scs.setGroundVisible(false);

      DefaultCollisionVisualizer collisionVisualizer = new DefaultCollisionVisualizer(0.1, 0.1, 0.01, scs, 100);
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

      double coefficientOfRestitution = 0.9;
      double coefficientOfFriction = 0.0;
      CollisionHandler collisionHandler = createCollisionHandler(coefficientOfRestitution, coefficientOfFriction, scs.getRootRegistry(),
                                                                 yoGraphicsListRegistry);
      //      CollisionHandler collisionHandler = new SpringCollisionHandler(2.0, 1.1, 1.1, robot.getRobotsYoVariableRegistry());
      //      CollisionHandler collisionHandler = new SpringCollisionHandler(1, 1000, 10.0, robot.getRobotsYoVariableRegistry());
      //      CollisionHandler collisionHandler = new DefaultCollisionHandler(0.98, 0.1, robot);
      //      CollisionHandler collisionHandler = new DefaultCollisionHandler(0.3, 0.7, robot);

      scs.initializeCollisionDetectionAndHandling(collisionVisualizer, collisionHandler);
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);

      scs.startOnAThread();
   }

   public static void createBoxDownRampSimulation()
   {
      double coefficientOfRestitution = 0.3;
      double coefficientOfFriction = 0.4;

      ArrayList<Robot> robots = new ArrayList<>();
      double mass = 1.0;
      double xLength = 0.2;
      double yWidth = 0.2;
      double zHeight = 0.1;
      SingleBoxRobotDescription singleBoxRobotDescription = new SingleBoxRobotDescription();
      singleBoxRobotDescription.setName("BoxOne");
      singleBoxRobotDescription.setMass(mass);
      singleBoxRobotDescription.setCollisionGroup(0xff);
      singleBoxRobotDescription.setCollisionMask(0xff);
      singleBoxRobotDescription.setXLength(xLength);
      singleBoxRobotDescription.setYWidth(yWidth);
      singleBoxRobotDescription.setZHeight(zHeight);

      RobotFromDescription boxRobot = new RobotFromDescription(singleBoxRobotDescription.createRobotDescription());

      FloatingJoint rootJoint = (FloatingJoint) boxRobot.getRootJoints().get(0);
      rootJoint.setPosition(0.0, 0.0, 0.15);
      robots.add(boxRobot);

      double groundAngle = Math.PI / 8.0;

      int estimatedNumberOfContactPoints = 32;
      GroundAsABoxRobot groundAsABoxRobot = new GroundAsABoxRobot();
      groundAsABoxRobot.setEstimatedNumberOfContactPoints(estimatedNumberOfContactPoints);
      groundAsABoxRobot.setGroundAngle(groundAngle);
      groundAsABoxRobot.setAddWalls(false);
      groundAsABoxRobot.setCollisionGroup(0xffff);
      groundAsABoxRobot.setCollisionMask(0xffff);
      robots.add(groundAsABoxRobot.createRobot());

      Robot[] robotArray = new Robot[robots.size()];
      robots.toArray(robotArray);

      CombinedTerrainObject3D boxTerrain = new CombinedTerrainObject3D("BoxTerrain");
      Box3d box = new Box3d(2.0, 1.0, 0.1);
      box.setPosition(new Point3D(0.0, 1.0, 0.0));
      box.setYawPitchRoll(0.0, groundAngle, 0.0);
      boxTerrain.addRotatableBox(box, YoAppearance.Blue());

      LinearStickSlipGroundContactModel groundContactModel = new LinearStickSlipGroundContactModel(boxRobot, boxRobot.getRobotsYoVariableRegistry());
      groundContactModel.setGroundProfile3D(boxTerrain);

      groundContactModel.setAlphaStickSlip(coefficientOfFriction, coefficientOfFriction);
      groundContactModel.enableSlipping();
      groundContactModel.enableSurfaceNormal();
      boxRobot.setGroundContactModel(groundContactModel);

      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setDataBufferSize(32000);
      SimulationConstructionSet scs = new SimulationConstructionSet(robotArray, parameters);
      scs.setDT(0.0001, 100);
      scs.setGroundVisible(false);
      scs.addStaticLinkGraphics(boxTerrain.getLinkGraphics());

      DefaultCollisionVisualizer collisionVisualizer = new DefaultCollisionVisualizer(0.1, 0.1, 0.01, scs, 100);
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

      CollisionHandler collisionHandler = createCollisionHandler(coefficientOfRestitution, coefficientOfFriction, scs.getRootRegistry(),
                                                                 yoGraphicsListRegistry);
      //      CollisionHandler collisionHandler = new SpringCollisionHandler(2.0, 1.1, 1.1, robot.getRobotsYoVariableRegistry());
      //      CollisionHandler collisionHandler = new SpringCollisionHandler(1, 1000, 10.0, robot.getRobotsYoVariableRegistry());
      //      CollisionHandler collisionHandler = new DefaultCollisionHandler(0.98, 0.1, robot);
      //      CollisionHandler collisionHandler = new DefaultCollisionHandler(0.3, 0.7, robot);

      scs.initializeCollisionDetectionAndHandling(collisionVisualizer, collisionHandler);
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);

      scs.startOnAThread();
   }

   public static void createRollingObjectsSimulation()
   {
      ArrayList<Robot> robots = new ArrayList<>();

      double ballRadius = 0.2;
      double ballMass = 1.0;
      double ballRadiusOfGyrationPercent = 1.0;

      double cylinderRadius = 0.2;
      double cylinderHeight = 0.5;
      double cylinderMass = 1.0;
      double cylinderRadiusOfGyrationPercent = 1.0;

      double capsuleRadius = 0.2;
      double capsuleHeight = 0.1 + 2.0 * capsuleRadius;
      double capsuleMass = 1.0;
      double capsuleRadiusOfGyrationPercent = 1.0;

      double initialVelocity = 1.0;

      RobotFromDescription ballRobot = createASingleBallRobot(ballRadius, ballMass, ballRadiusOfGyrationPercent);
      robots.add(ballRobot);
      FloatingJoint ballRootJoint = (FloatingJoint) ballRobot.getRootJoints().get(0);
      ballRootJoint.setPosition(-1.0, -2.0 * ballRadius - cylinderHeight, ballRadius * 1.02);
      ballRootJoint.setVelocity(initialVelocity, 0.0, 0.0);

      RobotFromDescription cylinderRobot = createASingleCylinderRobot("cylinder", cylinderRadius, cylinderHeight, cylinderMass,
                                                                      cylinderRadiusOfGyrationPercent);
      robots.add(cylinderRobot);
      FloatingJoint cylinderRootJoint = (FloatingJoint) cylinderRobot.getRootJoints().get(0);
      cylinderRootJoint.setPosition(-1.0, 0.0, cylinderRadius * 1.02);
      cylinderRootJoint.setVelocity(initialVelocity, 0.0, 0.0);
      cylinderRootJoint.setYawPitchRoll(0.0, 0.0, Math.PI / 2.0);

      RobotFromDescription capsuleRobot = createASingleCapsuleRobot(capsuleRadius, capsuleHeight, capsuleMass, capsuleRadiusOfGyrationPercent);
      robots.add(capsuleRobot);
      FloatingJoint capsuleRootJoint = (FloatingJoint) capsuleRobot.getRootJoints().get(0);
      capsuleRootJoint.setPosition(-1.0, cylinderHeight / 2.0 + capsuleHeight / 2.0 + capsuleRadius, capsuleRadius * 1.02);
      capsuleRootJoint.setVelocity(initialVelocity, 0.0, 0.0);
      capsuleRootJoint.setYawPitchRoll(0.0, 0.0, Math.PI / 2.00);

      int estimatedNumberOfContactPoints = 30;
      double groundAngle = 0.0;

      GroundAsABoxRobot groundAsABoxRobot = new GroundAsABoxRobot();
      groundAsABoxRobot.setEstimatedNumberOfContactPoints(estimatedNumberOfContactPoints);
      groundAsABoxRobot.setGroundAngle(groundAngle);
      groundAsABoxRobot.setAddWalls(false);
      groundAsABoxRobot.setCollisionGroup(0xffff);
      groundAsABoxRobot.setCollisionMask(0xffff);
      robots.add(groundAsABoxRobot.createRobot());

      Robot[] robotArray = new Robot[robots.size()];
      robots.toArray(robotArray);

      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setDataBufferSize(8000);
      SimulationConstructionSet scs = new SimulationConstructionSet(robotArray, parameters);
      scs.setDT(0.0001, 10);
      scs.setFastSimulate(true);
      scs.setGroundVisible(false);

      DefaultCollisionVisualizer collisionVisualizer = new DefaultCollisionVisualizer(50.0, 100.0, 0.003, scs, 100);
      //      DefaultCollisionVisualizer collisionVisualizer = null;
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

      double coefficientOfRestitution = 0.3;
      double coefficientOfFriction = 0.1;//0.7;
      CollisionHandler collisionHandler = createCollisionHandler(coefficientOfRestitution, coefficientOfFriction, scs.getRootRegistry(),
                                                                 yoGraphicsListRegistry);

      scs.initializeCollisionDetectionAndHandling(collisionVisualizer, collisionHandler);
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);

      scs.startOnAThread();
   }

   public static void createTeeteringEdgeToEdgeContactSimulation()
   {
      ArrayList<Robot> robots = new ArrayList<>();

      double boxXLength = 0.2;
      double boxYWidth = 0.12;
      double boxZHeight = 0.4;
      double boxMass = 1.0;
      double boxRadiusOfGyrationPercent = 0.8;

      double initialBoxRoll = -Math.PI / 64.0;
      double initialVelocity = 0.0;

      double groundWidth = 1.0;
      double groundLength = 1.0;

      RobotFromDescription boxRobot = createASingleBoxRobot(boxXLength, boxYWidth, boxZHeight, boxMass, boxRadiusOfGyrationPercent);
      robots.add(boxRobot);
      FloatingJoint boxRootJoint = (FloatingJoint) boxRobot.getRootJoints().get(0);
      boxRootJoint.setPosition(0.0, groundWidth/2.0 - 0.002, boxZHeight / 2.0 * 1.05 + boxYWidth / 2.0 * Math.sin(Math.abs(initialBoxRoll)));
      boxRootJoint.setVelocity(initialVelocity, 0.0, 0.0);
      boxRootJoint.setYawPitchRoll(0.0, 0.0, initialBoxRoll);
      boxRootJoint.setAngularVelocityInBody(new Vector3D(0.0, 0.0, 0.0));

      int estimatedNumberOfContactPoints = 30;
      double groundAngle = 0.0;

      GroundAsABoxRobot groundAsABoxRobot = new GroundAsABoxRobot();
      groundAsABoxRobot.setEstimatedNumberOfContactPoints(estimatedNumberOfContactPoints);
      groundAsABoxRobot.setGroundAngle(groundAngle);
      groundAsABoxRobot.setAddWalls(false);

      groundAsABoxRobot.setGroundLength(groundLength);
      groundAsABoxRobot.setGroundWidth(groundWidth);

      groundAsABoxRobot.setCollisionGroup(0xffff);
      groundAsABoxRobot.setCollisionMask(0xffff);

      robots.add(groundAsABoxRobot.createRobot());

      Robot[] robotArray = new Robot[robots.size()];
      robots.toArray(robotArray);

      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setDataBufferSize(8000);
      SimulationConstructionSet scs = new SimulationConstructionSet(robotArray, parameters);
      scs.setDT(0.0001, 10);
      scs.setFastSimulate(true);
      scs.setGroundVisible(false);

      DefaultCollisionVisualizer collisionVisualizer = new DefaultCollisionVisualizer(50.0, 100.0, 0.003, scs, 100);
      //      DefaultCollisionVisualizer collisionVisualizer = null;
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

      double coefficientOfRestitution = 0.3;
      double coefficientOfFriction = 0.4;
      CollisionHandler collisionHandler = createCollisionHandler(coefficientOfRestitution, coefficientOfFriction, scs.getRootRegistry(),
                                                                 yoGraphicsListRegistry);

      scs.initializeCollisionDetectionAndHandling(collisionVisualizer, collisionHandler);
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);

      scs.startOnAThread();
   }

   public static void createSpinningAndDroppingObjectsSimulation()
   {
      ArrayList<Robot> robots = new ArrayList<>();

      double ballRadius = 0.2;
      double ballMass = 1.0;
      double ballRadiusOfGyrationPercent = 1.0;

      double cylinderRadius = 0.25;
      double cylinderHeight = 0.1;
      double cylinderMass = 1.0;
      double cylinderRadiusOfGyrationPercent = 1.0;

      double capsuleSegmentHeight = 0.05;//0.2;//0.3;
      double capsuleRadius = 0.2;
      double capsuleHeight = capsuleSegmentHeight + 2.0 * capsuleRadius;
      double capsuleMass = 1.0;
      double capsuleRadiusOfGyrationPercent = 0.8;

      double boxXLength = 0.2;
      double boxYWidth = capsuleSegmentHeight;
      double boxZHeight = 2.0 * capsuleRadius;
      double boxMass = 1.0;
      double boxRadiusOfGyrationPercent = 0.8;

      double initialPitch = -Math.PI / 8.0;
      double initialCapsuleRoll = -Math.PI / 8.0;
      double initialBoxRoll = -Math.PI / 64.0;
      double initialVelocity = 0.0;
      double initialCylinderSpinRotationalVelocity = 3.0;
      double initialRotationalVelocity = 6.0;

      RobotFromDescription ballRobot = createASingleBallRobot(ballRadius, ballMass, ballRadiusOfGyrationPercent);
      robots.add(ballRobot);
      FloatingJoint ballRootJoint = (FloatingJoint) ballRobot.getRootJoints().get(0);
      ballRootJoint.setPosition(-1.0, -3.0 * ballRadius - cylinderHeight, ballRadius * 1.02);
      ballRootJoint.setVelocity(initialVelocity, 0.0, 0.0);
      ballRootJoint.setAngularVelocityInBody(new Vector3D(0.0, 0.0, initialRotationalVelocity));

      RobotFromDescription cylinderRobot = createASingleCylinderRobot("cylinder", cylinderRadius, cylinderHeight, cylinderMass,
                                                                      cylinderRadiusOfGyrationPercent);
      robots.add(cylinderRobot);
      FloatingJoint cylinderRootJoint = (FloatingJoint) cylinderRobot.getRootJoints().get(0);
      cylinderRootJoint.setPosition(-1.0, 0.0, cylinderHeight / 2.0 * 1.05 + cylinderRadius * Math.abs(Math.sin(initialPitch)));
      cylinderRootJoint.setVelocity(initialVelocity, 0.0, 0.0);
      cylinderRootJoint.setYawPitchRoll(0.0, initialPitch, 0.0);
      cylinderRootJoint.setAngularVelocityInBody(new Vector3D(initialCylinderSpinRotationalVelocity, 0.0, 0.0));

//      RobotFromDescription cylinderRobotTwo = createASingleCylinderRobot("cylinderTwo", cylinderRadius, cylinderHeight, cylinderMass,
//                                                                         cylinderRadiusOfGyrationPercent);
//      robots.add(cylinderRobotTwo);
//      FloatingJoint cylinderRootJointTwo = (FloatingJoint) cylinderRobotTwo.getRootJoints().get(0);
//      cylinderRootJointTwo.setPosition(0.0, 0.0, cylinderHeight / 2.0 * 1.05 + cylinderRadius * Math.abs(Math.sin(initialPitch)));
//      cylinderRootJointTwo.setVelocity(initialVelocity, 0.0, 0.0);
//      cylinderRootJointTwo.setYawPitchRoll(0.0, 0.0, 0.0);
//      cylinderRootJointTwo.setAngularVelocityInBody(new Vector3D(0.0, 0.0, initialRotationalVelocity));
//
//      RobotFromDescription cylinderRobotThree = createASingleCylinderRobot("cylinderThree", cylinderRadius, cylinderHeight, cylinderMass,
//                                                                           cylinderRadiusOfGyrationPercent);
//      robots.add(cylinderRobotThree);
//      FloatingJoint cylinderRootJointThree = (FloatingJoint) cylinderRobotThree.getRootJoints().get(0);
//      cylinderRootJointThree.setPosition(0.0, -cylinderRadius * 3.0, cylinderHeight / 2.0 * 1.05 + cylinderRadius * Math.abs(Math.sin(initialPitch)));
//      cylinderRootJointThree.setVelocity(initialVelocity, 0.0, 0.0);
//      cylinderRootJointThree.setYawPitchRoll(0.0, initialPitch, 0.0);
//      cylinderRootJointThree.setAngularVelocityInBody(new Vector3D(0.0, 0.0, 0.0));
//
      RobotFromDescription capsuleRobot = createASingleCapsuleRobot(capsuleRadius, capsuleHeight, capsuleMass, capsuleRadiusOfGyrationPercent);
      robots.add(capsuleRobot);
      FloatingJoint capsuleRootJoint = (FloatingJoint) capsuleRobot.getRootJoints().get(0);
      capsuleRootJoint.setPosition(-1.0, cylinderRadius + capsuleHeight / 2.0 + capsuleRadius,
                                   capsuleRadius * 1.02 + capsuleSegmentHeight / 2.0 * Math.sin(Math.abs(initialCapsuleRoll)));
      capsuleRootJoint.setVelocity(initialVelocity, 0.0, 0.0);
      capsuleRootJoint.setYawPitchRoll(0.0, 0.0, Math.PI / 2.0 + initialCapsuleRoll);
      capsuleRootJoint.setAngularVelocityInBody(new Vector3D(0.0, 0.0, 0.0));

//      RobotFromDescription boxRobot = createASingleBoxRobot(boxXLength, boxYWidth, boxZHeight, boxMass, boxRadiusOfGyrationPercent);
//      robots.add(boxRobot);
//      FloatingJoint boxRootJoint = (FloatingJoint) boxRobot.getRootJoints().get(0);
//      boxRootJoint.setPosition(0.0, 2.0 * cylinderRadius + boxYWidth, boxZHeight / 2.0 * 1.05 + boxYWidth / 2.0 * Math.sin(Math.abs(initialBoxRoll)));
//      boxRootJoint.setVelocity(initialVelocity, 0.0, 0.0);
//      boxRootJoint.setYawPitchRoll(0.0, 0.0, initialBoxRoll);
//      boxRootJoint.setAngularVelocityInBody(new Vector3D(0.0, 0.0, 0.0));

      int estimatedNumberOfContactPoints = 100;
      double groundAngle = 0.0;

      GroundAsABoxRobot groundAsABoxRobot = new GroundAsABoxRobot();
      groundAsABoxRobot.setEstimatedNumberOfContactPoints(estimatedNumberOfContactPoints);
      groundAsABoxRobot.setGroundAngle(groundAngle);
      groundAsABoxRobot.setAddWalls(false);
      groundAsABoxRobot.setCollisionGroup(0xffff);
      groundAsABoxRobot.setCollisionMask(0xffff);

      robots.add(groundAsABoxRobot.createRobot());

      Robot[] robotArray = new Robot[robots.size()];
      robots.toArray(robotArray);

      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setDataBufferSize(8000);
      SimulationConstructionSet scs = new SimulationConstructionSet(robotArray, parameters);
      scs.setDT(0.0001, 100);
      scs.setFastSimulate(true);
      scs.setGroundVisible(false);

      DefaultCollisionVisualizer collisionVisualizer = new DefaultCollisionVisualizer(50.0, 100.0, 0.003, scs, 100);
      //      DefaultCollisionVisualizer collisionVisualizer = null;
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

      double coefficientOfRestitution = 0.3;
      double coefficientOfFriction = 0.4;
      CollisionHandler collisionHandler = createCollisionHandler(coefficientOfRestitution, coefficientOfFriction, scs.getRootRegistry(),
                                                                 yoGraphicsListRegistry);

      scs.initializeCollisionDetectionAndHandling(collisionVisualizer, collisionHandler);
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);

      scs.startOnAThread();
   }

   private static RobotFromDescription createASingleBoxRobot(double boxXLength, double boxYWidth, double boxZHeight, double boxMass,
                                                             double boxRadiusOfGyrationPercent)
   {
      SingleBoxRobotDescription boxDescription = new SingleBoxRobotDescription();
      boxDescription.setName("box");
      boxDescription.setMass(boxMass);
      boxDescription.setXLength(boxXLength);
      boxDescription.setYWidth(boxYWidth);
      boxDescription.setZHeight(boxZHeight);
      boxDescription.setRadiusOfGyrationPercent(boxRadiusOfGyrationPercent);

      boxDescription.setCollisionGroup(0xffff);
      boxDescription.setCollisionMask(0xffff);

      AppearanceDefinition boxAppearance = YoAppearance.DarkCyan();
      //      boxAppearance.setTransparency(0.5);
      boxDescription.setAppearance(boxAppearance);

      RobotFromDescription boxRobot = new RobotFromDescription(boxDescription.createRobotDescription());
      return boxRobot;
   }

   private static RobotFromDescription createASingleCapsuleRobot(double capsuleRadius, double capsuleHeight, double capsuleMass,
                                                                 double capsuleRadiusOfGyrationPercent)
   {
      SingleCapsuleRobotDescription capsuleDescription = new SingleCapsuleRobotDescription();
      capsuleDescription.setName("capsule");
      capsuleDescription.setMass(capsuleMass);
      capsuleDescription.setRadius(capsuleRadius);
      capsuleDescription.setHeight(capsuleHeight);
      capsuleDescription.setRadiusOfGyrationPercent(capsuleRadiusOfGyrationPercent);

      capsuleDescription.setCollisionGroup(0xffff);
      capsuleDescription.setCollisionMask(0xffff);

      AppearanceDefinition capsuleAppearance = YoAppearance.DarkCyan();
      //      capsuleAppearance.setTransparency(0.5);
      capsuleDescription.setAppearance(capsuleAppearance);
      capsuleDescription.setAddStripes(true);

      capsuleAppearance = YoAppearance.Gold();
      //      capsuleAppearance.setTransparency(0.5);
      capsuleDescription.setStripeAppearance(capsuleAppearance);

      RobotFromDescription capsuleRobot = new RobotFromDescription(capsuleDescription.createRobotDescription());
      return capsuleRobot;
   }

   private static RobotFromDescription createASingleCylinderRobot(String name, double cylinderRadius, double cylinderHeight, double cylinderMass,
                                                                  double cylinderRadiusOfGyrationPercent)
   {
      SingleCylinderRobotDescription cylinderDescription = new SingleCylinderRobotDescription();
      cylinderDescription.setName(name);
      cylinderDescription.setMass(cylinderMass);
      cylinderDescription.setRadius(cylinderRadius);
      cylinderDescription.setHeight(cylinderHeight);
      cylinderDescription.setRadiusOfGyrationPercent(cylinderRadiusOfGyrationPercent);

      cylinderDescription.setCollisionGroup(0xffff);
      cylinderDescription.setCollisionMask(0xffff);
      cylinderDescription.setAppearance(YoAppearance.DarkCyan());
      cylinderDescription.setAddStripes(true);
      cylinderDescription.setStripeAppearance(YoAppearance.Gold());
      RobotFromDescription cylinderRobot = new RobotFromDescription(cylinderDescription.createRobotDescription());
      return cylinderRobot;
   }

   private static RobotFromDescription createASingleBallRobot(double ballRadius, double ballMass, double radiusOfGyrationPercent)
   {
      SingleBallRobotDescription ballDescription = new SingleBallRobotDescription();
      ballDescription.setName("ball");
      ballDescription.setMass(ballMass);
      ballDescription.setRadius(ballRadius);
      ballDescription.setRadiusOfGyrationPercent(radiusOfGyrationPercent);

      ballDescription.setCollisionGroup(0xffff);
      ballDescription.setCollisionMask(0xffff);
      ballDescription.setAppearance(YoAppearance.DarkCyan());
      ballDescription.setAddStripes(true);
      ballDescription.setStripeAppearance(YoAppearance.Gold());
      RobotFromDescription ballRobot = new RobotFromDescription(ballDescription.createRobotDescription());
      return ballRobot;
   }

   public static void createRowOfDominosSimulation()
   {
      ArrayList<Robot> robots = new ArrayList<>();
      int numberOfDominos = 30;
      boolean firstDominoFalling = false;
      RowOfDominosRobot robot = new RowOfDominosRobot(numberOfDominos, firstDominoFalling);
      robots.add(robot);

      double ballRadius = 0.02;
      double ballMass = 0.2;
      double ballRadiusOfGyrationPercent = 0.5;
      double ballInitialVelocity = 0.5;
      double ballInitialRoationalVelocity = 0.0;

      Robot ballRobot = createASingleBallRobot(ballRadius, ballMass, ballRadiusOfGyrationPercent);
      FloatingJoint ballRootJoint = (FloatingJoint) ballRobot.getRootJoints().get(0);
      ballRootJoint.setPosition(-6.0 * ballRadius, 0.0, ballRadius * 1.01);
      ballRootJoint.setVelocity(ballInitialVelocity, 0.0, 0.0);
      ballRootJoint.setAngularVelocityInBody(new Vector3D(0.0, 0.0, ballInitialRoationalVelocity));
      robots.add(ballRobot);

      int estimatedNumberOfContactPoints = 200;
      GroundAsABoxRobot groundAsABoxRobot = new GroundAsABoxRobot();
      groundAsABoxRobot.setEstimatedNumberOfContactPoints(estimatedNumberOfContactPoints);
      robots.add(groundAsABoxRobot.createRobot());

      Robot[] robotArray = new Robot[robots.size()];
      robots.toArray(robotArray);

      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setDataBufferSize(4000);
      SimulationConstructionSet scs = new SimulationConstructionSet(robotArray, parameters);
      scs.setDT(0.0002, 10);
      scs.setFastSimulate(true);
      scs.setGroundVisible(false);

      DefaultCollisionVisualizer collisionVisualizer = new DefaultCollisionVisualizer(50.0, 100.0, 0.003, scs, 100);
      //      DefaultCollisionVisualizer collisionVisualizer = null;
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

      double coefficientOfRestitution = 0.3;
      double coefficientOfFriction = 0.7;
      //      CollisionHandler handler = new SpringCollisionHandler(2.0, 1.1, 1.1, robot.getRobotsYoVariableRegistry());
      //      CollisionHandler handler = new SpringCollisionHandler(1, 1000, 10.0, robot.getRobotsYoVariableRegistry());
      //      CollisionHandler handler = new DefaultCollisionHandler(0.98, 0.1, robot);
      CollisionHandler collisionHandler = createCollisionHandler(coefficientOfRestitution, coefficientOfFriction, scs.getRootRegistry(),
                                                                 yoGraphicsListRegistry);

      scs.initializeCollisionDetectionAndHandling(collisionVisualizer, collisionHandler);
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);

      scs.startOnAThread();
   }

   public static void createStackOfBlocksSimulation()
   {
      int numberOfBlocks = 6;
      StackOfBlocksRobot stackOfBlocksRobot = new StackOfBlocksRobot(numberOfBlocks);
      ArrayList<Robot> robots = stackOfBlocksRobot.getRobots();

      int estimatedNumberOfContactPoints = 16;
      GroundAsABoxRobot groundAsABoxRobot = new GroundAsABoxRobot();
      groundAsABoxRobot.setEstimatedNumberOfContactPoints(estimatedNumberOfContactPoints);
      robots.add(groundAsABoxRobot.createRobot());

      boolean showGUI = true;

      Robot[] robotArray = new Robot[robots.size()];
      robots.toArray(robotArray);

      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setCreateGUI(showGUI);
      parameters.setDataBufferSize(32000);

      SimulationConstructionSet scs = new SimulationConstructionSet(robotArray, parameters);
      scs.setDT(0.0001, 100);
      scs.setFastSimulate(false);
      scs.setGroundVisible(false);
      scs.setSimulateDuration(2.0);

      DefaultCollisionVisualizer collisionVisualizer = new DefaultCollisionVisualizer(100.0, 100.0, 0.01, scs, 1000);
      //      DefaultCollisionVisualizer collisionVisualizer = null;
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

      double coefficientOfRestitution = 0.3;
      double coefficientOfFriction = 0.7;
      CollisionHandler collisionHandler = createCollisionHandler(coefficientOfRestitution, coefficientOfFriction, scs.getRootRegistry(),
                                                                 yoGraphicsListRegistry);
      scs.initializeCollisionDetectionAndHandling(collisionVisualizer, collisionHandler);
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);

      scs.startOnAThread();
   }

   public static void createPileOfRandomObjectsSimulation()
   {
      int numberOfObjects = 60;

      PileOfRandomObjectsRobot pileOfRandomObjectsRobot = new PileOfRandomObjectsRobot();
      pileOfRandomObjectsRobot.setNumberOfObjects(numberOfObjects);
      ArrayList<Robot> robots = pileOfRandomObjectsRobot.createAndGetRobots();

      int estimatedNumberOfContactPoints = 2 * numberOfObjects * 8;//500;
      GroundAsABoxRobot groundAsABoxRobot = new GroundAsABoxRobot();
      groundAsABoxRobot.setEstimatedNumberOfContactPoints(estimatedNumberOfContactPoints);
      groundAsABoxRobot.setAddWalls(true);
      robots.add(groundAsABoxRobot.createRobot());

      boolean showGUI = true;

      Robot[] robotArray = new Robot[robots.size()];
      robots.toArray(robotArray);

      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setDataBufferSize(4000);
      parameters.setCreateGUI(showGUI);

      SimulationConstructionSet scs = new SimulationConstructionSet(robotArray, parameters);

      //    DefaultCollisionVisualizer collisionVisualizer = new DefaultCollisionVisualizer(100.0, 100.0, scs, 1000);
      DefaultCollisionVisualizer collisionVisualizer = null;
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

      double coefficientOfRestitution = 0.3;
      double coefficientOfFriction = 0.7;
      CollisionHandler collisionHandler = createCollisionHandler(coefficientOfRestitution, coefficientOfFriction, scs.getRootRegistry(),
                                                                 yoGraphicsListRegistry);
      scs.initializeCollisionDetectionAndHandling(collisionVisualizer, collisionHandler);
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);

      scs.setDT(0.00025, 10);
      scs.setFastSimulate(true);
      scs.setGroundVisible(false);
      scs.startOnAThread();

      //      scs.simulate();

      long wallStartTime = System.currentTimeMillis();
      while (true)
      {
         ThreadTools.sleep(5000);

         double simTime = scs.getTime();
         long wallTime = System.currentTimeMillis();

         double wallTimeElapsed = ((double) (wallTime - wallStartTime)) * 0.001;
         double realTimeRate = simTime / wallTimeElapsed;
         System.out.println("Real Time Rate = " + realTimeRate);
      }
   }

   public static void main(String[] args)
   {
//                  createNewtonsCradleSimulation();
      //    createSpinningCoinSimulation();
      //            createStackOfBouncyBallsSimulation();
      //            createBoxDownRampSimulation();
//                  createRowOfDominosSimulation();
      //      createStackOfBlocksSimulation();
//            createRollingObjectsSimulation();
//            createSpinningAndDroppingObjectsSimulation();
//      createTeeteringEdgeToEdgeContactSimulation();
            createPileOfRandomObjectsSimulation();
   }

}
