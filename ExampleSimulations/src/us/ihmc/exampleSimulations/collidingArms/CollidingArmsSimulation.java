package us.ihmc.exampleSimulations.collidingArms;

import java.util.ArrayList;

import javax.vecmath.Vector3d;

import us.ihmc.exampleSimulations.newtonsCradle.GroundAsABoxRobot;
import us.ihmc.exampleSimulations.newtonsCradle.PileOfRandomObjectsRobot;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.robotics.robotDescription.CollisionMasksHelper;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.physics.CollisionHandler;
import us.ihmc.simulationconstructionset.physics.collision.DefaultCollisionHandler;
import us.ihmc.simulationconstructionset.physics.visualize.DefaultCollisionVisualizer;

public class CollidingArmsSimulation
{

   public CollidingArmsSimulation()
   {
      double dt = 0.001;

      Vector3d offsetOne = new Vector3d(-0.8, 0.0, 0.0);
      Vector3d offsetTwo = new Vector3d(0.8, 0.0, 0.0);

      CollidingArmRobotDescription armOneDescription = new CollidingArmRobotDescription("ArmOne", offsetOne);
      CollidingArmRobotDescription armTwoDescription = new CollidingArmRobotDescription("ArmTwo", offsetTwo);

      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setCreateGUI(true);
      parameters.setDataBufferSize(8000);

      SimulationConstructionSet scs = new SimulationConstructionSet(parameters);

      Graphics3DObject staticLinkGraphics = new Graphics3DObject();
      staticLinkGraphics.addCoordinateSystem(0.1);
      scs.addStaticLinkGraphics(staticLinkGraphics);

      CollisionMasksHelper helper = new CollisionMasksHelper();

      helper.addCollisionGroup("ArmOne", armOneDescription.getArmCollisionGroup());
      helper.addCollisionGroup("ArmTwo", armTwoDescription.getArmCollisionGroup());

      helper.addCollisionGroup("BaseOne", armOneDescription.getBaseCollisionGroup());
      helper.addCollisionGroup("BaseTwo", armTwoDescription.getBaseCollisionGroup());

      helper.addCollisionGroup("UpperArmOne", armOneDescription.getUpperArmCollisionGroup());
      helper.addCollisionGroup("UpperArmTwo", armTwoDescription.getUpperArmCollisionGroup());

      helper.addCollisionGroup("LowerArmOne", armOneDescription.getLowerArmCollisionGroup());
      helper.addCollisionGroup("LowerArmTwo", armTwoDescription.getLowerArmCollisionGroup());

      helper.setToCollideWithGroup("ArmOne", "ArmTwo");
      helper.setToCollideWithGroup("LowerArmOne", "BaseOne");
      helper.setToCollideWithGroup("LowerArmTwo", "BaseTwo");

      helper.setToCollideWithGroup("UpperArmOne", "BaseOne");
      helper.setToCollideWithGroup("UpperArmTwo", "BaseTwo");

      Robot armOne = scs.addRobot(armOneDescription);
      Robot armTwo = scs.addRobot(armTwoDescription);

      CollidingArmController controllerOne = new CollidingArmController(armOne, dt);
      CollidingArmController controllerTwo = new CollidingArmController(armTwo, dt);

      armOne.setController(controllerOne);
      armTwo.setController(controllerTwo);

      int nextGroupBitMask = helper.getNextGroupBitMask();

      helper.addToCollisionMasks("LowerArmOne", nextGroupBitMask);
      helper.addToCollisionMasks("LowerArmTwo", nextGroupBitMask);

      //      armOne.setGravity(0.0);
      //      armTwo.setGravity(0.0);

      PileOfRandomObjectsRobot pileOfRandomObjectsRobot = new PileOfRandomObjectsRobot();
      pileOfRandomObjectsRobot.setSizeScale(2.0);
      pileOfRandomObjectsRobot.setGroupAndCollisionMask(0xffff, 0xffff);
      pileOfRandomObjectsRobot.setNumberOfObjects(100);
      pileOfRandomObjectsRobot.setXYExtents(1.0, 1.0);
      pileOfRandomObjectsRobot.setZMinAndMax(1.0, 2.0);
      ArrayList<Robot> robots = pileOfRandomObjectsRobot.createAndGetRobots();
      for (Robot robot : robots)
      {
         scs.addRobot(robot);
      }

      int estimatedNumberOfContactPoints = 100;
      Robot groundRobot = new GroundAsABoxRobot(estimatedNumberOfContactPoints , false, nextGroupBitMask, 0xff);
      scs.addRobot(groundRobot);

      scs.setGroundVisible(false);
      //      SingleBallRobotDescription ball = new SingleBallRobotDescription("ballOne", 0.1, 0.1);
      //      Robot ballRobot = scs.addRobot(ball);
      //      FloatingJoint ballJoint = (FloatingJoint) ballRobot.getRootJoints().get(0);
      //      ballJoint.setPosition(-0.601, 0.0, 2.4);
      //      ballRobot.setGravity(0.0);
      //      
      //      SingleCylinderRobotDescription cylinder = new SingleCylinderRobotDescription("cylinderOne", 0.1, 0.2, 0.05);
      //      Robot cylinderRobot = scs.addRobot(cylinder);
      //      FloatingJoint cylinderJoint = (FloatingJoint) cylinderRobot.getRootJoints().get(0);
      //      cylinderJoint.setPosition(-0.601, 0.0, 2.1);
      //      cylinderRobot.setGravity(0.0);

      PinJoint shoulderOne = (PinJoint) armOne.getRootJoints().get(0).getChildrenJoints().get(0);
      shoulderOne.setInitialState(0.1, 0.0);

      PinJoint shoulderTwo = (PinJoint) armTwo.getRootJoints().get(0).getChildrenJoints().get(0);
      shoulderTwo.setInitialState(-0.1, 0.0);

      DefaultCollisionVisualizer collisionVisualizer = new DefaultCollisionVisualizer(100.0, 100.0, 0.01, scs, 1000);
      //      DefaultCollisionVisualizer collisionVisualizer = null;

      double coefficientOfRestitution = 0.3;
      double coefficientOfFriction = 0.7;
      CollisionHandler collisionHandler = new DefaultCollisionHandler(coefficientOfRestitution, coefficientOfFriction);
      scs.initializeCollisionDetectionAndHandling(collisionVisualizer, collisionHandler);

      scs.setDT(dt, 1);
      scs.setFastSimulate(true);
      //      scs.setGroundVisible(false);

      scs.setCameraFix(0.0, 0.0, 0.8);
      scs.setCameraPosition(0.0, -8.0, 1.4);
      scs.startOnAThread();

      //      scs.simulate(2.0);
   }

   public static void main(String[] args)
   {
      new CollidingArmsSimulation();
   }
}
