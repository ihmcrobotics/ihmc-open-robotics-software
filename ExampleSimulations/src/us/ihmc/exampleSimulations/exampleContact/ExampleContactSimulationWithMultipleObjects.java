package us.ihmc.exampleSimulations.exampleContact;


import java.util.ArrayList;
import java.util.Random;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Vector3d;

import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.util.environments.ContactableSelectableBoxRobot;
import us.ihmc.simulationconstructionset.util.environments.ContactableSphereRobot;
import us.ihmc.simulationconstructionset.util.environments.ContactableStaticCylinderRobot;
import us.ihmc.simulationconstructionset.util.environments.ContactableToroidRobot;
import us.ihmc.simulationconstructionset.util.environments.PointMassRobot;
import us.ihmc.simulationconstructionset.util.ground.Contactable;

public class ExampleContactSimulationWithMultipleObjects
{
   private static final int numberOfSpheres = 0; //3;
   private static final int numberOfBoxes = 0; //3;
   private static final int numberOfToroids = 3;
   private static final int numberOfCylinders = 1;
   
   protected static final int NUMBER_OF_POINT_MASSES = 40;

   private final ArrayList<ExternalForcePoint> contactPoints = new ArrayList<ExternalForcePoint>();
   private final ArrayList<Contactable> contactables = new ArrayList<Contactable>();
   private final ArrayList<Robot> robots = new ArrayList<Robot>();



   public ExampleContactSimulationWithMultipleObjects()
   {
	   // Refactored simulation code from the constructor so it would be more amenable to unit testing.
   }

   private void runWithGUI()
   {
      Robot[] robotsArray = generateRobots();

      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setDataBufferSize(36000);
      SimulationConstructionSet scs = new SimulationConstructionSet(robotsArray, parameters);
      scs.setDT(0.001, 1);
      scs.setSimulateDuration(8.0);

      Graphics3DObject staticLinkGraphics = new Graphics3DObject();
      staticLinkGraphics.addCoordinateSystem(1.0);
      scs.addStaticLinkGraphics(staticLinkGraphics );
      
      scs.setGroundVisible(false);

      scs.startOnAThread();
   }

   protected Robot[] generateRobots()
   {
      double forceVectorScale = 1.0 / 50.0;

      Random random = new Random(1776L);

      for (int i = 0; i < numberOfToroids; i++)
      {
         Vector3d toroidPosition = new Vector3d(randomDouble(random, 0.0, 5.0), randomDouble(random, 0.0, 5.0), randomDouble(random, 0.2, 0.6));
         RigidBodyTransform transform3d = new RigidBodyTransform();
         
         AxisAngle4d randomRotation = RandomTools.generateRandomRotation(random);
         transform3d.setTranslation(toroidPosition);
         transform3d.setRotation(randomRotation);

         double steeringWheelRadius = 0.5;
         double steeringWheelThickness = 0.1; 
         double mass = 2.0;

         ContactableToroidRobot contactableToroidRobot = new ContactableToroidRobot("ToroidRobot" + i, transform3d, steeringWheelRadius,
               steeringWheelThickness, mass);

         //         contactableToroidRobot.setPosition(randomDouble(random, 0.0, 5.0), randomDouble(random, 0.0, 5.0), 0.5);
         contactableToroidRobot.createAvailableContactPoints(1, 10, forceVectorScale, true);
         contactableToroidRobot.setGravity(0.0);

         contactables.add(contactableToroidRobot);
         robots.add(contactableToroidRobot);
      }
      
      for (int i = 0; i < numberOfCylinders; i++)
      {
         Vector3d cylinderPosition = new Vector3d(randomDouble(random, 0.0, 5.0), randomDouble(random, 0.0, 5.0), randomDouble(random, -1.2, 0));
         RigidBodyTransform transform3d = new RigidBodyTransform();
         
         AxisAngle4d randomRotation = RandomTools.generateRandomRotation(random);
         transform3d.setTranslation(cylinderPosition);
         transform3d.setRotation(randomRotation);

         double cylinderHeight = 1.5;
         double cylinderRadius = 0.25;

         ContactableStaticCylinderRobot contactableStaticCylinderRobot = new ContactableStaticCylinderRobot("CylinderRobot" + i, transform3d, cylinderHeight, cylinderRadius, YoAppearance.Blue());

         //         contactableToroidRobot.setPosition(randomDouble(random, 0.0, 5.0), randomDouble(random, 0.0, 5.0), 0.5);
         contactableStaticCylinderRobot.createAvailableContactPoints(1, 10, forceVectorScale, true);
         contactableStaticCylinderRobot.setGravity(0.0);

         contactables.add(contactableStaticCylinderRobot);
         robots.add(contactableStaticCylinderRobot);
      }
   
      for (int i = 0; i < numberOfSpheres; i++)
      {
         ContactableSphereRobot contactableSphereRobot = new ContactableSphereRobot("SphereRobot" + i);
         contactableSphereRobot.setPosition(randomDouble(random, 0.0, 5.0), randomDouble(random, 0.0, 5.0), 0.5);
         contactableSphereRobot.createAvailableContactPoints(1, 10, forceVectorScale, true);
         contactableSphereRobot.setGravity(0.0);

         contactables.add(contactableSphereRobot);
         robots.add(contactableSphereRobot);
      }

      for (int i = 0; i < numberOfBoxes; i++)
      {
         ContactableSelectableBoxRobot contactableBoxRobot = null;

         int whichType = random.nextInt(3);
         if (whichType == 0)
         {
            contactableBoxRobot = new ContactableSelectableBoxRobot("BoxRobot" + i);
         }

         else if (whichType == 1)
         {
            contactableBoxRobot = ContactableSelectableBoxRobot.createContactableCardboardBoxRobot("BoxRobot" + i, 0.8, 0.4, 0.6, 10.0);
         }

         else if (whichType == 2)
         {
            contactableBoxRobot = ContactableSelectableBoxRobot.createContactableWoodBoxRobot("BoxRobot" + i, 0.8, 0.4, 0.6, 10.0);
         }

         contactableBoxRobot.setPosition(randomDouble(random, 0.0, 5.0), randomDouble(random, 0.0, 5.0), 0.5);
         contactableBoxRobot.createAvailableContactPoints(1, 10, forceVectorScale, true);
         contactableBoxRobot.setGravity(0.0);

         contactables.add(contactableBoxRobot);
         robots.add(contactableBoxRobot);
      }

      for (int i = 0; i < NUMBER_OF_POINT_MASSES; i++)
      {
         PointMassRobot pointMassRobot = new PointMassRobot("PointMassRobot" + i);

         boolean frontOrBack = random.nextBoolean();

         if (frontOrBack)
         {
            pointMassRobot.setPosition(randomDouble(random, -2.0, 0.0), randomDouble(random, 0.0, 5.0), randomDouble(random, 0.0, 0.5));
            pointMassRobot.setVelocity(0.8, 0.0, 0.0);
         }
         else
         {
            pointMassRobot.setPosition(randomDouble(random, 5.0, 7.0), randomDouble(random, 0.0, 5.0), randomDouble(random, 0.0, 0.5));
            pointMassRobot.setVelocity(-0.8, 0.0, 0.0);
         }

         contactPoints.add(pointMassRobot.getExternalForcePoint());
         robots.add(pointMassRobot);
      }

      ContactController contactController = new ContactController();
      contactController.addContactPoints(contactPoints);
      contactController.addContactables(contactables);

      robots.get(0).setController(contactController);

      Robot[] robotsArray = new Robot[robots.size()];
      robots.toArray(robotsArray);

      return robotsArray;
   }

   private double randomDouble(Random random, double min, double max)
   {
      return min + random.nextDouble() * (max - min);
   }

   public static void main(String[] args)
   {
      ExampleContactSimulationWithMultipleObjects sim = new ExampleContactSimulationWithMultipleObjects();
      sim.runWithGUI();
   }


}
