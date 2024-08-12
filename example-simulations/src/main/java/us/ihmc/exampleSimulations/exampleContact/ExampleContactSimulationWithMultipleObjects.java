package us.ihmc.exampleSimulations.exampleContact;


import java.util.ArrayList;
import java.util.Random;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationConstructionSetTools.util.environments.environmentRobots.ContactableSelectableBoxRobot;
import us.ihmc.simulationConstructionSetTools.util.environments.environmentRobots.ContactableSphereRobot;
import us.ihmc.simulationConstructionSetTools.util.environments.environmentRobots.ContactableStaticCylinderRobot;
import us.ihmc.simulationConstructionSetTools.util.environments.environmentRobots.ContactableToroidRobot;
import us.ihmc.exampleSimulations.pointMassRobot.PointMassRobot;
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
         Vector3D toroidPosition = new Vector3D(randomDouble(random, 0.0, 5.0), randomDouble(random, 0.0, 5.0), randomDouble(random, 0.2, 0.6));
         RigidBodyTransform transform3d = new RigidBodyTransform();
         
         AxisAngle randomRotation = EuclidCoreRandomTools.nextAxisAngle(random);
         transform3d.getTranslation().set(toroidPosition);
         transform3d.getRotation().set(randomRotation);

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
         Vector3D cylinderPosition = new Vector3D(randomDouble(random, 0.0, 5.0), randomDouble(random, 0.0, 5.0), randomDouble(random, -1.2, 0));
         RigidBodyTransform transform3d = new RigidBodyTransform();
         
         AxisAngle randomRotation = EuclidCoreRandomTools.nextAxisAngle(random);
         transform3d.getTranslation().set(cylinderPosition);
         transform3d.getRotation().set(randomRotation);

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
