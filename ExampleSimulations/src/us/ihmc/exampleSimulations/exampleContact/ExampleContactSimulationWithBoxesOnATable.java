package us.ihmc.exampleSimulations.exampleContact;


import java.util.ArrayList;
import java.util.Random;

import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.GroundContactModel;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.util.LinearStickSlipGroundContactModel;
import us.ihmc.simulationConstructionSetTools.util.environments.environmentRobots.ContactableSelectableBoxRobot;
import us.ihmc.exampleSimulations.pointMassRobot.PointMassRobot;
import us.ihmc.simulationconstructionset.util.ground.Contactable;

public class ExampleContactSimulationWithBoxesOnATable
{
   private static final double[][] boxDimensions = new double[][]{
      {0.2, 0.3, 0.5},
      {0.7, 0.9, 1.5},
      {0.5, 0.4, 0.5},
   };
   
   private static final double[][] boxLocations = new double[][]{
      {0.0, 0.0, 1.5},
      {1.0, 1.0, 1.5},
      {3.0, 0.0, 1.5}
   };
   
   private static final int numberOfPointMasses = 100;

   private final ArrayList<ExternalForcePoint> contactPoints = new ArrayList<ExternalForcePoint>();
   private final ArrayList<Contactable> contactables = new ArrayList<Contactable>();
   private final ArrayList<Robot> robots = new ArrayList<Robot>();
   
   public ExampleContactSimulationWithBoxesOnATable()
   {
//    SimpleTableEnvironmentOne simpleTableEnvironmentOne = new SimpleTableEnvironmentOne(150.0);
      ExampleTerrainWithTable exampleTerrainWithTable = new ExampleTerrainWithTable();
    
      double forceVectorScale = 1.0/50.0;

      Random random = new Random(1776L);
      
      for (int i=0; i<boxDimensions.length; i++)
      {
         double[] boxDimension = boxDimensions[i];
         double length = boxDimension[0];
         double width = boxDimension[1];
         double height = boxDimension[2];
         
         ContactableSelectableBoxRobot contactableBoxRobot = null;
         
         if (i==0)
         {
            contactableBoxRobot = new ContactableSelectableBoxRobot("BoxRobot" + i, length, width, height, 5.0);
         }
         else if (i==1)
         {
            contactableBoxRobot = ContactableSelectableBoxRobot.createContactableCardboardBoxRobot("BoxRobot" + i, length, width, height, 5.0);
         }
         else if (i==2)
         {
            contactableBoxRobot = ContactableSelectableBoxRobot.createContactableWoodBoxRobot("BoxRobot" + i, length, width, height, 5.0);
         }
         
         contactableBoxRobot.setPosition(boxLocations[i]); 
         contactableBoxRobot.createAvailableContactPoints(1, 10, forceVectorScale, true);

//         contactableBoxRobot.setGravity(0.0);
         
         double kXY = 40000.0;
         double bXY = 10.0;
         double kZ = 80.0;
         double bZ = 500.0;
         double alphaStick = 0.7;
         double alphaSlip = 0.5;
         
         GroundContactModel groundContactModel = new LinearStickSlipGroundContactModel(contactableBoxRobot, 
               kXY, bXY, kZ, bZ, alphaSlip, alphaStick, contactableBoxRobot.getRobotsYoVariableRegistry());  
         
         groundContactModel.setGroundProfile3D(exampleTerrainWithTable);
         contactableBoxRobot.setGroundContactModel(groundContactModel);
         
         contactables.add(contactableBoxRobot);
         robots.add(contactableBoxRobot);
      }
      
      createABarrageOfPointMasses(random, numberOfPointMasses, contactPoints, robots);
      
      ContactController contactController = new ContactController();
      contactController.addContactPoints(contactPoints);
      contactController.addContactables(contactables);
      
      robots.get(0).setController(contactController);
      
      Robot[] robotsArray = new Robot[robots.size()];
      robots.toArray(robotsArray);
      
      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setDataBufferSize(36000);
      
      SimulationConstructionSet scs = new SimulationConstructionSet(robotsArray, parameters);
      scs.setDT(0.001, 1);
      scs.setSimulateDuration(8.0);
             
//      scs.addStaticLinkGraphics(simpleTableEnvironmentOne.getLinkGraphics());
      scs.addStaticLinkGraphics(exampleTerrainWithTable.getLinkGraphics());
      
      Graphics3DObject referenceFrameLinkGraphics = new Graphics3DObject();
      referenceFrameLinkGraphics.addCoordinateSystem(1.0);
      scs.addStaticLinkGraphics(referenceFrameLinkGraphics);
      
      
//      SelectablePickedListener boxPickedListener = new SelectablePickedListener();
//      scs.attachClickedPickableObjectListener(boxPickedListener);
      
      scs.setGroundVisible(false);
      
      scs.startOnAThread();
   }

   private static void createABarrageOfPointMasses(Random random, int numberOfPointMasses, ArrayList<ExternalForcePoint> contactPointsToAddTo, ArrayList<Robot> robotsToAddTo)
   {
      for (int i=0; i<numberOfPointMasses; i++)
      {
         PointMassRobot pointMassRobot = new PointMassRobot("PointMassRobot" + i, 10.0);
         
         boolean frontOrBack = random.nextBoolean();
         
         if (frontOrBack)
         {
            pointMassRobot.setPosition(randomDouble(random, -2.0, -0.5), randomDouble(random, -1.0, 1.5), randomDouble(random, 0.0, 1.6)); 
            pointMassRobot.setVelocity(0.8, 0.0, 0.0);
         }
         else
         {
            pointMassRobot.setPosition(randomDouble(random, 3.0, 4.5), randomDouble(random, -1.0, 1.5), randomDouble(random, 0.0, 1.6)); 
            pointMassRobot.setVelocity(-0.8, 0.0, 0.0);
         }
         
         contactPointsToAddTo.add(pointMassRobot.getExternalForcePoint());
         robotsToAddTo.add(pointMassRobot);
      }
   }
   
   private static double randomDouble(Random random, double min, double max)
   {
      return min + random.nextDouble() * (max - min);
   }
   
   public static void main(String[] args)
   {
      new ExampleContactSimulationWithBoxesOnATable();
   }
}


