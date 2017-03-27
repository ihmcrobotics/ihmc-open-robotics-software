package us.ihmc.simulationConstructionSetTools.util.environments;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.simulationConstructionSetTools.util.environments.environmentRobots.ContactableDoorRobot;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationConstructionSetTools.robotController.ContactController;
import us.ihmc.simulationconstructionset.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;

public class DoorEnvironment implements CommonAvatarEnvironmentInterface
{
   private final List<ContactableDoorRobot> doorRobots = new ArrayList<ContactableDoorRobot>();
   private final CombinedTerrainObject3D combinedTerrainObject;
      
   private final ArrayList<ExternalForcePoint> contactPoints = new ArrayList<ExternalForcePoint>();

   public DoorEnvironment()
   {
      combinedTerrainObject = new CombinedTerrainObject3D(getClass().getSimpleName());
      combinedTerrainObject.addTerrainObject(setUpGround("Ground"));
      
      ContactableDoorRobot door = new ContactableDoorRobot("doorRobot", new Point3D(3.0, 0.0, 0.0));
      doorRobots.add(door);
      door.createAvailableContactPoints(0, 15, 15, 0.02, true);
   }
   
   private CombinedTerrainObject3D setUpGround(String name)
   {
      CombinedTerrainObject3D combinedTerrainObject = new CombinedTerrainObject3D(name);

      combinedTerrainObject.addBox(-10.0, -10.0, 10.0, 10.0, -0.05, 0.0, YoAppearance.DarkGray());
      combinedTerrainObject.addBox(2.0, -0.05, 3.0, 0.05, 2.0, YoAppearance.Beige());
      combinedTerrainObject.addBox(3.0 + ContactableDoorRobot.DEFAULT_DOOR_DIMENSIONS.getX(), -0.05, 4.0 + ContactableDoorRobot.DEFAULT_DOOR_DIMENSIONS.getX(), 0.05, 2.0, YoAppearance.Beige());
      
      return combinedTerrainObject;
   }
   
   @Override
   public TerrainObject3D getTerrainObject3D()
   {
      return combinedTerrainObject;
   }

   @Override
   public List<? extends Robot> getEnvironmentRobots()
   {
      return doorRobots;
   }

   @Override
   public void createAndSetContactControllerToARobot()
   {
      ContactController contactController = new ContactController();
      contactController.setContactParameters(100000.0, 100.0, 0.5, 0.3);

      contactController.addContactPoints(contactPoints);
      contactController.addContactables(doorRobots);
      doorRobots.get(0).setController(contactController);    
   }

   @Override
   public void addContactPoints(List<? extends ExternalForcePoint> externalForcePoints)
   {
      this.contactPoints.addAll(externalForcePoints);      
   }

   @Override
   public void addSelectableListenerToSelectables(SelectableObjectListener selectedListener)
   {
   }
}
