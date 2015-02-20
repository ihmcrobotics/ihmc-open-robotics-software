package us.ihmc.darpaRoboticsChallenge.environment;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Vector3d;

import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.robotController.ContactController;
import us.ihmc.simulationconstructionset.util.environments.ContactableDoorRobot;
import us.ihmc.simulationconstructionset.util.environments.SelectableObjectListener;
import us.ihmc.simulationconstructionset.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;

/**
 * DRCDoorEnvironment - all specific numbers taken from
 * <a href="http://archive.darpa.mil/roboticschallengetrialsarchive/sites/default/files/DRC%20Trials%20Task%20Description%20Release%2011%20DISTAR%2022197.pdf">the task description</a>, including:
 * 
 * <ul>
 *    <li> Door width = 33.5in
 *    <li> Force to open if weighted = 3lb applied at the handle
 * </ul>
 */
public class DRCDoorEnvironment implements CommonAvatarEnvironmentInterface
{
   private final List<ContactableDoorRobot> doorRobots = new ArrayList<ContactableDoorRobot>();
   private final CombinedTerrainObject3D combinedTerrainObject;
      
   private final ArrayList<ExternalForcePoint> contactPoints = new ArrayList<ExternalForcePoint>();

   public DRCDoorEnvironment()
   {
      combinedTerrainObject = new CombinedTerrainObject3D(getClass().getSimpleName());
      combinedTerrainObject.addTerrainObject(setUpGround("Ground"));
   }
   
   public void createDoor(Vector3d positionInWorld, int id)
   {
      ContactableDoorRobot door = new ContactableDoorRobot("doorRobot" + id, positionInWorld);
      door.setKdDoor(1.0);
      door.setKpDoor(10.0);
      doorRobots.add(door);
      door.createAvailableContactPoints(id, 40, 0.02, true);
   }
   
   private CombinedTerrainObject3D setUpGround(String name)
   {
      CombinedTerrainObject3D combinedTerrainObject = new CombinedTerrainObject3D(name);

      combinedTerrainObject.addBox(-10.0, -10.0, 10.0, 10.0, -0.05, 0.0, YoAppearance.DarkGray());
      
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
      System.out.println("# of contact pts = " + contactPoints.size());
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
   
   public enum DoorType
   {
      NO_TORQUE, THREE_LBS_TO_MOVE, FIVE_LBS_TO_MOVE;
   }

}
