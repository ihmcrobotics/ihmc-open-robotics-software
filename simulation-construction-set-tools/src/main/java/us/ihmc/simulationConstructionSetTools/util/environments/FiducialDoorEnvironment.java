package us.ihmc.simulationConstructionSetTools.util.environments;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.appearance.YoAppearanceTexture;
import us.ihmc.simulationConstructionSetTools.robotController.ContactController;
import us.ihmc.simulationConstructionSetTools.util.environments.environmentRobots.ContactableDoorRobot;
import us.ihmc.simulationConstructionSetTools.util.environments.environmentRobots.FiducialDoorRobot;
import us.ihmc.simulationConstructionSetTools.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.ground.RotatableBoxTerrainObject;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;

import java.util.ArrayList;
import java.util.List;

public class FiducialDoorEnvironment implements CommonAvatarEnvironmentInterface
{
   private final List<FiducialDoorRobot> robots = new ArrayList<>();
   private final CombinedTerrainObject3D combinedTerrainObject;

   private final ArrayList<ExternalForcePoint> contactPoints = new ArrayList<>();

   private final double distanceToDoor = 3.0;
   private final double wallWidth = 1.0;
   private final double doorWidth = ContactableDoorRobot.DEFAULT_DOOR_DIMENSIONS.getX();
   private final double doorThickness = ContactableDoorRobot.DEFAULT_DOOR_DIMENSIONS.getY();
   private final double doorHeight = ContactableDoorRobot.DEFAULT_DOOR_DIMENSIONS.getZ();

   public FiducialDoorEnvironment()
   {
      combinedTerrainObject = new CombinedTerrainObject3D(getClass().getSimpleName());
      combinedTerrainObject.addTerrainObject(setUpGround("Ground"));

      Point3D doorPosition = new Point3D(distanceToDoor, doorWidth / 2.0, 0.0);
      AxisAngle doorOrientation = new AxisAngle(Axis3D.Z, -Math.PI / 2.0);
      FiducialDoorRobot door = new FiducialDoorRobot("doorRobot", doorPosition, doorOrientation);
      robots.add(door);
      door.createAvailableContactPoints(0, 15, 15, 0.02, true);
   }

   public CombinedTerrainObject3D setUpGround(String name)
   {
      CombinedTerrainObject3D combinedTerrainObject = new CombinedTerrainObject3D(name);

      //    URL fileURL = DRCDemo01NavigationEnvironment.class.getClassLoader().getResource("Textures/ground2.png");
      YoAppearanceTexture texture = new YoAppearanceTexture("Textures/ground2.png");

      RigidBodyTransform location = new RigidBodyTransform();
      location.setTranslation(new Vector3D(0, 0, -0.5));

      RotatableBoxTerrainObject newBox = new RotatableBoxTerrainObject(new Box3D(location, 45, 45, 1), texture);
      combinedTerrainObject.addTerrainObject(newBox);
      RotatableBoxTerrainObject newBox2 = new RotatableBoxTerrainObject(new Box3D(location, 200, 200, 0.75), YoAppearance.DarkGray());
      combinedTerrainObject.addTerrainObject(newBox2);

      double xStart;
      double yStart;
      double xEnd;
      double yEnd;
      double height;

      xStart = distanceToDoor;
      yStart = wallWidth +(doorWidth / 2.0);
      xEnd = distanceToDoor + doorThickness;
      yEnd = doorWidth / 2.0;
      height = doorHeight;
      combinedTerrainObject.addBox(xStart, yStart, xEnd, yEnd, height, YoAppearance.Beige());

      xStart = distanceToDoor;
      yStart = -doorWidth / 2.0;
      xEnd = distanceToDoor + doorThickness;
      yEnd = -(doorWidth / 2.0) - wallWidth;
      height = doorHeight;
      combinedTerrainObject.addBox(xStart, yStart, xEnd, yEnd, height, YoAppearance.Beige());

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
      return robots;
   }

   @Override
   public void createAndSetContactControllerToARobot()
   {
      ContactController contactController = new ContactController();
      contactController.setContactParameters(100000.0, 100.0, 0.5, 0.3);

      contactController.addContactPoints(contactPoints);
      contactController.addContactables(robots);
      robots.get(0).setController(contactController);
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
