package us.ihmc.darpaRoboticsChallenge.environment;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Vector3d;

import us.ihmc.darpaRoboticsChallenge.DRCVehicleModelObjects;
import us.ihmc.darpaRoboticsChallenge.controllers.SteeringWheelDisturbanceController;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.graveYard.commonWalkingControlModules.vrc.highLevelHumanoidControl.driving.VehicleObject;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.robotController.ContactController;
import us.ihmc.simulationconstructionset.util.environments.ContactableStaticCylinderRobot;
import us.ihmc.simulationconstructionset.util.environments.ContactableToroidRobot;
import us.ihmc.simulationconstructionset.util.environments.SelectableObject;
import us.ihmc.simulationconstructionset.util.environments.SelectableObjectListener;
import us.ihmc.simulationconstructionset.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.util.ground.Contactable;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;
import us.ihmc.simulationconstructionset.util.math.functionGenerator.YoFunctionGeneratorMode;
import us.ihmc.robotics.geometry.shapes.Box3d;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;

public class DRCDemoEnvironmentWithBoxAndSteeringWheel implements CommonAvatarEnvironmentInterface
{
   private final CombinedTerrainObject3D combinedTerrainObject;

   private final ArrayList<Robot> environmentRobots = new ArrayList<Robot>();
   private final ArrayList<ExternalForcePoint> contactPoints = new ArrayList<ExternalForcePoint>();
   private final ArrayList<Contactable> contactables = new ArrayList<Contactable>();
   private final ContactableToroidRobot steeringWheelRobot;

   public DRCDemoEnvironmentWithBoxAndSteeringWheel(YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      DRCVehicleModelObjects drcVehicleModelObjects = new DRCVehicleModelObjects();



      combinedTerrainObject = createCombinedTerrainObject();

      steeringWheelRobot = createSteeringWheel(drcVehicleModelObjects);
      contactables.add(steeringWheelRobot);
      environmentRobots.add(steeringWheelRobot);

      createBars(environmentRobots, contactables);
   }

   private static ContactableToroidRobot createSteeringWheel(DRCVehicleModelObjects drcVehicleModelObjects)
   {
      RigidBodyTransform steeringWheelTransform = drcVehicleModelObjects.getTransform(VehicleObject.STEERING_WHEEL);

      /*
       * Quick estimates from 3D files:
       */
      double externalSteeringWheelRadius = 0.173;
      double internalSteeringWheelRadius = 0.143;

      double toroidRadius = (externalSteeringWheelRadius - internalSteeringWheelRadius) / 2.0;
      double steeringWheelRadius = (externalSteeringWheelRadius + internalSteeringWheelRadius) / 2.0;

      double mass = 1.0;
      ContactableToroidRobot steeringWheelRobot = new ContactableToroidRobot("steeringWheel", steeringWheelTransform, steeringWheelRadius, toroidRadius, mass);
      steeringWheelRobot.setDamping(2.0);
      steeringWheelRobot.createAvailableContactPoints(1, 30, 0.005, false);
      return steeringWheelRobot;
   }

   private static void createBars(ArrayList<Robot> environmentRobots, ArrayList<Contactable> contactables)
   {
      addContactableCylinderRobot(environmentRobots, contactables, "rollcage_top_left", 0.0, 0.61, 1.92, -3.141592, 1.531593, -3.141592, 0.030000, 0.680000);

      addContactableCylinderRobot(environmentRobots, contactables, "rollcage_top_right", 0.000000, -0.610000, 1.920000, -3.141592, 1.531593, -3.141592, 0.03, 0.68);

      addContactableCylinderRobot(environmentRobots, contactables, "rollcage_top_front", 0.325000, 0.000000, 1.890000, 1.570796, 0.000000, 0.000000, 0.030000, 1.220000);

      addContactableCylinderRobot(environmentRobots, contactables, "rollcage_top_back", -0.330000, 0.000000, 1.920000, 1.570796, 0.000000, 0.000000, 0.030000, 1.220000);

      addContactableCylinderRobot(environmentRobots, contactables, "rollcage_front_left", 0.540000, 0.610000, 1.450000, 0.000000, -0.440000, 0.000000, 0.030000, 1.040000);

      addContactableCylinderRobot(environmentRobots, contactables, "rollcage_front_right", 0.540000, -0.610000, 1.450000, 0.000000, -0.440000, 0.000000, 0.030000, 1.040000);
   }

   private static void addContactableCylinderRobot(ArrayList<Robot> environmentRobots, ArrayList<Contactable> contactables, String name, double x, double y,
                                            double z, double roll, double pitch, double yaw, double radius, double height)
   {
      RigidBodyTransform transform = new RigidBodyTransform();
      transform.setEuler(roll, pitch, yaw);

      Vector3d position = new Vector3d(x, y, z);
      Vector3d baseToCenter = new Vector3d(0.0, 0.0, -height / 2.0);
      transform.transform(baseToCenter);
      position.add(baseToCenter);

      transform.setTranslation(position);
      ContactableStaticCylinderRobot robot = new ContactableStaticCylinderRobot(name, transform, height, radius, YoAppearance.DarkGray());
      environmentRobots.add(robot);
      contactables.add(robot);

      robot.createAvailableContactPoints(1, 30, 0.005, false);
   }

   private CombinedTerrainObject3D createCombinedTerrainObject()
   {
      CombinedTerrainObject3D terrainObject = new CombinedTerrainObject3D("carSeatBox");

      // mud_seat
//      addBox(-0.1, 0.0, 0.81, 0.0, 0.0, 0.0, 0.6, 1.15, 0.1, terrainObject);

      // seat
      addBox(-0.1, 0.0, 0.56, 0.0, 0.0, 0.0, 0.6, 1.22, 0.5, terrainObject);

      // seat_back
      addBox(-0.3, 0.0, 1.125, 0.0, -0.2, 0.0, 0.06, 1.0, 0.4, terrainObject);

      // chassis_bottom
      addBox(0.1, 0.0, 0.37, 0.0, 0.0, 0.0, 1.5, 1.34, 0.06, terrainObject);

      // engine
      addBox(1.05, 0.0, 0.7, 0.0, 0.0, 0.0, 0.58, 1.0, 0.8, terrainObject);

      // ground
      terrainObject.addBox(-1.0, -1.0, 1.0, 1.0, -0.05, 0.0, YoAppearance.DarkGray());

      return terrainObject;
   }

   private void addBox(double x, double y, double z, double roll, double pitch, double yaw, double sizeX, double sizeY, double sizeZ, CombinedTerrainObject3D terrainObject)
   {
      RigidBodyTransform transform = new RigidBodyTransform();
      transform.setEuler(roll, pitch, yaw);
      transform.setTranslation(new Vector3d(x, y, z));
      Box3d box = new Box3d(transform, sizeX, sizeY, sizeZ);
      terrainObject.addRotatableBox(box, YoAppearance.DarkGray());
   }
   
   @Override
   public TerrainObject3D getTerrainObject3D()
   {
      return combinedTerrainObject;
   }

   @Override
   public List<Robot> getEnvironmentRobots()
   {
      return new ArrayList<Robot>(environmentRobots);
   }

   @Override
   public void addContactPoints(List<? extends ExternalForcePoint> externalForcePoints)
   {
      for (ExternalForcePoint contactPoint : contactPoints)
      {
         this.contactPoints.add(contactPoint);
      }
   }

   @Override
   public void createAndSetContactControllerToARobot()
   {
      // add contact controller to any robot so it gets called
      ContactController contactController = new ContactController();
      contactController.setContactParameters(10000.0, 1000.0, 0.5, 0.3);
      contactController.addContactPoints(contactPoints);
      contactController.addContactables(contactables);
      environmentRobots.get(0).setController(contactController);
   }
   
   public void activateDisturbanceControllerOnSteeringWheel(YoFunctionGeneratorMode disturbanceMode)
   {
      SteeringWheelDisturbanceController controller = new SteeringWheelDisturbanceController(steeringWheelRobot, disturbanceMode);
      steeringWheelRobot.setController(controller);
   }

   @Override
   public void addSelectableListenerToSelectables(SelectableObjectListener selectedListener)
   {
      for (Contactable contactable : contactables)
      {
         if (contactable instanceof SelectableObject)
         {
            ((SelectableObject) contactable).addSelectedListeners(selectedListener);
         }
      }
   }
}
