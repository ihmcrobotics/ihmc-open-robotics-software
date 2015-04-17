package us.ihmc.darpaRoboticsChallenge.environment;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.robotController.ContactController;
import us.ihmc.simulationconstructionset.util.environments.ContactableSteeringWheelRobot;
import us.ihmc.simulationconstructionset.util.environments.ContactableValveRobot;
import us.ihmc.simulationconstructionset.util.environments.SelectableObjectListener;
import us.ihmc.simulationconstructionset.util.environments.ValveType;
import us.ihmc.simulationconstructionset.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RotationFunctions;

public class DRCSteeringWheelEnvironment implements CommonAvatarEnvironmentInterface
{
   private final List<ContactableValveRobot> wheelRobots = new ArrayList<ContactableValveRobot>();
   private final CombinedTerrainObject3D combinedTerrainObject;

   private final ArrayList<ExternalForcePoint> contactPoints = new ArrayList<ExternalForcePoint>();

   public DRCSteeringWheelEnvironment()
   {
      this(0.5, 0.0, 1.0, 0.0, 57.0);
   }

   public DRCSteeringWheelEnvironment(ArrayList<Point3d> wheelLocations, LinkedHashMap<Point3d, Double> wheelYawAngles_degrees, LinkedHashMap<Point3d, Double> wheelPitchAngles_degrees)
   {
      double forceVectorScale = 1.0 / 50.0;

      combinedTerrainObject = new CombinedTerrainObject3D(getClass().getSimpleName());
      combinedTerrainObject.addTerrainObject(setUpGround("Ground"));

      int i = 0;
      for (Point3d wheelLocation : wheelLocations)
      {
         String wheelRobotName = "SteeringWheelRobot" + i;
         createSteeringWheel(wheelRobotName, ValveType.BIG_VALVE, wheelLocation.x, wheelLocation.y, wheelLocation.z, wheelYawAngles_degrees.get(wheelLocation), wheelPitchAngles_degrees.get(wheelLocation),
               forceVectorScale);
         i++;
      }
   }
   
   public DRCSteeringWheelEnvironment(Point3d wheelLocation, double valveYaw_degrees, double valvePitch_degrees)
   {
      this(wheelLocation.x, wheelLocation.y, wheelLocation.z, valveYaw_degrees, valvePitch_degrees);
   }

   public DRCSteeringWheelEnvironment(double valveX, double valveY, double valveZ, double valveYaw_degrees, double valvePitch_degrees)
   {
      double forceVectorScale = 1.0 / 50.0;

      combinedTerrainObject = new CombinedTerrainObject3D(getClass().getSimpleName());
      combinedTerrainObject.addTerrainObject(setUpGround("Ground"));

      createSteeringWheel("SteeringWheelRobot", ValveType.SMALL_VALVE, valveX, valveY, valveZ, valveYaw_degrees, valvePitch_degrees, forceVectorScale);

   }
   
   private void createSteeringWheel(String wheelRobotName, ValveType valveType, double x, double y, double z, double yaw_degrees, double pitch_degrees, double forceVectorScale)
   {
      FramePose wheelPose = new FramePose(ReferenceFrame.getWorldFrame());
      Point3d position = new Point3d(x, y, z);
      Quat4d orientation = new Quat4d();

      RotationFunctions.setQuaternionBasedOnYawPitchRoll(orientation, Math.toRadians(yaw_degrees), Math.toRadians(pitch_degrees), Math.toRadians(0));
      wheelPose.setPose(position, orientation);

      ContactableSteeringWheelRobot wheel = new ContactableSteeringWheelRobot(wheelRobotName, valveType, 2.0, wheelPose);  // 3.25 turns from lock to lock on Polaris

      wheel.createValveRobot();
      wheel.createAvailableContactPoints(1, 30, forceVectorScale, true);
      wheel.setClosePercentage(50.0);
      
      wheelRobots.add(wheel);
   }

   private CombinedTerrainObject3D setUpGround(String name)
   {
      CombinedTerrainObject3D combinedTerrainObject = new CombinedTerrainObject3D(name);

      combinedTerrainObject.addBox(-10.0, -10.0, 10.0, 10.0, -0.05, 0.0, YoAppearance.DarkBlue());

      return combinedTerrainObject;
   }

   @Override
   public TerrainObject3D getTerrainObject3D()
   {
      return combinedTerrainObject;
   }

   @Override
   public List<ContactableValveRobot> getEnvironmentRobots()
   {
      return wheelRobots;
   }

   @Override
   public void createAndSetContactControllerToARobot()
   {
      ContactController contactController = new ContactController("SteeringWheel");
      contactController.setContactParameters(1000.0, 100.0, 0.5, 0.3);
      contactController.addContactPoints(contactPoints);
      contactController.addContactables(wheelRobots);
      wheelRobots.get(0).setController(contactController);
   }

   @Override
   public void addContactPoints(List<? extends ExternalForcePoint> externalForcePoints)
   {
      this.contactPoints.addAll(externalForcePoints);

   }

   @Override
   public void addSelectableListenerToSelectables(SelectableObjectListener selectedListener)
   {
      // TODO Auto-generated method stub

   }
}
