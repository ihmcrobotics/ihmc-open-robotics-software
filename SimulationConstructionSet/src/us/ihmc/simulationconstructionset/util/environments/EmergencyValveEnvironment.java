package us.ihmc.simulationconstructionset.util.environments;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.robotController.ContactController;
import us.ihmc.simulationconstructionset.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;

public class EmergencyValveEnvironment implements CommonAvatarEnvironmentInterface
{
   private final List<ContactableValveRobot> valveRobot = new ArrayList<ContactableValveRobot>();
   private final CombinedTerrainObject3D combinedTerrainObject;

   private final ArrayList<ExternalForcePoint> contactPoints = new ArrayList<ExternalForcePoint>();

   public EmergencyValveEnvironment()
   {
      
      this(0.75f,-0.38f,1.1811f, 0.0);
   }

   public EmergencyValveEnvironment(ArrayList<Point3D> valveLocations, LinkedHashMap<Point3D, Double> valveYawAngles_degrees)
   {
      double forceVectorScale = 1.0 / 50.0;

      combinedTerrainObject = new CombinedTerrainObject3D(getClass().getSimpleName());
      combinedTerrainObject.addTerrainObject(DefaultCommonAvatarEnvironment.setUpGround("Ground"));

      int i = 0;
      for (Point3D valveLocation : valveLocations)
      {
         String valveRobotName = "ValveRobot" + i;
         createValve(valveRobotName, ValveType.BIG_VALVE, valveLocation.getX(), valveLocation.getY(), valveLocation.getZ(), valveYawAngles_degrees.get(valveLocation),
               forceVectorScale);
         i++;
      }
   }

   public EmergencyValveEnvironment(double valveX, double valveY, double valveZ, double valveYaw_degrees)
   {
      double forceVectorScale = 1.0 / 50.0;

      combinedTerrainObject = new CombinedTerrainObject3D(getClass().getSimpleName());
      combinedTerrainObject.addTerrainObject(DefaultCommonAvatarEnvironment.setUpGround("Ground"));

      createValve("ValveRobot", ValveType.SMALL_VALVE, valveX, valveY, valveZ, valveYaw_degrees, forceVectorScale);

   }

   private void createValve(String valveRobotName, ValveType valveType, double x, double y, double z, double yaw_degrees, double forceVectorScale)
   {
      FramePose valvePose = new FramePose(ReferenceFrame.getWorldFrame());
      Point3D position = new Point3D(x, y, z);
      Quaternion orientation = new Quaternion();

      orientation.setYawPitchRoll(Math.toRadians(yaw_degrees), Math.toRadians(0), Math.toRadians(0));
      valvePose.setPose(position, orientation);

      ContactableValveRobot valve = new ContactableValveRobot(valveRobotName, valveType, 0.5, valvePose);

      valve.createValveRobot();
      valve.createAvailableContactPoints(1, 30, forceVectorScale, true);

      valveRobot.add(valve);
   }

   @Override
   public TerrainObject3D getTerrainObject3D()
   {
      return combinedTerrainObject;
   }

   @Override
   public List<ContactableValveRobot> getEnvironmentRobots()
   {
      return valveRobot;
   }

   @Override
   public void createAndSetContactControllerToARobot()
   {
      ContactController contactController = new ContactController();
      contactController.setContactParameters(1000.0, 100.0, 0.5, 0.3);
      contactController.addContactPoints(contactPoints);
      contactController.addContactables(valveRobot);
      valveRobot.get(0).setController(contactController);
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
