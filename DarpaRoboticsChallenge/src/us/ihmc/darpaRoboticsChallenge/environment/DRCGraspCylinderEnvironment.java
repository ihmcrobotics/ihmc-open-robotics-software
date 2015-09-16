package us.ihmc.darpaRoboticsChallenge.environment;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.geometry.RotationFunctions;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.robotController.ContactController;
import us.ihmc.simulationconstructionset.util.environments.ContactableStaticCylinderRobot;
import us.ihmc.simulationconstructionset.util.environments.SelectableObjectListener;
import us.ihmc.simulationconstructionset.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;

public class DRCGraspCylinderEnvironment implements CommonAvatarEnvironmentInterface
{
   private final List<ContactableStaticCylinderRobot> cylinderRobots = new ArrayList<ContactableStaticCylinderRobot>();
   private final CombinedTerrainObject3D combinedTerrainObject;

   private final ArrayList<ExternalForcePoint> contactPoints = new ArrayList<ExternalForcePoint>();

   public DRCGraspCylinderEnvironment()
   {
      this(0.5, 0.0, 1.0, 0.0, 0.0, 57.0);
   }

   public DRCGraspCylinderEnvironment(ArrayList<Point3d> wheelLocations, LinkedHashMap<Point3d, Double> yawAngles_degrees, LinkedHashMap<Point3d, Double> pitchAngles_degrees, LinkedHashMap<Point3d, Double> rollAngles_degrees)
   {
      double forceVectorScale = 1.0 / 50.0;

      combinedTerrainObject = new CombinedTerrainObject3D(getClass().getSimpleName());
      combinedTerrainObject.addTerrainObject(setUpGround("Ground"));

      int i = 0;
      for (Point3d wheelLocation : wheelLocations)
      {
         String cylinderName = "Cylinder" + i;
         createRotatedZUpCylinder(cylinderName, 0.2, 0.03, wheelLocation.x, wheelLocation.y, wheelLocation.z, yawAngles_degrees.get(wheelLocation),
               pitchAngles_degrees.get(wheelLocation),  rollAngles_degrees.get(wheelLocation), forceVectorScale);
         i++;
      }
   }

   public DRCGraspCylinderEnvironment(double x, double y, double z, double yaw_degrees, double pitch_degrees, double roll_degrees)
   {
      double forceVectorScale = 1.0 / 50.0;

      combinedTerrainObject = new CombinedTerrainObject3D(getClass().getSimpleName());
      combinedTerrainObject.addTerrainObject(setUpGround("Ground"));

      createRotatedZUpCylinder("Cylinder", 0.2, 0.03, x, y, z, yaw_degrees, pitch_degrees, roll_degrees, forceVectorScale);

   }
   
   private void createRotatedZUpCylinder(String cylinderName, double height, double radius, double x, double y, double z, double yaw_degrees, double pitch_degrees, double roll_degrees, double forceVectorScale)
   {
      FramePose cylinderPose = new FramePose(ReferenceFrame.getWorldFrame());
      Point3d position = new Point3d(x, y, z);
      Quat4d orientation = new Quat4d();

      RotationFunctions.setQuaternionBasedOnYawPitchRoll(orientation, Math.toRadians(yaw_degrees), Math.toRadians(pitch_degrees), Math.toRadians(0));
      cylinderPose.setPose(position, orientation);
      
      RigidBodyTransform cylinderTransformToWorld = new RigidBodyTransform();
      cylinderPose.getPose(cylinderTransformToWorld);
      
      ContactableStaticCylinderRobot cylinder = new ContactableStaticCylinderRobot(cylinderName, cylinderTransformToWorld, height, radius, YoAppearance.Red());
      cylinder.createAvailableContactPoints(1, 30, forceVectorScale, true);
      
      cylinderRobots.add(cylinder);
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
   public List<ContactableStaticCylinderRobot> getEnvironmentRobots()
   {
      return cylinderRobots;
   }

   @Override
   public void createAndSetContactControllerToARobot()
   {
      ContactController contactController = new ContactController();
      contactController.setContactParameters(1000.0, 100.0, 0.5, 0.3);
      contactController.addContactPoints(contactPoints);
      contactController.addContactables(cylinderRobots);
      cylinderRobots.get(0).setController(contactController);
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
