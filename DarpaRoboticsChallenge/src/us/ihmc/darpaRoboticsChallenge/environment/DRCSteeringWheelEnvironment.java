package us.ihmc.darpaRoboticsChallenge.environment;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.darpaRoboticsChallenge.PolarisRobot;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.robotController.ContactController;
import us.ihmc.simulationconstructionset.util.environments.SelectableObjectListener;
import us.ihmc.simulationconstructionset.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;

public class DRCSteeringWheelEnvironment implements CommonAvatarEnvironmentInterface
{
   private static final boolean CREATE_FACE = false;

   private final List<Robot> robots = new ArrayList<Robot>();
   
   private final CombinedTerrainObject3D combinedTerrainObject;

   private final ArrayList<ExternalForcePoint> contactPoints = new ArrayList<ExternalForcePoint>();

   public DRCSteeringWheelEnvironment(double percentOfSteeringWheelRadius)
   {
      this(0.39, 0.39, 1.27, 0.0, -32.0, percentOfSteeringWheelRadius);
   }

   public DRCSteeringWheelEnvironment(Point3d wheelLocation, double valveYawInDegrees, double valvePitchInDegrees, double percentOfSteeringWheelRadius)
   {
      this(wheelLocation.x, wheelLocation.y, wheelLocation.z, valveYawInDegrees, valvePitchInDegrees, percentOfSteeringWheelRadius);
   }

   public DRCSteeringWheelEnvironment(double x, double y, double z, double yawInDegrees, double pitchInDegrees, double percentOfSteeringWheelRadius)
   {
      combinedTerrainObject = new CombinedTerrainObject3D(getClass().getSimpleName());
      combinedTerrainObject.addTerrainObject(DRCDemo01NavigationEnvironment.setUpGround("Ground"));

      // add Polaris to environment
      RigidBodyTransform polarisTransform = new RigidBodyTransform();
      polarisTransform.setRotation(new AxisAngle4d(new Vector3d(0.0, 0.0, 1.0), 0.0));
      PolarisRobot polarisRobot = new PolarisRobot("polaris", polarisTransform);
      
      if (CREATE_FACE)
      {
         polarisRobot.attachFaceCube();
      }
      
      robots.add(polarisRobot);
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
      ContactController contactController = new ContactController("SteeringWheel");
      contactController.setContactParameters(1000.0, 100.0, 0.5, 0.3);
      contactController.addContactPoints(contactPoints);
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
