package us.ihmc.darpaRoboticsChallenge.environment;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Point3d;

import us.ihmc.darpaRoboticsChallenge.PolarisRobot;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.robotController.ContactController;
import us.ihmc.simulationconstructionset.util.environments.ContactableSteeringWheelRobot;
import us.ihmc.simulationconstructionset.util.environments.SelectableObjectListener;
import us.ihmc.simulationconstructionset.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;

public class DRCSteeringWheelEnvironment implements CommonAvatarEnvironmentInterface
{
   public enum SteeringWheelAttachment
   {
      SPINNER, CROSS_BAR, NONE
   }
   
   private final List<Robot> robots = new ArrayList<Robot>();
   private final ContactableSteeringWheelRobot wheelRobot;
   
   private final CombinedTerrainObject3D combinedTerrainObject;

   private final ArrayList<ExternalForcePoint> contactPoints = new ArrayList<ExternalForcePoint>();

   public DRCSteeringWheelEnvironment(double percentOfSteeringWheelRadius, SteeringWheelAttachment wheelAttachment)
   {
      this(0.39, 0.39, 1.27, 0.0, -32.0, percentOfSteeringWheelRadius, wheelAttachment);
   }

   public DRCSteeringWheelEnvironment(Point3d wheelLocation, double valveYawInDegrees, double valvePitchInDegrees, double percentOfSteeringWheelRadius,
         SteeringWheelAttachment wheelAttachment)
   {
      this(wheelLocation.x, wheelLocation.y, wheelLocation.z, valveYawInDegrees, valvePitchInDegrees, percentOfSteeringWheelRadius, wheelAttachment);
   }

   public DRCSteeringWheelEnvironment(double x, double y, double z, double yawInDegrees, double pitchInDegrees, double percentOfSteeringWheelRadius,
         SteeringWheelAttachment wheelAttachment)
   {
      double forceVectorScale = 1.0 / 50.0;

      combinedTerrainObject = new CombinedTerrainObject3D(getClass().getSimpleName());
      combinedTerrainObject.addTerrainObject(DRCDemo01NavigationEnvironment.setUpGround("Ground"));

      wheelRobot = ContactableSteeringWheelRobot.createPolarisSteeringWheel(x, y, z, Math.toRadians(yawInDegrees), Math.toRadians(pitchInDegrees));
      wheelRobot.createSteeringWheelRobot();
      wheelRobot.createAvailableContactPoints(1, 30, forceVectorScale, true);
      switch (wheelAttachment)
      {
         case SPINNER:
            wheelRobot.addSpinnerHandle(percentOfSteeringWheelRadius);
            break;
         case CROSS_BAR:
            wheelRobot.addCrossBar();
            break;
         default:
            break;
      }
      
      robots.add(wheelRobot);
      
      RigidBodyTransform polarisTransform = new RigidBodyTransform();
      robots.add(new PolarisRobot("polaris", polarisTransform));
   }



   public ReferenceFrame getSteeringWheelFrame()
   {
      return wheelRobot.getSteeringWheelFrame();
   }

   public double getSteeringWheelRadius()
   {
      return wheelRobot.getSteeringWheelRadius();
   }

   public FramePoint getSpinnerHandleCenter()
   {
      return wheelRobot.getSpinnerHandleCenter();
   }

   public FrameVector getSteeringWheelAxis()
   {
      return wheelRobot.getSteeringWheelAxis();
   }

   public FramePoint getSteeringWheelCenter()
   {
      return wheelRobot.getSteeringWheelCenter();
   }

   public double getSteeringWheelAngleAsAbsolutePercentageOfRangeOfMotion()
   {
      return wheelRobot.getSteeringWheelAngleAsAbsolutePercentageOfRangeOfMotion();
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
      contactController.addContactable(wheelRobot);
      wheelRobot.setController(contactController);
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
