package us.ihmc.darpaRoboticsChallenge.environment;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Point3d;

import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.robotController.ContactController;
import us.ihmc.simulationconstructionset.util.environments.ContactableSteeringWheelRobot;
import us.ihmc.simulationconstructionset.util.environments.SelectableObjectListener;
import us.ihmc.simulationconstructionset.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

public class DRCSteeringWheelEnvironment implements CommonAvatarEnvironmentInterface
{
   private final ContactableSteeringWheelRobot wheelRobot;
   private final CombinedTerrainObject3D combinedTerrainObject;

   private final ArrayList<ExternalForcePoint> contactPoints = new ArrayList<ExternalForcePoint>();

   public DRCSteeringWheelEnvironment()
   {
      this(0.5, 0.0, 1.0, 0.0, -33.0);
   }

   public DRCSteeringWheelEnvironment(Point3d wheelLocation, double valveYawInDegrees, double valvePitchInDegrees)
   {
      this(wheelLocation.x, wheelLocation.y, wheelLocation.z, valveYawInDegrees, valvePitchInDegrees);
   }

   public DRCSteeringWheelEnvironment(double x, double y, double z, double yawInDegrees, double pitchInDegrees)
   {
      double forceVectorScale = 1.0 / 50.0;

      combinedTerrainObject = new CombinedTerrainObject3D(getClass().getSimpleName());
      combinedTerrainObject.addTerrainObject(DRCDemo01NavigationEnvironment.setUpGround("Ground"));

      wheelRobot = ContactableSteeringWheelRobot.createPolarisSteeringWheel(x, y, z, Math.toRadians(yawInDegrees), Math.toRadians(pitchInDegrees));
      wheelRobot.createSteeringWheelRobot();
      wheelRobot.createAvailableContactPoints(1, 30, forceVectorScale, true);
      wheelRobot.addSpinnerHandle();
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
   public List<ContactableSteeringWheelRobot> getEnvironmentRobots()
   {
      ArrayList<ContactableSteeringWheelRobot> ret = new ArrayList<>();
      ret.add(wheelRobot);
      return ret;
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
