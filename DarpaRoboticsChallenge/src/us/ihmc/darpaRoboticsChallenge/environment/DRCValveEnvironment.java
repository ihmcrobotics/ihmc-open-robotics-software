package us.ihmc.darpaRoboticsChallenge.environment;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import us.ihmc.darpaRoboticsChallenge.ContactController;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.util.environments.ContactableValveRobot;
import us.ihmc.simulationconstructionset.util.environments.SelectableObjectListener;
import us.ihmc.simulationconstructionset.util.environments.ValveType;
import us.ihmc.simulationconstructionset.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RotationFunctions;

public class DRCValveEnvironment implements CommonAvatarEnvironmentInterface
{
   private final List<ContactableValveRobot> valveRobot = new ArrayList<ContactableValveRobot>();
   private final CombinedTerrainObject3D combinedTerrainObject;

   private final ArrayList<ExternalForcePoint> contactPoints = new ArrayList<ExternalForcePoint>();

   public DRCValveEnvironment()
   {
      double forceVectorScale = 1.0 / 50.0;

      combinedTerrainObject = new CombinedTerrainObject3D(getClass().getSimpleName());
      combinedTerrainObject.addTerrainObject(setUpGround("Ground"));

      valveRobot.add(createValve());

      for (int i = 0; i < valveRobot.size(); i++)
      {
         ContactableValveRobot valve = valveRobot.get(i);
         valve.createValveRobot();
         valve.createAvailableContactPoints(1, 30, forceVectorScale, true);
      }
   }

   private ContactableValveRobot createValve()
   {
      FramePose valvePose = new FramePose(ReferenceFrame.getWorldFrame());
      Point3d position = new Point3d(0.5, 0.0, 1.0);
      Quat4d orientation = new Quat4d();

      RotationFunctions.setQuaternionBasedOnYawPitchRoll(orientation, Math.toRadians(0), Math.toRadians(0), Math.toRadians(0));
      valvePose.setPose(position, orientation);

      ContactableValveRobot valve = new ContactableValveRobot("valveRobot",ValveType.BIG_VALVE,3.5,valvePose);

      return valve;

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
      return valveRobot;
   }

   @Override
   public void createAndSetContactControllerToARobot()
   {
      ContactController contactController = new ContactController();
      contactController.setContactParameters(10000.0, 1000.0, 0.5, 0.3);
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
