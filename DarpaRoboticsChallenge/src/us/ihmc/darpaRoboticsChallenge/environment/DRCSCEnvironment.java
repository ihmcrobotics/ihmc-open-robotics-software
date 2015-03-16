package us.ihmc.darpaRoboticsChallenge.environment;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;

import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.GroundContactModel;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.robotController.ContactController;
import us.ihmc.simulationconstructionset.util.LinearGroundContactModel;
import us.ihmc.simulationconstructionset.util.environments.ContactableCylinderRobot;
import us.ihmc.simulationconstructionset.util.environments.ContactableDoorRobot;
import us.ihmc.simulationconstructionset.util.environments.ContactableValveRobot;
import us.ihmc.simulationconstructionset.util.environments.SelectableObjectListener;
import us.ihmc.simulationconstructionset.util.environments.ValveType;
import us.ihmc.simulationconstructionset.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.util.ground.Contactable;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.math.geometry.RotationFunctions;

public class DRCSCEnvironment implements CommonAvatarEnvironmentInterface
{
   private final List<Robot> contactableRobots = new ArrayList<Robot>();
   private final CombinedTerrainObject3D combinedTerrainObject;
   private final double WALL_HEIGHT = 2.4384;
   private final ArrayList<ExternalForcePoint> contactPoints = new ArrayList<ExternalForcePoint>();

   //DRILL PARAMETERS

   //**************

   public DRCSCEnvironment()
   {
      combinedTerrainObject = new CombinedTerrainObject3D(getClass().getSimpleName());
      combinedTerrainObject.addTerrainObject(setUpGround("Ground"));
      addFullCourse3dModel();
      
      createDoor();
      createValve();
      createDrill();
   }

   private void addFullCourse3dModel()
   {
      combinedTerrainObject.getLinkGraphics().identity();
      combinedTerrainObject.getLinkGraphics().addModelFile("models/SCTestBed.obj");
   }

   private void createDrill()
   {
      Vector3d tableCenter = new Vector3d(-0.851, -6.833, 1.118);

      double drillHeight = 0.3;
      double drillRadius = 0.03;
      double drillMass = 1.5;
      double forceVectorScale = 1.0 / 500.0;

      RigidBodyTransform initialDrillTransform = new RigidBodyTransform(new AxisAngle4d(), tableCenter);
      ContactableCylinderRobot drillRobot = new ContactableCylinderRobot("drill", initialDrillTransform, drillRadius, drillHeight, drillMass,
            "models/drill.obj");
      final int groundContactGroupIdentifier = 0;
      drillRobot.createAvailableContactPoints(groundContactGroupIdentifier, 30, forceVectorScale, true);
      for (int i = 0; i < 4; i++)
      {
         double angle = i * 2.0 * Math.PI / 4.0;
         double x = 1.5 * drillRadius * Math.cos(angle);
         double y = 1.5 * drillRadius * Math.sin(angle);
         GroundContactPoint groundContactPoint = new GroundContactPoint("gc_drill_" + i, new Vector3d(x, y, 0.0), drillRobot);
         drillRobot.getRootJoints().get(0).addGroundContactPoint(groundContactPoint);
      }

      GroundContactModel groundContactModel = new LinearGroundContactModel(drillRobot, groundContactGroupIdentifier, 1422.0, 150.6, 50.0, 600.0,
            drillRobot.getRobotsYoVariableRegistry());
      groundContactModel.setGroundProfile3D(combinedTerrainObject);
      drillRobot.setGroundContactModel(groundContactModel);

      contactableRobots.add(drillRobot);
      
      combinedTerrainObject.addBox(-0.777, -6.916,-0.947, -6.746 , 1.061,1.08);

      
   }

   private void createDoor()
   {
      ContactableDoorRobot door = new ContactableDoorRobot("doorRobot", new Point3d());
      contactableRobots.add(door);
      door.createAvailableContactPoints(0, 15, 15, 0.02, true);
   }

   private void createValve()
   {
      double forceVectorScale = 1.0 / 50.0;

      String valveRobotName = "ValveRobot";
      ValveType valveType = ValveType.SMALL_VALVE;
      double x = -1.043;
      double y = -5.937;
      double z = 1.168;
      double yaw_degrees = 135;

      FramePose valvePose = new FramePose(ReferenceFrame.getWorldFrame());
      Point3d position = new Point3d(x, y, z);
      Quat4d orientation = new Quat4d();

      RotationFunctions.setQuaternionBasedOnYawPitchRoll(orientation, Math.toRadians(yaw_degrees), Math.toRadians(0), Math.toRadians(0));
      valvePose.setPose(position, orientation);

      ContactableValveRobot valve = new ContactableValveRobot(valveRobotName, valveType, 0.5, valvePose);

      valve.createValveRobot();
      valve.createAvailableContactPoints(1, 30, forceVectorScale, true);

      contactableRobots.add(valve);
   }

   private CombinedTerrainObject3D setUpGround(String name)
   {
      CombinedTerrainObject3D combinedTerrainObject = new CombinedTerrainObject3D(name);

      combinedTerrainObject.addBox(-5.0, -30.0, 5.0, 5.0, -0.05, 0.0, YoAppearance.DarkGray());
      combinedTerrainObject.addBox(-1.2192, -0.025, 0, 0.025, WALL_HEIGHT, YoAppearance.Beige());
      combinedTerrainObject.addBox(0.0 + ContactableDoorRobot.DEFAULT_DOOR_DIMENSIONS.x, -0.025, 1.2192 + ContactableDoorRobot.DEFAULT_DOOR_DIMENSIONS.x,
            0.025, WALL_HEIGHT, YoAppearance.Beige());
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
      return contactableRobots;
   }

   @Override
   public void createAndSetContactControllerToARobot()
   {
      ContactController contactController = new ContactController();
      contactController.setContactParameters(100000.0, 100.0, 0.5, 0.3);

      contactController.addContactPoints(contactPoints);

      for (Robot r : contactableRobots)
      {
         if (r instanceof Contactable)
            contactController.addContactable((Contactable) r);

      }
      contactableRobots.get(0).setController(contactController);
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
