package us.ihmc.darpaRoboticsChallenge.environment;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Vector3d;

import com.sun.org.apache.regexp.internal.recompile;

import us.ihmc.graphics3DAdapter.GroundProfile3D;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearanceTexture;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.GroundContactModel;
import us.ihmc.simulationconstructionset.robotController.ContactController;
import us.ihmc.simulationconstructionset.util.LinearGroundContactModel;
import us.ihmc.simulationconstructionset.util.environments.ContactableCylinderRobot;
import us.ihmc.simulationconstructionset.util.environments.ContactableRobot;
import us.ihmc.simulationconstructionset.util.environments.SelectableObjectListener;
import us.ihmc.simulationconstructionset.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.util.ground.FlatGroundProfile;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;

public class DRCDrillEnvironment implements CommonAvatarEnvironmentInterface
{
   private final ArrayList<ContactableRobot> robots = new ArrayList<ContactableRobot>();  
   private final CombinedTerrainObject3D combinedTerrainObject;

   private final ArrayList<ExternalForcePoint> contactPoints = new ArrayList<ExternalForcePoint>();

   public DRCDrillEnvironment()
   {
      this(0.5, 0.0, 1.0, 0.0);
   }
   
   public DRCDrillEnvironment(double valveX, double valveY, double valveZ, double valveYaw_degrees)
   {
      double forceVectorScale = 1.0 / 50.0;

      combinedTerrainObject = new CombinedTerrainObject3D(getClass().getSimpleName());
      Vector3d tableCenter = new Vector3d(1.0, 0.0, 0.7);
      combinedTerrainObject.addTerrainObject(setUpGround("Ground", tableCenter, 0.1));

      ContactableCylinderRobot robot = new ContactableCylinderRobot("drill", new RigidBodyTransform(new AxisAngle4d(), tableCenter), 0.03, 0.30, 1.5, "models/drill.obj");
      final int groundContactGroupIdentifier = 0;
      robot.createAvailableContactPoints(groundContactGroupIdentifier, 30, forceVectorScale, true);

      GroundContactModel groundContactModel = new LinearGroundContactModel(robot, groundContactGroupIdentifier,1422.0, 150.6, 50.0, 600.0, robot.getRobotsYoVariableRegistry());
      groundContactModel.setGroundProfile3D(combinedTerrainObject);
      robot.setGroundContactModel(groundContactModel);
      
      robots.add(robot);
   }
   
   private CombinedTerrainObject3D setUpGround(String name, Vector3d tableCenter, double tableLength)
   {
      CombinedTerrainObject3D combinedTerrainObject = new CombinedTerrainObject3D(name);
      YoAppearanceTexture texture = new YoAppearanceTexture("Textures/gridGroundProfile.png");
      combinedTerrainObject.addBox(-10.0, -10.0, 10.0, 10.0, -0.05, 0.0, texture);
      combinedTerrainObject.addBox(tableCenter.x-tableLength , tableCenter.y-tableLength, tableCenter.x+tableLength, tableCenter.y+tableLength , tableCenter.z);

      return combinedTerrainObject;
   }

   @Override
   public TerrainObject3D getTerrainObject3D()
   {
      return combinedTerrainObject;
   }

   @Override
   public List<ContactableRobot> getEnvironmentRobots()
   {
      return robots;
   }

   @Override
   public void createAndSetContactControllerToARobot()
   {
      ContactController contactController = new ContactController();
      contactController.setContactParameters(10000.0, 1000.0, 0.5, 0.3);
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
      // TODO Auto-generated method stub

   }
}
