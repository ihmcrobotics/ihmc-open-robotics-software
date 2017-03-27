package us.ihmc.simulationConstructionSetTools.util.environments;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.appearance.YoAppearanceTexture;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.GroundContactModel;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.robotController.ContactController;
import us.ihmc.simulationconstructionset.util.LinearGroundContactModel;
import us.ihmc.simulationconstructionset.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;

public class HandDrillManipulationEnvironment implements CommonAvatarEnvironmentInterface
{
   private final ArrayList<ContactableRobot> robots = new ArrayList<ContactableRobot>();
   private final ContactableCylinderRobot drillRobot;
   private final CombinedTerrainObject3D combinedTerrainObject;

   private final ArrayList<ExternalForcePoint> contactPoints = new ArrayList<ExternalForcePoint>();
   
   private final double tableHeight = 1.4;
   private final Vector3D tableCenter = new Vector3D(1.0, 0.0, tableHeight / 2.0);
   private final Vector2D wallPosition = new Vector2D(0.0, -2.05);
   
   private final double drillHeight = 0.3;
   private final double drillRadius = 0.025;
   private final double drillMass = 1.5;
   
   public HandDrillManipulationEnvironment()
   {
      double forceVectorScale = 1.0 / 500.0;

      combinedTerrainObject = new CombinedTerrainObject3D(getClass().getSimpleName());
      combinedTerrainObject.addTerrainObject(setUpGround("Ground", tableCenter, 0.1));
      
      RigidBodyTransform initialDrillTransform = new RigidBodyTransform(new AxisAngle(), tableCenter);
      drillRobot = new ContactableCylinderRobot("drill", initialDrillTransform , drillRadius, drillHeight, drillMass, "models/drill.obj");
      final int groundContactGroupIdentifier = 0;
      drillRobot.createAvailableContactPoints(groundContactGroupIdentifier, 30, forceVectorScale, true);
      for (int i = 0; i < 4; i++)
      {
         double angle = i * 2.0 * Math.PI / 4.0;
         double x = 1.5 * drillRadius * Math.cos(angle);
         double y = 1.5 * drillRadius * Math.sin(angle);
         GroundContactPoint groundContactPoint = new GroundContactPoint("gc_drill_" + i, new Vector3D(x, y, 0.0), drillRobot);
         drillRobot.getRootJoints().get(0).addGroundContactPoint(groundContactPoint);
      }

      GroundContactModel groundContactModel = new LinearGroundContactModel(drillRobot, groundContactGroupIdentifier,1422.0, 150.6, 50.0, 600.0, drillRobot.getRobotsYoVariableRegistry());
      groundContactModel.setGroundProfile3D(combinedTerrainObject);
      drillRobot.setGroundContactModel(groundContactModel);
      
      robots.add(drillRobot);
   }
   
   private CombinedTerrainObject3D setUpGround(String name, Vector3D tableCenter, double tableLength)
   {
      CombinedTerrainObject3D combinedTerrainObject = new CombinedTerrainObject3D(name);
      YoAppearanceTexture texture = new YoAppearanceTexture("Textures/gridGroundProfile.png");
      combinedTerrainObject.addBox(-10.0, -10.0, 10.0, 10.0, -0.05, 0.0, texture);
      combinedTerrainObject.addBox(tableCenter.getX()-tableLength , tableCenter.getY()-tableLength, tableCenter.getX()+tableLength, tableCenter.getY()+tableLength , tableCenter.getZ());
      combinedTerrainObject.addBox(wallPosition.getX() - 1.0, wallPosition.getY() - 0.05, wallPosition.getX() + 1.0, wallPosition.getY() + 0.05, 2.0);
      
      return combinedTerrainObject;
   }
   
   public ContactableCylinderRobot getDrillRobot()
   {
      return drillRobot;
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
