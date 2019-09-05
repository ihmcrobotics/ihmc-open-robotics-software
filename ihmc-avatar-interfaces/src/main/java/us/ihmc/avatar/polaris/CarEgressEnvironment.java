package us.ihmc.avatar.polaris;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.appearance.YoAppearanceTexture;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationConstructionSetTools.util.environments.SelectableObjectListener;
import us.ihmc.simulationConstructionSetTools.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.ground.RotatableBoxTerrainObject;
import us.ihmc.simulationconstructionset.util.ground.RotatableCinderBlockTerrainObject;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;

public class CarEgressEnvironment implements CommonAvatarEnvironmentInterface
{
   private final double edgeOfStepX = -0.8, edgeOfStepY = 0.5;
   private final static Point3D stepDimensions = new Point3D(1.0, 0.3, 0.2);
   private final static Point3D carDimensions = new Point3D(1.0, 0.6, 0.4);
   private final static Vector3D polarisPosition = new Vector3D(-0.4, 1.5, 0.0);
   private final CombinedTerrainObject3D terrain = new CombinedTerrainObject3D("drcCarEgressTerrain");
   private final List<Robot> robots = new ArrayList<Robot>();
   
   public CarEgressEnvironment()
   {
      RigidBodyTransform locationStep = new RigidBodyTransform();
      locationStep.setTranslation(new Vector3D(edgeOfStepX + 0.5 * stepDimensions.getX(), edgeOfStepY + 0.5 * stepDimensions.getY(), stepDimensions.getZ() / 2.0));
      Box3D stepBox = new Box3D(locationStep, stepDimensions.getX(), stepDimensions.getY(), stepDimensions.getZ());
      terrain.addTerrainObject(new RotatableCinderBlockTerrainObject(stepBox, YoAppearance.DarkGray()));   
      
      RigidBodyTransform locationCar = new RigidBodyTransform();
      locationCar.setTranslation(new Vector3D(edgeOfStepX + 0.5 * stepDimensions.getX(), edgeOfStepY + stepDimensions.getY() + 0.5 * carDimensions.getY(), carDimensions.getZ() / 2.0));
      Box3D carBox = new Box3D(locationCar, carDimensions.getX(), carDimensions.getY(), carDimensions.getZ());      
      terrain.addTerrainObject(new RotatableCinderBlockTerrainObject(carBox, YoAppearance.DarkGray()));   
      
      terrain.addTerrainObject(setUpGround("ground"));
      
      RigidBodyTransform polarisTransform = new RigidBodyTransform();
      polarisTransform.setTranslation(polarisPosition);
      robots.add(new PolarisRobot("polaris", polarisTransform));
   }
   
   @Override
   public TerrainObject3D getTerrainObject3D()
   {
      return terrain;
   }

   @Override
   public List<? extends Robot> getEnvironmentRobots()
   {
      return robots;
   }

   @Override
   public void createAndSetContactControllerToARobot()
   {
   }

   @Override
   public void addContactPoints(List<? extends ExternalForcePoint> externalForcePoints)
   {      
   }

   @Override
   public void addSelectableListenerToSelectables(SelectableObjectListener selectedListener)
   {      
   }
   
   private static CombinedTerrainObject3D setUpGround(String name)
   {
      CombinedTerrainObject3D combinedTerrainObject = new CombinedTerrainObject3D(name);

      YoAppearanceTexture texture = new YoAppearanceTexture("Textures/brick.png");

      RigidBodyTransform location = new RigidBodyTransform();
      location.setTranslation(new Vector3D(0, 0, -0.5));

      RotatableBoxTerrainObject newBox = new RotatableBoxTerrainObject(new Box3D(location, 10, 10, 1), texture);
      combinedTerrainObject.addTerrainObject(newBox);

      return combinedTerrainObject;
   }
}
