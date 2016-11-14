package us.ihmc.simulationconstructionset.util.environments;

import java.util.List;

import javax.vecmath.Vector3d;

import us.ihmc.graphics3DDescription.appearance.YoAppearanceTexture;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.geometry.shapes.Box3d;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.util.ground.RotatableBoxTerrainObject;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;

public class FiducialsFlatGroundEnvironment implements CommonAvatarEnvironmentInterface
{
   private final CombinedTerrainObject3D combinedTerrainObject;
   
   public enum Fiducial
   {
      FIDUCIAL50,
      FIDUCIAL100,
      FIDUCIAL150,
      FIDUCIAL200,
      FIDUCIAL250,
      FIDUCIAL300,
      FIDUCIAL350,
      FIDUCIAL400,
      FIDUCIAL450;
      
      public static final Fiducial[] values = values();
      
      public String getPathString()
      {
         return "fiducials/" + name().toLowerCase() + ".png";
      }
   }

   public FiducialsFlatGroundEnvironment()
   {
      combinedTerrainObject = new CombinedTerrainObject3D(getClass().getSimpleName());
      combinedTerrainObject.addTerrainObject(DefaultCommonAvatarEnvironment.setUpGround("Ground"));

      double angle = 0.0;
      double radius = 3.0;
      for (Fiducial fiducial : Fiducial.values)
      {
         CombinedTerrainObject3D fiducualTerrainObject = addFiducial(radius * Math.cos(angle), radius * Math.sin(angle), angle, fiducial);
         
         combinedTerrainObject.addTerrainObject(fiducualTerrainObject);
         
         angle += 2.0 * Math.PI / Fiducial.values.length;
      }
   }

   private CombinedTerrainObject3D addFiducial(double x, double y, double rotation, Fiducial fiducial)
   {
      YoAppearanceTexture fiducialTexture = new YoAppearanceTexture(fiducial.getPathString());
      double boxSideLength = 0.3;

      CombinedTerrainObject3D fiducualTerrainObject = new CombinedTerrainObject3D(fiducial.name());
      
      RigidBodyTransform location = new RigidBodyTransform();
      location.setRotationEulerAndZeroTranslation(Math.toRadians(90.0), 0.0, rotation - Math.toRadians(90.0));
      location.setTranslation(new Vector3d(x, y, 0.7));
      
      RotatableBoxTerrainObject newBox = new RotatableBoxTerrainObject(new Box3d(location, boxSideLength, boxSideLength, boxSideLength), fiducialTexture);
      fiducualTerrainObject.addTerrainObject(newBox);
      return fiducualTerrainObject;
   }

   @Override
   public TerrainObject3D getTerrainObject3D()
   {
      return combinedTerrainObject;
   }

   @Override
   public List<Robot> getEnvironmentRobots()
   {
      return null;
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
}
