package us.ihmc.simulationconstructionset.util.environments;

import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;

import javax.vecmath.Vector3d;
import java.util.List;

public class CinderBlockFieldWithFiducialEnvironment implements CommonAvatarEnvironmentInterface
{
   private final CombinedTerrainObject3D combinedTerrainObject3D = new CombinedTerrainObject3D(getClass().getSimpleName());
   private static final boolean SHORT_FIELD = false;

   public enum FiducialType
   {
      FIDUCIAL_50,
      VALVE
   }

   public CinderBlockFieldWithFiducialEnvironment(FiducialType fiducialType)
   {
      combinedTerrainObject3D.addTerrainObject(DefaultCommonAvatarEnvironment.setUpGround("FlatGround"));

      if(SHORT_FIELD)
         combinedTerrainObject3D.addTerrainObject(DefaultCommonAvatarEnvironment.setUpShortCinderBlockField("CinderBlockField", 0.0, 1.0));
      else
         combinedTerrainObject3D.addTerrainObject(DefaultCommonAvatarEnvironment.setUpCinderBlockFieldActual("CinderBlockField", 0.0, 0.0));

      switch (fiducialType)
      {
      case FIDUCIAL_50: combinedTerrainObject3D.addTerrainObject(DefaultCommonAvatarEnvironment.addFiducial(new Vector3d(12.0, 0.0, 1.7), 0.0, Fiducial.FIDUCIAL50)); break;
      case VALVE: combinedTerrainObject3D.addTerrainObject(DefaultCommonAvatarEnvironment.addValveTextureBox(new Vector3d(10.0, 0.0, 1.7), 0.0)); break;
      }
   }

   @Override
   public TerrainObject3D getTerrainObject3D()
   {
      return combinedTerrainObject3D;
   }

   @Override
   public List<? extends Robot> getEnvironmentRobots()
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
