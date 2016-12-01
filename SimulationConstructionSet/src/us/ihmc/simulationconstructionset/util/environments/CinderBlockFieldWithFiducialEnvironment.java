package us.ihmc.simulationconstructionset.util.environments;

import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;

import javax.vecmath.Vector3d;
import java.util.ArrayList;
import java.util.List;

public class CinderBlockFieldWithFiducialEnvironment implements CommonAvatarEnvironmentInterface
{
   private final CombinedTerrainObject3D combinedTerrainObject3D = new CombinedTerrainObject3D(getClass().getSimpleName());

   public CinderBlockFieldWithFiducialEnvironment()
   {
      combinedTerrainObject3D.addTerrainObject(DefaultCommonAvatarEnvironment.setUpGround("FlatGround"));
      combinedTerrainObject3D.addTerrainObject(DefaultCommonAvatarEnvironment.setUpCinderBlockFieldActual("CinderBlockField", 0.0, 0.0, new ArrayList<List<FramePose>>()));
      combinedTerrainObject3D.addTerrainObject(DefaultCommonAvatarEnvironment.addFiducial(new Vector3d(5.0, 0.0, 1.7), 0.0, Fiducial.FIDUCIAL50));
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
