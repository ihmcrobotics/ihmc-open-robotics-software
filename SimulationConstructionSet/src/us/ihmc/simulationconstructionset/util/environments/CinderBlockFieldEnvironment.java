package us.ihmc.simulationconstructionset.util.environments;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;

public class CinderBlockFieldEnvironment implements CommonAvatarEnvironmentInterface
{
   private final CombinedTerrainObject3D combinedTerrainObject3D = new CombinedTerrainObject3D(getClass().getSimpleName());
   private final List<List<FramePose>> cinderBlockPoses = new ArrayList<>();

   public CinderBlockFieldEnvironment()
   {
      combinedTerrainObject3D.addTerrainObject(DefaultCommonAvatarEnvironment.setUpGround("FlatGround"));
      combinedTerrainObject3D.addTerrainObject(DefaultCommonAvatarEnvironment.setUpCinderBlockFieldActual("CinderBlockField", 0.0, 0.0, cinderBlockPoses));
   }

   public List<List<FramePose>> getCinderBlockPoses()
   {
      return cinderBlockPoses;
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
