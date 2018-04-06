package us.ihmc.simulationConstructionSetTools.util.environments;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.Box3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.util.ground.RotatableBoxTerrainObject;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;

import java.util.List;

public class SteppingStonesEnvironment implements CommonAvatarEnvironmentInterface
{
   private static final double blockHeight = 0.1;
   private static final AppearanceDefinition groundColor = YoAppearance.Texture("Textures/water.png");

   private final CombinedTerrainObject3D combinedTerrainObject3D = new CombinedTerrainObject3D(getClass().getSimpleName());

   public SteppingStonesEnvironment()
   {
      //combinedTerrainObject3D.addTerrainObject(DefaultCommonAvatarEnvironment.setUpGround("FlatGround"));
      //combinedTerrainObject3D.addTerrainObject(DefaultCommonAvatarEnvironment.setUpPath4DRCTrialsTrainingWalkingCourse("SteppingStones"));
      addGround();
      addBlock(0.0, new Vector3D(0.0, 0.0, 1.0), new Vector3D(1.0, 1.0, 1.0), YoAppearance.Red());
   }

   private void addGround()
   {
      Vector3D position = new Vector3D(0.0, 0.0, -0.05 - blockHeight / 2.0);
      Vector3D dimensions = new Vector3D(30.0, 30.0, 0.1);
      addBlock(0.0, position, dimensions, groundColor);
   }

   private void addBlock(double yaw, Vector3D position, Vector3D dimensions, AppearanceDefinition color)
   {
      AxisAngle orientation = new AxisAngle(new Vector3D(0.0, 0.0, 1.0), yaw);
      RigidBodyTransform blockPose = new RigidBodyTransform(orientation, position);
      Box3D block = new Box3D(blockPose, dimensions.getX(), dimensions.getY(), dimensions.getZ());
      combinedTerrainObject3D.addTerrainObject(new RotatableBoxTerrainObject(block, color));
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
