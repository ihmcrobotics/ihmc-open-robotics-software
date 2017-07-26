package us.ihmc.simulationConstructionSetTools.util.environments;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.Box3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.util.ground.RotatableBoxTerrainObject;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;

import java.util.ArrayList;
import java.util.List;

public class SmallStepDownEnvironment implements CommonAvatarEnvironmentInterface
{
   private static final double length = 0.35;
   private static final double blockHeight = 0.1;
   private static final AppearanceDefinition groundColor = YoAppearance.Texture("Textures/water.png");
   private final AppearanceDefinition blockColor = YoAppearance.Grey();

   private final String name = getClass().getSimpleName();
   private final CombinedTerrainObject3D terrainObject = new CombinedTerrainObject3D(name);

   private final int numberOfStepsDown;
   private final double stepDownHeight;
   private final double stepDepth;

   public SmallStepDownEnvironment(int numberOfStepsDown, double stepDepth, double stepDownHeight)
   {
      this.numberOfStepsDown = numberOfStepsDown;
      this.stepDownHeight = stepDownHeight;
      this.stepDepth = stepDepth;

      addGround();
      addStartBlock();

      for (int i = 0; i < numberOfStepsDown; i++)
         addBlockDown(i);

      addEndBlock();
   }

   private void addGround()
   {
      Vector3D position = new Vector3D(0.0, 0.0, -0.05 - (2 + numberOfStepsDown) * stepDownHeight - blockHeight / 2.0);
      Vector3D dimensions = new Vector3D(30.0, 30.0, 0.1);
      addBlock(0.0, position, dimensions, groundColor);
   }

   private void addStartBlock()
   {
      Vector3D position = new Vector3D(0.0, 0.0, -blockHeight / 2.0);
      Vector3D dimensions = new Vector3D(length, 0.6, blockHeight);
      addBlock(0.0, position, dimensions, blockColor);
   }

   private void addEndBlock()
   {
      double endBlockLength = 5.0 * stepDepth;
      Vector3D position = new Vector3D(0.5 * (length + endBlockLength) + (numberOfStepsDown - 0.5) * stepDepth, 0.0, -numberOfStepsDown * stepDownHeight - blockHeight / 2.0);
      Vector3D dimensions = new Vector3D(endBlockLength, 0.6, blockHeight);
      addBlock(0.0, position, dimensions, blockColor);
   }

   private void addBlockDown(int stepNumber)
   {
      Vector3D position = new Vector3D(0.5 * length + (stepNumber + 0.5) * stepDepth, 0.0, -blockHeight / 2.0 - (1 + stepNumber) * stepDownHeight);
      Vector3D dimensions = new Vector3D(stepDepth, 0.6, blockHeight);
      addBlock(0.0, position, dimensions, blockColor);
   }


   private void addBlock(double yaw, Vector2D position, Vector2D dimensions)
   {
      Vector3D position3d = new Vector3D(position.getX(), position.getY(), -blockHeight/2.0);
      Vector3D dimensions3d = new Vector3D(dimensions.getX(), dimensions.getY(), blockHeight);
      addBlock(yaw, position3d, dimensions3d, blockColor);
   }

   private void addBlock(double yaw, Vector3D position, Vector3D dimensions, AppearanceDefinition color)
   {
      AxisAngle orientation = new AxisAngle(new Vector3D(0.0, 0.0, 1.0), yaw);
      RigidBodyTransform blockPose = new RigidBodyTransform(orientation, position);
      Box3D block = new Box3D(blockPose, dimensions.getX(), dimensions.getY(), dimensions.getZ());
      terrainObject.addTerrainObject(new RotatableBoxTerrainObject(block, color));
   }

   @Override
   public TerrainObject3D getTerrainObject3D()
   {
      return terrainObject;
   }

   @Override
   public ArrayList<Robot> getEnvironmentRobots()
   {
      return new ArrayList<Robot>();
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
