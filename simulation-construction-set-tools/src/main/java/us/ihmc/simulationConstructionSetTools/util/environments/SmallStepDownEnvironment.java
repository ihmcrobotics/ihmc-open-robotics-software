package us.ihmc.simulationConstructionSetTools.util.environments;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.simulationConstructionSetTools.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.ground.RotatableBoxTerrainObject;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;

public class SmallStepDownEnvironment implements CommonAvatarEnvironmentInterface
{
   private static final double length = 0.35;
   private static final double blockHeight = 0.1;
   private static final double blockWidth = 1.5;
   private static final AppearanceDefinition groundColor = YoAppearance.Texture("Textures/water.png");
   private final AppearanceDefinition blockColor = YoAppearance.Grey();

   private final String name = getClass().getSimpleName();
   private final CombinedTerrainObject3D terrainObject = new CombinedTerrainObject3D(name);

   private final int numberOfStepsDown;

   public SmallStepDownEnvironment(int numberOfStepsDown, double stepDepth, double stepDownHeight)
   {
      this.numberOfStepsDown = numberOfStepsDown;

      addGround(numberOfStepsDown, stepDownHeight);
      addStartBlock();

      for (int i = 0; i < numberOfStepsDown; i++)
         addBlockDown(i, stepDepth, stepDownHeight);

      addEndBlock(numberOfStepsDown, stepDepth, stepDownHeight);
   }

   public SmallStepDownEnvironment(List<Double> heightField, List<Double> stepLength, double startBlockWidth, double startBlockHeight, double endBlockHeight)
   {
      this.numberOfStepsDown = heightField.size();

      double minHeight = Double.POSITIVE_INFINITY;
      for (int i = 0; i < numberOfStepsDown; i++)
         minHeight = Math.min(minHeight, heightField.get(i));

      addGround(minHeight);
      addStartBlock(startBlockWidth, startBlockHeight);

      double distanceTraveled = 0.0;
      for (int i = 0; i < numberOfStepsDown; i++)
      {
         double length = stepLength.get(i);
         addRandomBlock(distanceTraveled, heightField.get(i), length);

         distanceTraveled += length;
      }

      addEndBlock(distanceTraveled, 0.5, endBlockHeight);
   }

   private void addGround(double height)
   {
      Vector3D position = new Vector3D(0.0, 0.0, -0.05 + height - blockHeight / 2.0);
      Vector3D dimensions = new Vector3D(30.0, 30.0, 0.1);
      addBlock(0.0, position, dimensions, groundColor);
   }

   private void addGround(int numberOfStepsDown, double stepDownHeight)
   {
      Vector3D position = new Vector3D(0.0, 0.0, -0.05 - (2 + numberOfStepsDown) * stepDownHeight - blockHeight / 2.0);
      Vector3D dimensions = new Vector3D(30.0, 30.0, 0.1);
      addBlock(0.0, position, dimensions, groundColor);
   }

   private void addStartBlock()
   {
      addStartBlock(length, 0.0);
   }

   private void addStartBlock(double startBlockLength, double startBlockHeight)
   {
      Vector3D position = new Vector3D(0.0, 0.0, -blockHeight / 2.0 + startBlockHeight);
      Vector3D dimensions = new Vector3D(startBlockLength, blockWidth, blockHeight);
      addBlock(0.0, position, dimensions, blockColor);
   }

   private void addEndBlock(int numberOfStepsDown, double stepDepth, double stepDownHeight)
   {
      double endBlockLength = 5.0 * stepDepth;
      Vector3D position = new Vector3D(0.5 * (length + endBlockLength) + (numberOfStepsDown - 0.5) * stepDepth, 0.0, -numberOfStepsDown * stepDownHeight - blockHeight / 2.0);
      Vector3D dimensions = new Vector3D(endBlockLength, blockWidth, blockHeight);
      addBlock(0.0, position, dimensions, blockColor);
   }

   private void addEndBlock(double forwardPosition, double stepDepth, double endBlockHeight)
   {
      double endBlockLength = 5.0 * stepDepth;
      Vector3D position = new Vector3D(0.5 * (length + endBlockLength) + forwardPosition, 0.0, -blockHeight / 2.0 + endBlockHeight);
      Vector3D dimensions = new Vector3D(endBlockLength, blockWidth, blockHeight);
      addBlock(0.0, position, dimensions, blockColor);
   }

   private void addBlockDown(int stepNumber, double stepDepth, double stepDownHeight)
   {
      Vector3D position = new Vector3D(0.5 * length + (stepNumber + 0.5) * stepDepth, 0.0, -blockHeight / 2.0 - (1 + stepNumber) * stepDownHeight);
      Vector3D dimensions = new Vector3D(stepDepth, blockWidth, blockHeight);
      addBlock(0.0, position, dimensions, blockColor);
   }

   private void addRandomBlock(double forwardPosition, double height, double stepDepth)
   {
      Vector3D position = new Vector3D(0.5 * (length + stepDepth) + forwardPosition, 0.0, -blockHeight / 2.0 + height);
      Vector3D dimensions = new Vector3D(stepDepth, blockWidth, blockHeight);
      addBlock(0.0, position, dimensions, blockColor);
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
