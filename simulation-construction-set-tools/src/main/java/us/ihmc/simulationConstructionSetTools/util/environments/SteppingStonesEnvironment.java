package us.ihmc.simulationConstructionSetTools.util.environments;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.simulationConstructionSetTools.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.ground.RotatableBoxTerrainObject;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;

public class SteppingStonesEnvironment implements CommonAvatarEnvironmentInterface
{
   private static final double blockHeight = 0.125;
   private static final double blockFaceHeight = 0.25;
   private static final AppearanceDefinition groundColor = YoAppearance.Texture("Textures/water.png");
   private static final AppearanceDefinition steppingStoneColor = YoAppearance.DarkGray();

   private final ReferenceFrame baseBlockFrame;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final CombinedTerrainObject3D combinedTerrainObject3D = new CombinedTerrainObject3D(getClass().getSimpleName());

   private final List<FramePoint3D> blockPositions = new ArrayList<>();

   public SteppingStonesEnvironment()
   {
      addGround();
      double yaw = Math.toRadians(45.0);

      RigidBodyTransform transform = new RigidBodyTransform();
      transform.setTranslation(0.5, 0.5, blockFaceHeight);
      transform.appendYawRotation(yaw);
      transform.invert();
      baseBlockFrame = ReferenceFrame.constructFrameWithUnchangingTransformFromParent("baseFrame", ReferenceFrame.getWorldFrame(), transform);

      FramePoint3D baseBlockPosition = new FramePoint3D(baseBlockFrame);
      FramePoint3D block1Position = new FramePoint3D(baseBlockFrame, 0.925, 0.125, 0.0);
      FramePoint3D block2Position = new FramePoint3D(baseBlockFrame, 1.1, -0.375, 0.0);
      FramePoint3D block3Position = new FramePoint3D(baseBlockFrame, 1.35, 0.05, 0.0);
      FramePoint3D block4Position = new FramePoint3D(baseBlockFrame, 1.65, 0.2, 0.0);
      FramePoint3D block5Position = new FramePoint3D(baseBlockFrame, 1.65, -0.175, 0.0);
      FramePoint3D block6Position = new FramePoint3D(baseBlockFrame, 1.975, 0.175, 0.0);
      FramePoint3D block7Position = new FramePoint3D(baseBlockFrame, 2.0, -0.2, 0.0);
      FramePoint3D endBlockPosition = new FramePoint3D(baseBlockFrame, 2.9, 0.0, 0.0);

      baseBlockPosition.changeFrame(worldFrame);
      block1Position.changeFrame(worldFrame);
      block2Position.changeFrame(worldFrame);
      block3Position.changeFrame(worldFrame);
      block4Position.changeFrame(worldFrame);
      block5Position.changeFrame(worldFrame);
      block6Position.changeFrame(worldFrame);
      block7Position.changeFrame(worldFrame);
      endBlockPosition.changeFrame(worldFrame);

      blockPositions.add(baseBlockPosition);
      blockPositions.add(block1Position);
      blockPositions.add(block2Position);
      blockPositions.add(block3Position);
      blockPositions.add(block4Position);
      blockPositions.add(block5Position);
      blockPositions.add(block6Position);
      blockPositions.add(block7Position);
      blockPositions.add(endBlockPosition);

      addBlock(yaw, baseBlockPosition, new Vector3D(1.5, 2.0, 0.25), steppingStoneColor);
      addBlock(yaw, block1Position, new Vector3D(0.25, 0.25, 0.25), steppingStoneColor);
      addBlock(yaw, block2Position, new Vector3D(0.6, 0.25, 0.25), steppingStoneColor);
      addBlock(yaw, block3Position, new Vector3D(0.2, 0.2, 0.25), steppingStoneColor);
      addBlock(yaw, block4Position, new Vector3D(0.3, 0.3, 0.25), steppingStoneColor);
      addBlock(yaw, block5Position, new Vector3D(0.2, 0.2, 0.25), steppingStoneColor);
      addBlock(yaw, block6Position, new Vector3D(0.25, 0.25, 0.25), steppingStoneColor);
      addBlock(yaw, block7Position, new Vector3D(0.3, 0.25, 0.25), steppingStoneColor);
      addBlock(yaw, endBlockPosition, new Vector3D(1.5, 2.0, 0.25), steppingStoneColor);
   }

   public List<FramePoint3D> getBlockPositions()
   {
      return blockPositions;
   }

   public ReferenceFrame getBaseBlockFrame()
   {
      return baseBlockFrame;
   }

   public Point3D getStartPosition()
   {
      return new Point3D(0.5, 0.5, 0.25);
   }

   public double getStartYaw()
   {
      return Math.toRadians(45.0);
   }

   private void addGround()
   {
      Point3D position = new Point3D(0.0, 0.0, -0.05 - blockHeight / 2.0);
      Vector3D dimensions = new Vector3D(30.0, 30.0, 0.1);
      addBlock(0.0, position, dimensions, groundColor);
   }

   private void addBlock(double yaw, Point3DBasics facePosition, Vector3D dimensions, AppearanceDefinition color)
   {
      facePosition.subZ(blockFaceHeight / 2.0);
      AxisAngle orientation = new AxisAngle(new Vector3D(0.0, 0.0, 1.0), yaw);
      RigidBodyTransform blockPose = new RigidBodyTransform(orientation, facePosition);
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
