package us.ihmc.darpaRoboticsChallenge.environment;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;

import us.ihmc.graphics3DAdapter.graphics.appearances.AppearanceDefinition;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.geometry.shapes.Box3d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.environments.SelectableObjectListener;
import us.ihmc.simulationconstructionset.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.util.ground.RotatableBoxTerrainObject;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;

public class PointyRocksWorld implements CommonAvatarEnvironmentInterface
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final String name = getClass().getSimpleName();
   private final AppearanceDefinition groundColor = YoAppearance.Texture("Textures/water.png");
   private final AppearanceDefinition blockColor = YoAppearance.Grey();
   private final CombinedTerrainObject3D terrainObject = new CombinedTerrainObject3D(name);

   private final double height = 0.2;
   private final Random random = new Random(752149823458L);
   private final double length = 0.35;
   private final int steps;
   private final double step = 0.3;
   private final double linbeWidth = 0.03;

   private final ArrayList<FramePoint> stepLocations = new ArrayList<>();

   public PointyRocksWorld(int steps)
   {
      this.steps = steps;

      addGround();
      addStartBlock();
      for (int i = 0; i < steps; i++)
      {
         double y = i%2 == 0 ? -0.15 : 0.15;
         addLine(new Vector2d(step*i + length/2.0 + step/2.0, y), (random.nextDouble()-0.5) * 180.0);
      }
      addFinalBlock(step*steps + length);
   }

   private void addFinalBlock(double distance)
   {
      Vector3d position = new Vector3d(distance, 0.0, -height/2.0);
      Vector3d dimensions = new Vector3d(length, 0.6, height);
      addBlock(0.0, position, dimensions, blockColor);

      double y = steps%2 == 0 ? -0.15 : 0.15;
      stepLocations.add(new FramePoint(worldFrame, distance, y, 0.0));
      stepLocations.add(new FramePoint(worldFrame, distance, -y, 0.0));
   }

   private void addGround()
   {
      Vector3d position = new Vector3d(0.0, 0.0, -0.05-height);
      Vector3d dimensions = new Vector3d(30.0, 30.0, 0.1);
      addBlock(0.0, position, dimensions, groundColor);
   }

   private void addStartBlock()
   {
      Vector3d position = new Vector3d(0.0, 0.0, -height/2.0);
      Vector3d dimensions = new Vector3d(length, 0.6, height);
      addBlock(0.0, position, dimensions, blockColor);
   }

   private void addLine(Vector2d position, double yawDegree)
   {
      double yaw = yawDegree * Math.PI/180.0;
      double blockLength = (step+0.1) / Math.max(Math.sin(Math.abs(yaw)), Math.cos(yaw));

      Vector3d position3d = new Vector3d(position.getX(), position.getY(), -height/2.0);
      Vector3d dimensions = new Vector3d(blockLength, linbeWidth, height);
      addBlock(yaw, position3d, dimensions, blockColor);

      stepLocations.add(new FramePoint(worldFrame, position.getX(), position.getY(), 0.0));
   }

   private void addBlock(double yaw, Vector3d position, Vector3d dimensions, AppearanceDefinition color)
   {
      AxisAngle4d orientation = new AxisAngle4d(new Vector3d(0.0, 0.0, 1.0), yaw);
      RigidBodyTransform blockPose = new RigidBodyTransform(orientation, position);
      Box3d block = new Box3d(blockPose, dimensions.getX(), dimensions.getY(), dimensions.getZ());
      terrainObject.addTerrainObject(new RotatableBoxTerrainObject(block, color));
   }

   public ArrayList<FramePoint> getStepLocations()
   {
      return stepLocations;
   }

   public void setupCamera(Point3d cameraFixToPack, Point3d cameraPositionToPack)
   {
      cameraFixToPack.set((step*steps + length)/2.0, 0.0, 1.0);
      cameraPositionToPack.set((step*steps + length)/2.0, -10.0, 5.0);
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
