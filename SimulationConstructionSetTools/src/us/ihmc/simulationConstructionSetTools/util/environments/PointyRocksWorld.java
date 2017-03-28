package us.ihmc.simulationConstructionSetTools.util.environments;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.shapes.Box3d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.util.ground.RotatableBoxTerrainObject;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;

public class PointyRocksWorld implements CommonAvatarEnvironmentInterface
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   public enum PointyRocksType {
      SINGLE_LINE_BALANCE,
      LINES,
      POINT,
      PARTIAL
   }

   private final PointyRocksType type;

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

   public PointyRocksWorld(PointyRocksType type, int steps)
   {
      this.steps = steps;
      this.type = type;

      addGround();

      switch (type)
      {
      case SINGLE_LINE_BALANCE:
         setupSingleLine();
         break;
      case LINES:
         setupLines();
         break;
      case POINT:
         setupPoint();
         break;
      default:
         throw new RuntimeException("unimplemented poity rocks type");
      }
   }

   private void setupSingleLine()
   {
      addStartBlock();

      Vector2D linePosition = new Vector2D(0.4, -0.16);
      addLine(linePosition, 0.0);
   }

   private void setupPoint()
   {
      addStartBlock();
      double step = 0.5;

      Vector2D position = new Vector2D(length/2.0 + step/2.0, 0.15);
      Vector2D dimensions = new Vector2D(0.04, 0.04);

      addBlock(0.0, position, dimensions);
      stepLocations.add(new FramePoint(worldFrame, position.getX(), position.getY(), 0.0));

      addFinalBlock(step + length);
   }

   private void setupLines()
   {
      addStartBlock();
      for (int i = 0; i < steps; i++)
      {
         double y = i%2 == 0 ? -0.15 : 0.15;
         addLine(new Vector2D(step*i + length/2.0 + step/2.0, y), (random.nextDouble()-0.5) * 180.0);
      }
      addFinalBlock(step*steps + length);
   }

   private void addFinalBlock(double distance)
   {
      Vector3D position = new Vector3D(distance, 0.0, -height/2.0);
      Vector3D dimensions = new Vector3D(length, 0.6, height);
      addBlock(0.0, position, dimensions, blockColor);

      double y = steps%2 == 0 ? -0.15 : 0.15;
      stepLocations.add(new FramePoint(worldFrame, distance, y, 0.0));
      stepLocations.add(new FramePoint(worldFrame, distance, -y, 0.0));
   }

   private void addGround()
   {
      Vector3D position = new Vector3D(0.0, 0.0, -0.05-height);
      Vector3D dimensions = new Vector3D(30.0, 30.0, 0.1);
      addBlock(0.0, position, dimensions, groundColor);
   }

   private void addStartBlock()
   {
      Vector3D position = new Vector3D(0.0, 0.0, -height/2.0);
      Vector3D dimensions = new Vector3D(length, 0.6, height);
      addBlock(0.0, position, dimensions, blockColor);
   }

   private void addLine(Vector2D position, double yawDegree)
   {
      double yaw = yawDegree * Math.PI/180.0;
      double blockLength = (step+0.1) / Math.max(Math.sin(Math.abs(yaw)), Math.cos(yaw));

      Vector3D position3d = new Vector3D(position.getX(), position.getY(), -height/2.0);
      Vector3D dimensions = new Vector3D(blockLength, linbeWidth, height);
      addBlock(yaw, position3d, dimensions, blockColor);

      stepLocations.add(new FramePoint(worldFrame, position.getX(), position.getY(), 0.0));
   }

   private void addBlock(double yaw, Vector2D position, Vector2D dimensions)
   {
      Vector3D position3d = new Vector3D(position.getX(), position.getY(), -height/2.0);
      Vector3D dimensions3d = new Vector3D(dimensions.getX(), dimensions.getY(), height);
      addBlock(yaw, position3d, dimensions3d, blockColor);
   }

   private void addBlock(double yaw, Vector3D position, Vector3D dimensions, AppearanceDefinition color)
   {
      AxisAngle orientation = new AxisAngle(new Vector3D(0.0, 0.0, 1.0), yaw);
      RigidBodyTransform blockPose = new RigidBodyTransform(orientation, position);
      Box3d block = new Box3d(blockPose, dimensions.getX(), dimensions.getY(), dimensions.getZ());
      terrainObject.addTerrainObject(new RotatableBoxTerrainObject(block, color));
   }

   public ArrayList<FramePoint> getStepLocations()
   {
      return stepLocations;
   }

   public void setupCamera(Point3D cameraFixToPack, Point3D cameraPositionToPack)
   {
      switch (type)
      {
      case SINGLE_LINE_BALANCE:
         cameraFixToPack.set(0.0, 0.0, 1.0);
         cameraPositionToPack.set(-10.0, 0.0, 5.0);
         break;
      case LINES:
         cameraFixToPack.set((step*steps + length)/2.0, 0.0, 1.0);
         cameraPositionToPack.set((step*steps + length)/2.0, -10.0, 5.0);
         break;
      case POINT:
         cameraFixToPack.set((length + 0.5)/2.0, 0.0, 1.0);
         cameraPositionToPack.set((length + 0.5)/2.0, -10.0, 5.0);
         break;
      default:
         throw new RuntimeException("unimplemented poity rocks type");
      }
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
