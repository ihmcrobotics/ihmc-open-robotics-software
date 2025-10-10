package us.ihmc.footstepPlanning.bodyPath;

import gnu.trove.list.array.TIntArrayList;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FrameBox3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.footstepPlanning.AStarBodyPathPlannerParametersReadOnly;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCoordinateSystem;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicShape;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.perception.gpuMapping.HeightMapTools;
import us.ihmc.perception.gpuMapping.TerrainMapData;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoseUsingYawPitchRoll;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.List;

public class AStarBodyPathSmootherWaypoint
{
   static final double boxGroundOffset = 0.35;

   private static final AppearanceDefinition collisionBoxColor = YoAppearance.RGBColorFromHex(0x824e38);
   private final boolean visualize;

   private TerrainMapData terrainMapData;
   private final int waypointIndex;
   private final YoFramePoint3D initialWaypoint;
   private final YoFramePoseUsingYawPitchRoll waypoint;
   private final PoseReferenceFrame waypointFrame;
   private final YoBoolean isTurnPoint;
   private int previousCellKey, cellKey;

   private final TIntArrayList xSnapOffsets = new TIntArrayList();
   private final TIntArrayList ySnapOffsets = new TIntArrayList();

   private final TIntArrayList groundPlaneXOffsets = new TIntArrayList();
   private final TIntArrayList groundPlaneYOffsets = new TIntArrayList();
   private final YoDouble maxCollision;

   private static final double gradientGraphicScale = 0.17;
   private static final AppearanceDefinition smoothnessColor = YoAppearance.Blue();
   private static final AppearanceDefinition spacingColor = YoAppearance.Green();
   private static final AppearanceDefinition collisionColor = YoAppearance.Crimson();
   private static final AppearanceDefinition displacementColor = YoAppearance.White();
   private final YoGraphicPosition waypointGraphic, turnPointGraphic;

   private final YoFrameVector3D yoSmoothnessGradient;
   private final YoFrameVector3D yoEqualSpacingGradient;
   private final YoFrameVector3D yoCollisionGradient;
   private final YoFrameVector3D yoDisplacementGradient;

   private int pathSize;
   private final AStarBodyPathPlannerParametersReadOnly plannerParameters;
   private AStarBodyPathSmootherWaypoint[] waypoints;

   public AStarBodyPathSmootherWaypoint(int waypointIndex,
                                        AStarBodyPathPlannerParametersReadOnly plannerParameters,
                                        YoGraphicsListRegistry graphicsListRegistry,
                                        YoRegistry parentRegistry)
   {
      this.waypointIndex = waypointIndex;
      this.plannerParameters = plannerParameters;

      YoRegistry registry = new YoRegistry("Waypoint" + waypointIndex);
      maxCollision = new YoDouble("maxCollision" + waypointIndex, registry);
      isTurnPoint = new YoBoolean("isTurnPoint" + waypointIndex, registry);

      visualize = parentRegistry != null;
      waypoint = new YoFramePoseUsingYawPitchRoll("waypoint" + waypointIndex, ReferenceFrame.getWorldFrame(), registry);
      initialWaypoint = new YoFramePoint3D("initWaypoint" + waypointIndex, ReferenceFrame.getWorldFrame(), registry);
      waypointFrame = new PoseReferenceFrame("waypointFrame" + waypointIndex, ReferenceFrame.getWorldFrame());

      yoSmoothnessGradient = new YoFrameVector3D("smoothGradient" + waypointIndex, ReferenceFrame.getWorldFrame(), registry);
      yoEqualSpacingGradient = new YoFrameVector3D("spacingGradient" + waypointIndex, ReferenceFrame.getWorldFrame(), registry);
      yoCollisionGradient = new YoFrameVector3D("collisionGradient" + waypointIndex, ReferenceFrame.getWorldFrame(), registry);
      yoDisplacementGradient = new YoFrameVector3D("displacementGradient" + waypointIndex, ReferenceFrame.getWorldFrame(), registry);

      int minMaxXOffsetGroundPlane = 1;
      int minYOffsetGroundPlane = 0;
      int maxYOffsetGroundPlane = 4;
      for (int xi = -minMaxXOffsetGroundPlane; xi <= minMaxXOffsetGroundPlane; xi++)
      {
         for (int yi = minYOffsetGroundPlane; yi <= maxYOffsetGroundPlane; yi++)
         {
            groundPlaneXOffsets.add(xi);
            groundPlaneYOffsets.add(yi);
         }
      }

      if (visualize)
      {
         FrameBox3D collisionBox = new FrameBox3D();
         collisionBox.getSize().set(plannerParameters.getCollisionBoxSizeX(), plannerParameters.getCollisionBoxSizeY(), 0.6);

         waypointGraphic = new YoGraphicPosition("waypointViz" + waypointIndex, waypoint.getPosition(), 0.02, YoAppearance.Red());
         turnPointGraphic = new YoGraphicPosition("turnPointViz" + waypointIndex, waypoint.getPosition(), 0.02, YoAppearance.White());
         Graphics3DObject collisionBoxGraphic = new Graphics3DObject();
         collisionBoxColor.setTransparency(0.6);
         collisionBoxGraphic.addCube(collisionBox.getSizeX(), collisionBox.getSizeY(), collisionBox.getSizeZ(), true, collisionBoxColor);
         YoGraphicShape yoCollisionBoxGraphic = new YoGraphicShape("collisionGraphic" + waypointIndex, collisionBoxGraphic, waypoint, 1.0);

         YoGraphicCoordinateSystem waypointOrientedGraphic = new YoGraphicCoordinateSystem("waypointCoordViz" + waypointIndex, waypoint, 0.2);

         graphicsListRegistry.registerYoGraphic("Waypoints", waypointGraphic);
         graphicsListRegistry.registerYoGraphic("Waypoints", turnPointGraphic);
         graphicsListRegistry.registerYoGraphic("Collisions", yoCollisionBoxGraphic);
         graphicsListRegistry.registerYoGraphic("Normals", waypointOrientedGraphic);

         YoGraphicVector smoothnessGradientViz = new YoGraphicVector("smoothnessGradientViz" + waypointIndex,
                                                                     waypoint.getPosition(),
                                                                     yoSmoothnessGradient,
                                                                     gradientGraphicScale,
                                                                     smoothnessColor);
         YoGraphicVector equalSpacingGradientViz = new YoGraphicVector("spacingGradientViz" + waypointIndex,
                                                                       waypoint.getPosition(),
                                                                       yoEqualSpacingGradient,
                                                                       gradientGraphicScale,
                                                                       spacingColor);
         YoGraphicVector collisionGradientViz = new YoGraphicVector("collisionGradientViz" + waypointIndex,
                                                                    waypoint.getPosition(),
                                                                    yoCollisionGradient,
                                                                    gradientGraphicScale,
                                                                    collisionColor);
         YoGraphicVector displacementGradientViz = new YoGraphicVector("displacementGradientViz" + waypointIndex,
                                                                       waypoint.getPosition(),
                                                                       yoDisplacementGradient,
                                                                       gradientGraphicScale,
                                                                       displacementColor);

         graphicsListRegistry.registerYoGraphic("Smoothness Gradient", smoothnessGradientViz);
         graphicsListRegistry.registerYoGraphic("Spacing Gradient", equalSpacingGradientViz);
         graphicsListRegistry.registerYoGraphic("Collision Gradient", collisionGradientViz);
         graphicsListRegistry.registerYoGraphic("Displacement Gradient", displacementGradientViz);

         parentRegistry.addChild(registry);
      }
      else
      {
         waypointGraphic = null;
         turnPointGraphic = null;
      }
   }

   public void initialize(List<Point3D> bodyPath, TerrainMapData terrainMapData)
   {
      this.terrainMapData = terrainMapData;
      this.pathSize = bodyPath.size();

      isTurnPoint.set(false);
      AStarBodyPathPlanner.packRadialOffsets(terrainMapData, plannerParameters.getSnapRadius(), xSnapOffsets, ySnapOffsets);

      if (visualize)
      {
         waypointGraphic.showGraphicObject();
         turnPointGraphic.hideGraphicObject();
      }

      if (waypointIndex < bodyPath.size())
      {
         initialWaypoint.set(bodyPath.get(waypointIndex));
         waypoint.getPosition().set(bodyPath.get(waypointIndex));

         if (waypointIndex == 0 || waypointIndex == bodyPath.size() - 1)
         {
            // hide collision boxes
            waypoint.setOrientationYawPitchRoll(Double.NaN, Double.NaN, Double.NaN);
         }
      }
      else
      {
         initialWaypoint.setToNaN();
         waypoint.setToNaN();
      }

      yoSmoothnessGradient.setToNaN();
      yoEqualSpacingGradient.setToNaN();
      yoCollisionGradient.setToNaN();
      yoDisplacementGradient.setToNaN();
   }

   public void setNeighbors(AStarBodyPathSmootherWaypoint[] waypoints)
   {
      this.waypoints = waypoints;
   }

   public double getHeading()
   {
      return waypoint.getYaw();
   }

   public Point3DBasics getPosition()
   {
      return waypoint.getPosition();
   }

   public Pose3DReadOnly getPose()
   {
      return waypoint;
   }

   public void setTurnPoint()
   {
      this.isTurnPoint.set(true);

      if (visualize)
      {
         turnPointGraphic.showGraphicObject();
         waypointGraphic.hideGraphicObject();
      }
   }

   public boolean isTurnPoint()
   {
      return isTurnPoint.getValue();
   }

   public Vector2D computeCollisionGradient()
   {
      int maxOffset = (int) Math.round(
            0.5 * EuclidCoreTools.norm(plannerParameters.getCollisionBoxSizeX(), plannerParameters.getCollisionBoxSizeY()) / terrainMapData.getCellSize());

      double waypointX = waypoint.getX();
      double waypointY = waypoint.getY();
      double waypointZ = waypoint.getZ();
      double heading = waypoint.getYaw();

      double sH = Math.sin(heading);
      double cH = Math.cos(heading);

      int indexX = HeightMapTools.coordinateToIndex(waypointX,
                                                    terrainMapData.getGridCenterX(),
                                                    terrainMapData.getCellSize(),
                                                    terrainMapData.getCenterIndex());
      int indexY = HeightMapTools.coordinateToIndex(waypointY,
                                                    terrainMapData.getGridCenterY(),
                                                    terrainMapData.getCellSize(),
                                                    terrainMapData.getCenterIndex());

      Vector2D gradient = new Vector2D();
      int numCollisions = 0;
      double heightThreshold = waypointZ + boxGroundOffset;
      maxCollision.set(0.0);

      for (int xi = -maxOffset; xi <= maxOffset; xi++)
      {
         for (int yi = -maxOffset; yi <= maxOffset; yi++)
         {
            int indexXI = indexX + xi;
            int indexYI = indexY + yi;

            if (indexXI < 0 || indexXI >= terrainMapData.getCellsPerAxis() || indexYI < 0 || indexYI >= terrainMapData.getCellsPerAxis())
            {
               continue;
            }

            double px = HeightMapTools.indexToCoordinate(indexXI,
                                                         terrainMapData.getGridCenterX(),
                                                         terrainMapData.getGridCenterY(),
                                                         terrainMapData.getCenterIndex());
            double py = HeightMapTools.indexToCoordinate(indexYI,
                                                         terrainMapData.getGridCenterY(),
                                                         terrainMapData.getCellSize(),
                                                         terrainMapData.getCenterIndex());

            double dx = px - waypointX;
            double dy = py - waypointY;

            double dxLocal = cH * dx + sH * dy;
            double dyLocal = -sH * dx + cH * dy;

            if (Math.abs(dxLocal) > 0.5 * plannerParameters.getCollisionBoxSizeX() || Math.abs(dyLocal) > 0.5 * plannerParameters.getCollisionBoxSizeY())
            {
               continue;
            }

            double height = terrainMapData.getHeight(indexXI, indexYI);
            if (height < heightThreshold)
            {
               continue;
            }

            double lateralPenetration = 0.5 * plannerParameters.getCollisionBoxSizeY() - Math.abs(dyLocal);
            maxCollision.set(Math.max(maxCollision.getValue(), lateralPenetration));

            gradient.addX(Math.signum(dyLocal) * -lateralPenetration * sH);
            gradient.addY(Math.signum(dyLocal) * lateralPenetration * cH);
            numCollisions++;
         }
      }

      if (numCollisions > 0)
      {
         gradient.scale(plannerParameters.getSmootherCollisionWeight() / numCollisions);
      }

      if (visualize)
      {
         yoCollisionGradient.set(-gradient.getX(), -gradient.getY(), 0.0);
      }

      return gradient;
   }

   public void update(boolean firstTick)
   {
      // Cell key
      int currentKey = HeightMapTools.coordinateToKey(waypoint.getX(),
                                                      waypoint.getY(),
                                                      terrainMapData.getGridCenterX(),
                                                      terrainMapData.getGridCenterY(),
                                                      terrainMapData.getCellSize(),
                                                      terrainMapData.getCenterIndex());
      previousCellKey = cellKey;
      cellKey = currentKey;

      if (firstTick)
      {
         previousCellKey = currentKey;
      }

      // Update orientation
      AStarBodyPathSmootherWaypoint previous = waypoints[waypointIndex - 1];
      AStarBodyPathSmootherWaypoint next = waypoints[waypointIndex + 1];

      double x0 = previous.getPosition().getX();
      double y0 = previous.getPosition().getY();
      double x1 = getPosition().getX();
      double y1 = getPosition().getY();
      double x2 = next.getPosition().getX();
      double y2 = next.getPosition().getY();

      double heading0 = Math.atan2(y1 - y0, x1 - x0);
      double heading1 = Math.atan2(y2 - y1, x2 - x1);
      this.waypoint.setOrientationYawPitchRoll(AngleTools.computeAngleAverage(heading0, heading1), 0.0, 0.0);

      // Compute new height if shifted
      if (firstTick || cellKey != previousCellKey)
      {
         computeHeight();
      }

      // Update frames
      waypointFrame.setPoseAndUpdate(waypoint);
   }

   private void computeHeight()
   {
      int centerIndex = terrainMapData.getCenterIndex();
      int xIndex = HeightMapTools.coordinateToIndex(waypoint.getX(), terrainMapData.getGridCenterX(), terrainMapData.getCellSize(), centerIndex);
      int yIndex = HeightMapTools.coordinateToIndex(waypoint.getY(), terrainMapData.getGridCenterY(), terrainMapData.getCellSize(), centerIndex);

      double previousHeight = waypoint.getZ();
      double heightSampleDelta = plannerParameters.getMinSnapHeightThreshold();
      double minHeight = previousHeight - heightSampleDelta;
      double maxHeight = previousHeight + heightSampleDelta;

      double runningSum = 0.0;
      int numberOfSamples = 0;

      for (int i = 0; i < xSnapOffsets.size(); i++)
      {
         int xQuery = xIndex + xSnapOffsets.get(i);
         int yQuery = yIndex + ySnapOffsets.get(i);
         double heightQuery = terrainMapData.getHeight(xQuery, yQuery);

         if (!Double.isNaN(heightQuery) && MathTools.intervalContains(heightQuery, minHeight, maxHeight))
         {
            runningSum += heightQuery;
            numberOfSamples++;
         }
      }

      if (numberOfSamples > 0)
      {
         waypoint.setZ(runningSum / numberOfSamples);
      }
      else
      {
         waypoint.setZ(0.5 * (waypoints[waypointIndex - 1].getPosition().getZ() + waypoints[waypointIndex + 1].getPosition().getZ()));
      }
   }

   public void updateGradientGraphics(double spacingGradientX, double spacingGradientY, double smoothnessGradientX, double smoothnessGradientY)
   {
      yoEqualSpacingGradient.set(-spacingGradientX, -spacingGradientY, 0.0);
      yoSmoothnessGradient.set(-smoothnessGradientX, -smoothnessGradientY, 0.0);
   }

   public double getMaxCollision()
   {
      return maxCollision.getValue();
   }
}
