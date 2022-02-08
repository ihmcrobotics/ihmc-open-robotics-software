package us.ihmc.footstepPlanning.bodyPath;

import gnu.trove.list.array.TIntArrayList;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.referenceFrame.FrameBox3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DBasics;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.UnitVector3DBasics;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.*;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.sensorProcessing.heightMap.HeightMapTools;
import us.ihmc.yoVariables.euclid.YoVector2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoseUsingYawPitchRoll;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.List;

import static us.ihmc.footstepPlanning.bodyPath.AStarBodyPathPlanner.boxSizeX;
import static us.ihmc.footstepPlanning.bodyPath.AStarBodyPathPlanner.boxSizeY;
import static us.ihmc.footstepPlanning.bodyPath.AStarBodyPathSmoother.*;

public class AStarBodyPathSmootherWaypoint
{
   private static final double boxGroundOffset = 0.35;
   private static final FrameBox3D collisionBox = new FrameBox3D();

   static
   {
      collisionBox.getSize().set(boxSizeX, boxSizeY, 0.6);
   }

   private static final AppearanceDefinition collisionBoxColor = YoAppearance.RGBColorFromHex(0x824e38);
   private final boolean visualize;

   private HeightMapData heightMapData;
   private final int waypointIndex;
   private final YoFramePoint3D initialWaypoint;
   private final YoFramePoseUsingYawPitchRoll waypoint;
   private final PoseReferenceFrame waypointFrame;
   private final HeightMapLeastSquaresNormalCalculator surfaceNormalCalculator;
   private final YoBoolean isTurnPoint;
   private int cellKey;

   private final TIntArrayList xSnapOffsets = new TIntArrayList();
   private final TIntArrayList ySnapOffsets = new TIntArrayList();

   private final YoDouble maxCollision;
   private final YoDouble alphaRoll;

   private static final double gradientGraphicScale = 0.2;
   private static final AppearanceDefinition smoothnessColor = YoAppearance.Blue();
   private static final AppearanceDefinition spacingColor = YoAppearance.Green();
   private static final AppearanceDefinition collisionColor = YoAppearance.Crimson();
   private static final AppearanceDefinition rollColor = YoAppearance.Red();
   private static final AppearanceDefinition displacementColor = YoAppearance.White();
   private final YoGraphicPosition waypointGraphic, turnPointGraphic;

   private final YoFrameVector3D yoSmoothnessGradient;
   private final YoFrameVector3D yoEqualSpacingGradient;
   private final YoFrameVector3D yoCollisionGradient;
   private final YoFrameVector3D yoRollGradient;
   private final YoFrameVector3D yoDisplacementGradient;
   private final YoFrameVector3D yoSurfaceNormal;

   private final FrameVector3D tempVector = new FrameVector3D();

   private AStarBodyPathSmootherWaypoint previous, next;
   private final YoVector2D rollDelta;

   public AStarBodyPathSmootherWaypoint(int waypointIndex,
                                        HeightMapLeastSquaresNormalCalculator surfaceNormalCalculator,
                                        YoGraphicsListRegistry graphicsListRegistry,
                                        YoRegistry parentRegistry)
   {
      this.surfaceNormalCalculator = surfaceNormalCalculator;
      this.waypointIndex = waypointIndex;

      YoRegistry registry = new YoRegistry("Waypoint" + waypointIndex);
      maxCollision = new YoDouble("maxCollision" + waypointIndex, registry);
      alphaRoll = new YoDouble("alphaRoll" + waypointIndex, registry);
      rollDelta = new YoVector2D("deltaRoll" + waypointIndex, registry);
      isTurnPoint = new YoBoolean("isTurnPoint" + waypointIndex, registry);

      visualize = parentRegistry != null;
      waypoint = new YoFramePoseUsingYawPitchRoll("waypoint" + waypointIndex, ReferenceFrame.getWorldFrame(), registry);
      initialWaypoint = new YoFramePoint3D("initWaypoint" + waypointIndex, ReferenceFrame.getWorldFrame(), registry);
      waypointFrame = new PoseReferenceFrame("waypointFrame" + waypointIndex, ReferenceFrame.getWorldFrame());

      yoSmoothnessGradient = new YoFrameVector3D("smoothGradient" + waypointIndex, ReferenceFrame.getWorldFrame(), registry);
      yoEqualSpacingGradient = new YoFrameVector3D("spacingGradient" + waypointIndex, ReferenceFrame.getWorldFrame(), registry);
      yoCollisionGradient = new YoFrameVector3D("collisionGradient" + waypointIndex, ReferenceFrame.getWorldFrame(), registry);
      yoRollGradient = new YoFrameVector3D("rollGradient" + waypointIndex, ReferenceFrame.getWorldFrame(), registry);
      yoDisplacementGradient = new YoFrameVector3D("displacementGradient" + waypointIndex, ReferenceFrame.getWorldFrame(), registry);
      yoSurfaceNormal = new YoFrameVector3D("surfaceNormal" + waypointIndex, ReferenceFrame.getWorldFrame(), registry);

      yoSmoothnessGradient.setToNaN();
      yoEqualSpacingGradient.setToNaN();
      yoCollisionGradient.setToNaN();
      yoRollGradient.setToNaN();
      yoDisplacementGradient.setToNaN();

      if (visualize)
      {
         waypointGraphic = new YoGraphicPosition("waypointViz" + waypointIndex, waypoint.getPosition(), 0.02, YoAppearance.Red());
         turnPointGraphic = new YoGraphicPosition("turnPointViz" + waypointIndex, waypoint.getPosition(), 0.02, YoAppearance.White());
         Graphics3DObject collisionBoxGraphic = new Graphics3DObject();
         collisionBoxColor.setTransparency(0.6);
         collisionBoxGraphic.addCube(collisionBox.getSizeX(), collisionBox.getSizeY(), collisionBox.getSizeZ(), true, collisionBoxColor);
         YoGraphicShape yoCollisionBoxGraphic = new YoGraphicShape("collisionGraphic" + waypointIndex, collisionBoxGraphic, waypoint, 1.0);

         YoGraphicCoordinateSystem waypointOrientedGraphic = new YoGraphicCoordinateSystem("waypointCoordViz" + waypointIndex, waypoint, 0.2);
         YoGraphicVector surfaceNormal = new YoGraphicVector("surfaceNormal" + waypointIndex, waypoint.getPosition(), yoSurfaceNormal, 0.3);

         graphicsListRegistry.registerYoGraphic("Waypoints", waypointGraphic);
         graphicsListRegistry.registerYoGraphic("Waypoints", turnPointGraphic);
         graphicsListRegistry.registerYoGraphic("Waypoints", waypointOrientedGraphic);
         graphicsListRegistry.registerYoGraphic("Waypoints", surfaceNormal);
         graphicsListRegistry.registerYoGraphic("Collisions", yoCollisionBoxGraphic);

         YoGraphicVector smoothnessGradientViz = new YoGraphicVector("smoothnessGradientViz" + waypointIndex, waypoint.getPosition(), yoSmoothnessGradient, gradientGraphicScale, smoothnessColor);
         YoGraphicVector equalSpacingGradientViz = new YoGraphicVector("spacingGradientViz" + waypointIndex, waypoint.getPosition(), yoEqualSpacingGradient, gradientGraphicScale, spacingColor);
         YoGraphicVector collisionGradientViz = new YoGraphicVector("collisionGradientViz" + waypointIndex, waypoint.getPosition(), yoCollisionGradient, gradientGraphicScale, collisionColor);
         YoGraphicVector rollGradientViz = new YoGraphicVector("rollGradientViz" + waypointIndex, waypoint.getPosition(), yoRollGradient, gradientGraphicScale, rollColor);
         YoGraphicVector displacementGradientViz = new YoGraphicVector("displacementGradientViz" + waypointIndex, waypoint.getPosition(), yoDisplacementGradient, gradientGraphicScale, displacementColor);

         graphicsListRegistry.registerYoGraphic("Smoothness Gradient", smoothnessGradientViz);
         graphicsListRegistry.registerYoGraphic("Spacing Gradient", equalSpacingGradientViz);
         graphicsListRegistry.registerYoGraphic("Collision Gradient", collisionGradientViz);
         graphicsListRegistry.registerYoGraphic("Roll Gradient", rollGradientViz);
         graphicsListRegistry.registerYoGraphic("Displacement Gradient", displacementGradientViz);

         parentRegistry.addChild(registry);
      }
      else
      {
         waypointGraphic = null;
         turnPointGraphic = null;
      }
   }

   public void initialize(List<Point3D> bodyPath, HeightMapData heightMapData)
   {
      this.heightMapData = heightMapData;
      isTurnPoint.set(false);
      AStarBodyPathPlanner.packRadialOffsets(heightMapData, AStarBodyPathPlanner.snapRadius, xSnapOffsets, ySnapOffsets);

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
   }

   public void setNeighbors(AStarBodyPathSmootherWaypoint previous, AStarBodyPathSmootherWaypoint next)
   {
      this.previous = previous;
      this.next = next;
   }

   public double getHeading()
   {
      return waypoint.getYaw();
   }

   public Point3DBasics getPosition()
   {
      return waypoint.getPosition();
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
      int maxOffset = (int) Math.round(0.5 * EuclidCoreTools.norm(boxSizeX, boxSizeY) / heightMapData.getGridResolutionXY());

      double waypointX = waypoint.getX();
      double waypointY = waypoint.getY();
      double waypointZ = waypoint.getZ();
      double heading = waypoint.getYaw();

      double sH = Math.sin(heading);
      double cH = Math.cos(heading);

      int indexX = HeightMapTools.coordinateToIndex(waypointX, heightMapData.getGridCenter().getX(), heightMapData.getGridResolutionXY(), heightMapData.getCenterIndex());
      int indexY = HeightMapTools.coordinateToIndex(waypointY, heightMapData.getGridCenter().getY(), heightMapData.getGridResolutionXY(), heightMapData.getCenterIndex());

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

            if (indexXI < 0 || indexXI >= heightMapData.getCellsPerAxis() || indexYI < 0 || indexYI >= heightMapData.getCellsPerAxis())
            {
               continue;
            }

            double px = HeightMapTools.indexToCoordinate(indexXI, heightMapData.getGridCenter().getX(), heightMapData.getGridResolutionXY(), heightMapData.getCenterIndex());
            double py = HeightMapTools.indexToCoordinate(indexYI, heightMapData.getGridCenter().getY(), heightMapData.getGridResolutionXY(), heightMapData.getCenterIndex());

            double dx = px - waypointX;
            double dy = py - waypointY;

            double dxLocal = cH * dx + sH * dy;
            double dyLocal = -sH * dx + cH * dy;

            if (Math.abs(dxLocal) > 0.5 * boxSizeX || Math.abs(dyLocal) > 0.5 * boxSizeY)
            {
               continue;
            }

            double height = heightMapData.getHeightAt(indexXI, indexYI);
            if (height < heightThreshold)
            {
               continue;
            }

            double lateralPenetration = 0.5 * boxSizeY - Math.abs(dyLocal);
            maxCollision.set(Math.max(maxCollision.getValue(), lateralPenetration));

            gradient.addX(Math.signum(dyLocal) * -lateralPenetration * sH);
            gradient.addY(Math.signum(dyLocal) * lateralPenetration * cH);
            numCollisions++;
         }
      }

      if (numCollisions > 0)
      {
         gradient.scale(collisionWeight / numCollisions);
      }

      if (visualize)
      {
         yoCollisionGradient.set(-gradient.getX(), -gradient.getY(), 0.0);
      }

      return gradient;
   }

   public Vector2DBasics computeRollInclineGradient(HeightMapData heightMapData)
   {
      UnitVector3DBasics surfaceNormal = surfaceNormalCalculator.getSurfaceNormal(HeightMapTools.coordinateToKey(waypoint.getX(),
                                                                                                                 waypoint.getY(),
                                                                                                                 heightMapData.getGridCenter().getX(),
                                                                                                                 heightMapData.getGridCenter().getY(),
                                                                                                                 heightMapData.getGridResolutionXY(),
                                                                                                                 heightMapData.getCenterIndex()));

      if (surfaceNormal != null)
      {
         yoSurfaceNormal.set(surfaceNormal);
         tempVector.setIncludingFrame(waypointFrame, Axis3D.Y);
         tempVector.changeFrame(ReferenceFrame.getWorldFrame());
         double rollDotY = tempVector.dot(surfaceNormal);
         alphaRoll.set(Math.signum(rollDotY) * MathTools.square(rollDotY));
         tempVector.scale(alphaRoll.getValue());

         double alphaIncline = Math.atan2(next.getPosition().getZ() - previous.getPosition().getZ(), next.getPosition().distanceXY(previous.getPosition()));
         double inclineScale = EuclidCoreTools.clamp((Math.abs(alphaIncline) - Math.toRadians(4.0)) / Math.toRadians(20.0), 0.0, 1.0);
         rollDelta.set(rollWeight * inclineScale * tempVector.getX(), rollWeight * inclineScale * tempVector.getY());
      }
      else
      {
         yoSurfaceNormal.setToZero();
         rollDelta.setToZero();
      }

      return rollDelta;
   }

   public Tuple3DReadOnly computeDisplacementGradient()
   {
      tempVector.setToZero(ReferenceFrame.getWorldFrame());
      tempVector.sub(initialWaypoint, waypoint.getPosition());
      tempVector.changeFrame(waypointFrame);
      tempVector.setX(0.0);
      tempVector.changeFrame(ReferenceFrame.getWorldFrame());
      tempVector.scale(displacementWeight);
      yoDisplacementGradient.set(tempVector);

      return tempVector;
   }

   public void update()
   {
      // Update height
      double x0 = previous.getPosition().getX();
      double y0 = previous.getPosition().getY();
      double x1 = getPosition().getX();
      double y1 = getPosition().getY();
      double x2 = next.getPosition().getX();
      double y2 = next.getPosition().getY();

      double heading0 = Math.atan2(y1 - y0, x1 - x0);
      double heading1 = Math.atan2(y2 - y1, x2 - x1);
      this.waypoint.setYaw(AngleTools.computeAngleAverage(heading0, heading1));

      // Compute new height if shifted
      int currentKey = HeightMapTools.coordinateToKey(waypoint.getX(), waypoint.getY(), heightMapData.getGridCenter().getX(), heightMapData.getGridCenter().getY(), heightMapData.getGridResolutionXY(), heightMapData.getCenterIndex());
      if (currentKey != cellKey)
      {
         snap();
         cellKey = currentKey;
      }

      // Update frame
      waypointFrame.setPoseAndUpdate(waypoint);
   }

   private void snap()
   {
      int centerIndex = heightMapData.getCenterIndex();
      int xIndex = HeightMapTools.coordinateToIndex(waypoint.getX(), heightMapData.getGridCenter().getX(), heightMapData.getGridResolutionXY(), centerIndex);
      int yIndex = HeightMapTools.coordinateToIndex(waypoint.getY(), heightMapData.getGridCenter().getY(), heightMapData.getGridResolutionXY(), centerIndex);

      double maxHeight = heightMapData.getEstimatedGroundHeight();
      for (int i = 0; i < xSnapOffsets.size(); i++)
      {
         int xQuery = xIndex + xSnapOffsets.get(i);
         int yQuery = yIndex + ySnapOffsets.get(i);
         double heightQuery = heightMapData.getHeightAt(xQuery, yQuery);
         if (heightQuery > maxHeight)
         {
            maxHeight = heightQuery;
         }
      }

      waypoint.setZ(maxHeight);
   }

   public void updateGradientGraphics(double spacingGradientX, double spacingGradientY, double smoothnessGradientX, double smoothnessGradientY)
   {
      yoEqualSpacingGradient.set(-spacingGradientX, -spacingGradientY, 0.0);
      yoSmoothnessGradient.set(-smoothnessGradientX, -smoothnessGradientY, 0.0);
      yoRollGradient.setToZero();
   }

   public void updateRollGraphics(double gradientX, double gradientY)
   {
      yoRollGradient.add(-gradientX, -gradientY, 0.0);
   }

   public double getMaxCollision()
   {
      return maxCollision.getValue();
   }
}
