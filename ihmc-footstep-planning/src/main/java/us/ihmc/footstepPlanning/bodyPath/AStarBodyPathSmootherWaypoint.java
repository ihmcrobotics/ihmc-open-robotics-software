package us.ihmc.footstepPlanning.bodyPath;

import gnu.trove.list.array.TDoubleArrayList;
import gnu.trove.list.array.TIntArrayList;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
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
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
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

import static us.ihmc.footstepPlanning.bodyPath.AStarBodyPathPlanner.*;
import static us.ihmc.footstepPlanning.bodyPath.AStarBodyPathSmoother.*;
import static us.ihmc.footstepPlanning.bodyPath.BodyPathRANSACTraversibilityCalculator.*;

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
   private final SideDependentList<ReferenceFrame> nominalStepFrames;
   private final HeightMapLeastSquaresNormalCalculator leastSquaresSurfaceNormalCalculator;
   private final HeightMapRANSACNormalCalculator ransacNormalCalculator;
   private final YoBoolean isTurnPoint;
   private int previousCellKey, cellKey;

   private final TIntArrayList xSnapOffsets = new TIntArrayList();
   private final TIntArrayList ySnapOffsets = new TIntArrayList();

   private final TDoubleArrayList xTraversibilityGradientOffsets = new TDoubleArrayList();
   private final TDoubleArrayList yTraversibilityGradientOffsets = new TDoubleArrayList();
   private final TDoubleArrayList xTraversibilityNominalOffsets = new TDoubleArrayList();
   private final TDoubleArrayList yTraversibilityNominalOffsets = new TDoubleArrayList();

   private final SideDependentList<YoDouble> traversibilitySamplePos;
   private final SideDependentList<YoDouble> traversibilitySampleNeg;
   private final SideDependentList<YoDouble> traversibilitySampleNominal;

   private final YoDouble maxCollision;
   private final YoDouble alphaRoll;
   private final YoDouble sampledLSNormalHeight;
   private final YoDouble elevationIncline;

   private static final double gradientGraphicScale = 0.17;
   private static final AppearanceDefinition smoothnessColor = YoAppearance.Blue();
   private static final AppearanceDefinition spacingColor = YoAppearance.Green();
   private static final AppearanceDefinition collisionColor = YoAppearance.Crimson();
   private static final AppearanceDefinition rollColor = YoAppearance.Red();
   private static final AppearanceDefinition traversibilityColor = YoAppearance.Violet();
   private static final AppearanceDefinition displacementColor = YoAppearance.White();
   private final YoGraphicPosition waypointGraphic, turnPointGraphic;

   private final YoFrameVector3D yoSmoothnessGradient;
   private final YoFrameVector3D yoEqualSpacingGradient;
   private final YoFrameVector3D yoCollisionGradient;
   private final YoFrameVector3D yoRollGradient;
   private final YoFrameVector3D yoDisplacementGradient;
   private final YoFrameVector3D yoTraversibilityGradient;
   private final YoFrameVector3D yoSurfaceNormal;

   private final SideDependentList<YoFramePoseUsingYawPitchRoll> yoNominalStepPoses;
   private final SideDependentList<YoFrameVector3D> yoNominalTraversibility;
   private final FrameVector3D tempVector = new FrameVector3D();
   private final FramePose3D tempPose = new FramePose3D();
   private final FramePoint3D tempPoint = new FramePoint3D();

   private final SideDependentList<YoFramePoint3D> yoElevatedStepPositions;
   private final SideDependentList<YoFrameVector3D> yoSidedTraversibility;

   private int pathSize;
   private AStarBodyPathSmootherWaypoint[] waypoints;
   private final YoVector2D rollDelta;

   public AStarBodyPathSmootherWaypoint(int waypointIndex,
                                        HeightMapLeastSquaresNormalCalculator leastSquaresSurfaceNormalCalculator,
                                        HeightMapRANSACNormalCalculator ransacNormalCalculator,
                                        YoGraphicsListRegistry graphicsListRegistry,
                                        YoRegistry parentRegistry)
   {
      this.leastSquaresSurfaceNormalCalculator = leastSquaresSurfaceNormalCalculator;
      this.ransacNormalCalculator = ransacNormalCalculator;
      this.waypointIndex = waypointIndex;

      YoRegistry registry = new YoRegistry("Waypoint" + waypointIndex);
      maxCollision = new YoDouble("maxCollision" + waypointIndex, registry);
      alphaRoll = new YoDouble("alphaRoll" + waypointIndex, registry);
      rollDelta = new YoVector2D("deltaRoll" + waypointIndex, registry);
      isTurnPoint = new YoBoolean("isTurnPoint" + waypointIndex, registry);
      sampledLSNormalHeight = new YoDouble("sampledLSNormalHeight" + waypointIndex, registry);
      elevationIncline = new YoDouble("elevationIncline" + waypointIndex, registry);

      visualize = parentRegistry != null;
      waypoint = new YoFramePoseUsingYawPitchRoll("waypoint" + waypointIndex, ReferenceFrame.getWorldFrame(), registry);
      initialWaypoint = new YoFramePoint3D("initWaypoint" + waypointIndex, ReferenceFrame.getWorldFrame(), registry);
      waypointFrame = new PoseReferenceFrame("waypointFrame" + waypointIndex, ReferenceFrame.getWorldFrame());
      nominalStepFrames = new SideDependentList<>(side -> ReferenceFrameTools.constructFrameWithUnchangingTranslationFromParent(side.getCamelCaseNameForStartOfExpression() + "nominalStepFrame" + waypointIndex, waypointFrame, new Vector3D(0.0, side.negateIfRightSide(halfStanceWidthTraversibility), 0.0)));

      yoSmoothnessGradient = new YoFrameVector3D("smoothGradient" + waypointIndex, ReferenceFrame.getWorldFrame(), registry);
      yoEqualSpacingGradient = new YoFrameVector3D("spacingGradient" + waypointIndex, ReferenceFrame.getWorldFrame(), registry);
      yoCollisionGradient = new YoFrameVector3D("collisionGradient" + waypointIndex, ReferenceFrame.getWorldFrame(), registry);
      yoRollGradient = new YoFrameVector3D("rollGradient" + waypointIndex, ReferenceFrame.getWorldFrame(), registry);
      yoDisplacementGradient = new YoFrameVector3D("displacementGradient" + waypointIndex, ReferenceFrame.getWorldFrame(), registry);
      yoSurfaceNormal = new YoFrameVector3D("surfaceNormal" + waypointIndex, ReferenceFrame.getWorldFrame(), registry);
      yoNominalStepPoses = new SideDependentList<>(side -> new YoFramePoseUsingYawPitchRoll(side.getCamelCaseNameForStartOfExpression() + "nominalStepPose" + waypointIndex, ReferenceFrame.getWorldFrame(), registry));
      yoNominalTraversibility = new SideDependentList<>(side -> new YoFrameVector3D(side.getCamelCaseNameForStartOfExpression() + "TravNominal" + waypointIndex, ReferenceFrame.getWorldFrame(), registry));
      yoTraversibilityGradient = new YoFrameVector3D("traversibilityGradient" + waypointIndex, ReferenceFrame.getWorldFrame(), registry);

      traversibilitySampleNeg = new SideDependentList<>(side -> new YoDouble(side.getCamelCaseNameForStartOfExpression() + "TravSampleNeg" + waypointIndex, registry));
      traversibilitySamplePos = new SideDependentList<>(side -> new YoDouble(side.getCamelCaseNameForStartOfExpression() + "TravSamplePos" + waypointIndex, registry));
      traversibilitySampleNominal = new SideDependentList<>(side -> new YoDouble(side.getCamelCaseNameForStartOfExpression() + "TravSampleNominal" + waypointIndex, registry));

      yoElevatedStepPositions = new SideDependentList<>(side -> new YoFramePoint3D(side.getCamelCaseNameForStartOfExpression() + "DebugStepPose" + waypointIndex, ReferenceFrame.getWorldFrame(), registry));
      yoSidedTraversibility = new SideDependentList<>(side -> new YoFrameVector3D(side.getCamelCaseNameForStartOfExpression() + "DebugTrav" + waypointIndex, ReferenceFrame.getWorldFrame(), registry));

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

         for (RobotSide side : RobotSide.values)
         {
            YoGraphicCoordinateSystem nominalStepPose = new YoGraphicCoordinateSystem(side.getCamelCaseNameForStartOfExpression() + "stepPoseViz" + waypointIndex, yoNominalStepPoses.get(side), 0.2);
            YoGraphicVector nominalTraversibilityViz = new YoGraphicVector(side.getCamelCaseNameForStartOfExpression() + "nominalTraversibilityViz" + waypointIndex, yoNominalStepPoses.get(side).getPosition(), yoNominalTraversibility.get(side), 0.5, YoAppearance.Yellow());
            YoGraphicVector debugTravViz = new YoGraphicVector(side.getCamelCaseNameForStartOfExpression() + "debugTravViz" + waypointIndex, yoElevatedStepPositions.get(side), yoSidedTraversibility.get(side), 0.5);

            graphicsListRegistry.registerYoGraphic("Step Poses", nominalStepPose);
            graphicsListRegistry.registerYoGraphic("Step Poses", nominalTraversibilityViz);
            graphicsListRegistry.registerYoGraphic("Debug Trav", debugTravViz);
         }

         graphicsListRegistry.registerYoGraphic("Waypoints", waypointGraphic);
         graphicsListRegistry.registerYoGraphic("Waypoints", turnPointGraphic);
         graphicsListRegistry.registerYoGraphic("Collisions", yoCollisionBoxGraphic);
         graphicsListRegistry.registerYoGraphic("Normals", waypointOrientedGraphic);
         graphicsListRegistry.registerYoGraphic("Normals", surfaceNormal);

         YoGraphicVector smoothnessGradientViz = new YoGraphicVector("smoothnessGradientViz" + waypointIndex, waypoint.getPosition(), yoSmoothnessGradient, gradientGraphicScale, smoothnessColor);
         YoGraphicVector equalSpacingGradientViz = new YoGraphicVector("spacingGradientViz" + waypointIndex, waypoint.getPosition(), yoEqualSpacingGradient, gradientGraphicScale, spacingColor);
         YoGraphicVector collisionGradientViz = new YoGraphicVector("collisionGradientViz" + waypointIndex, waypoint.getPosition(), yoCollisionGradient, gradientGraphicScale, collisionColor);
         YoGraphicVector rollGradientViz = new YoGraphicVector("rollGradientViz" + waypointIndex, waypoint.getPosition(), yoRollGradient, gradientGraphicScale, rollColor);
         YoGraphicVector displacementGradientViz = new YoGraphicVector("displacementGradientViz" + waypointIndex, waypoint.getPosition(), yoDisplacementGradient, gradientGraphicScale, displacementColor);
         YoGraphicVector traversibilityGradientViz = new YoGraphicVector("traversibilityGradientViz" + waypointIndex, waypoint.getPosition(), yoTraversibilityGradient, gradientGraphicScale, traversibilityColor);

         graphicsListRegistry.registerYoGraphic("Smoothness Gradient", smoothnessGradientViz);
         graphicsListRegistry.registerYoGraphic("Spacing Gradient", equalSpacingGradientViz);
         graphicsListRegistry.registerYoGraphic("Collision Gradient", collisionGradientViz);
         graphicsListRegistry.registerYoGraphic("Roll Gradient", rollGradientViz);
         graphicsListRegistry.registerYoGraphic("Displacement Gradient", displacementGradientViz);
         graphicsListRegistry.registerYoGraphic("Traversibility Gradient", traversibilityGradientViz);

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
      this.pathSize = bodyPath.size();

      isTurnPoint.set(false);
      AStarBodyPathPlanner.packRadialOffsets(heightMapData, AStarBodyPathPlanner.snapRadius, xSnapOffsets, ySnapOffsets);

      xTraversibilityGradientOffsets.clear();
      yTraversibilityGradientOffsets.clear();
      xTraversibilityNominalOffsets.clear();
      yTraversibilityNominalOffsets.clear();

      for (RobotSide side : RobotSide.values)
      {
         if (waypointIndex == 0 || waypointIndex == pathSize - 1)
         {
            traversibilitySampleNominal.get(side).set(1.0);
         }
         else
         {
            traversibilitySampleNominal.get(side).set(0.0);
         }
      }

      int minYTraversibilityGradWindow = (int) Math.round((yOffsetTraversibilityGradientWindow - 0.5 * traversibilitySampleWindowY) / heightMapData.getGridResolutionXY());
      int maxYTraversibilityGradWindow = (int) Math.round((yOffsetTraversibilityGradientWindow + 0.5 * traversibilitySampleWindowY) / heightMapData.getGridResolutionXY());
      int minXTraversibilityGradWindow = (int) Math.round(-0.5 * traversibilitySampleWindowX / heightMapData.getGridResolutionXY());
      int maxXTraversibilityGradWindow = (int) Math.round(0.5 * traversibilitySampleWindowX / heightMapData.getGridResolutionXY());
      int minMaxYTraversibilityNomWindow = (int) Math.round(0.5 * yOffsetTraversibilityNominalWindow / heightMapData.getGridResolutionXY());

      for (int xi = minXTraversibilityGradWindow; xi <= maxXTraversibilityGradWindow; xi++)
      {
         for (int yi = minYTraversibilityGradWindow; yi <= maxYTraversibilityGradWindow; yi++)
         {
            double dx = xi * heightMapData.getGridResolutionXY();
            double dy = yi * heightMapData.getGridResolutionXY();
            xTraversibilityGradientOffsets.add(dx);
            yTraversibilityGradientOffsets.add(dy);
         }

         for (int yi = -minMaxYTraversibilityNomWindow; yi <= minMaxYTraversibilityNomWindow; yi++)
         {
            double dx = xi * heightMapData.getGridResolutionXY();
            double dy = yi * heightMapData.getGridResolutionXY();
            xTraversibilityNominalOffsets.add(dx);
            yTraversibilityNominalOffsets.add(dy);
         }
      }

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
      yoRollGradient.setToNaN();
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
      int key = HeightMapTools.coordinateToKey(waypoint.getX(), waypoint.getY(), heightMapData.getGridCenter().getX(), heightMapData.getGridCenter().getY(), heightMapData.getGridResolutionXY(), heightMapData.getCenterIndex());
      UnitVector3DBasics surfaceNormal = leastSquaresSurfaceNormalCalculator.getSurfaceNormal(key);

      if (surfaceNormal != null)
      {
         yoSurfaceNormal.set(surfaceNormal);
         tempVector.setIncludingFrame(waypointFrame, Axis3D.Y);
         tempVector.changeFrame(ReferenceFrame.getWorldFrame());
         double rollDotY = tempVector.dot(surfaceNormal);
         alphaRoll.set(Math.signum(rollDotY) * MathTools.square(rollDotY));
         tempVector.scale(alphaRoll.getValue());

         AStarBodyPathSmootherWaypoint previous = waypoints[Math.max(0, waypointIndex - 1)];
         AStarBodyPathSmootherWaypoint next = waypoints[Math.min(pathSize - 1, waypointIndex + 1)];

         double incline = Math.atan2(next.getPosition().getZ() - previous.getPosition().getZ(), next.getPosition().distanceXY(previous.getPosition()));
         elevationIncline.set(incline);

         double inclineClipped = EuclidCoreTools.clamp((Math.abs(incline) - Math.toRadians(10.0)) / Math.toRadians(20.0), 0.0, 1.0);
         rollDelta.set(rollWeight * inclineClipped * tempVector.getX(), rollWeight * inclineClipped * tempVector.getY());

         sampledLSNormalHeight.set(leastSquaresSurfaceNormalCalculator.getSampledHeight(key));
      }
      else
      {
         elevationIncline.set(Double.NaN);
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

   public void computeCurrentTraversibility()
   {
      for (RobotSide side : RobotSide.values)
      {
         YoDouble nominalTraversibility = traversibilitySampleNominal.get(side);
         nominalTraversibility.set(computeTraversibility(side, 1.0, waypoint.getZ(), xTraversibilityNominalOffsets, yTraversibilityNominalOffsets));
         yoNominalTraversibility.get(side).setZ(nominalTraversibility.getValue());
      }
   }

   public Tuple3DReadOnly computeTraversibilityGradient()
   {
      yoTraversibilityGradient.setToZero();

      for (RobotSide side : RobotSide.values)
      {
         YoFrameVector3D sidedTraversibility = yoSidedTraversibility.get(side);
         sidedTraversibility.setToZero();

         RobotSide oppositeSide = side.getOppositeSide();
         double nominalTraversibility = traversibilitySampleNominal.get(side).getValue();
         double oppositeTraversibility = traversibilitySampleNominal.get(oppositeSide).getValue();

         double nominalThreshold = 0.75;
         double oppositeThreshold = 0.75;
         double oppositeThresholdPreview = 0.75;

         if (nominalTraversibility > nominalThreshold)
         {
            continue;
         }
         if (oppositeTraversibility > oppositeThreshold)
         {
            continue;
         }

         double travOppositePrevious0 = getNeighbor(waypointIndex - 1).traversibilitySampleNominal.get(oppositeSide).getValue();
         double travOppositePrevious1 = getNeighbor(waypointIndex - 2).traversibilitySampleNominal.get(oppositeSide).getValue();
         double travOppositeNext0 = getNeighbor(waypointIndex + 1).traversibilitySampleNominal.get(oppositeSide).getValue();
         double travOppositeNext1 = getNeighbor(waypointIndex + 2).traversibilitySampleNominal.get(oppositeSide).getValue();
         double oppositeTraversibilityPreview = Math.min(Math.max(travOppositePrevious0, travOppositePrevious1), Math.max(travOppositeNext0, travOppositeNext1));
         if (oppositeTraversibilityPreview > oppositeThresholdPreview)
         {
            continue;
         }

         traversibilitySamplePos.get(side).set(computeTraversibility(side, 1.0, waypoint.getZ(), xTraversibilityGradientOffsets, yTraversibilityGradientOffsets));
         traversibilitySampleNeg.get(side).set(computeTraversibility(side, -1.0, waypoint.getZ(), xTraversibilityGradientOffsets, yTraversibilityGradientOffsets));

         double alphaStance = EuclidCoreTools.interpolate(1.0, 0.25, nominalTraversibility * oppositeTraversibility / (nominalThreshold * oppositeThreshold));
         double alphaStep = EuclidCoreTools.interpolate(1.0, 0.25, nominalTraversibility * oppositeTraversibilityPreview / (nominalThreshold * oppositeThresholdPreview));
         double alpha = Math.min(alphaStance, alphaStep);

         tempVector.setIncludingFrame(waypointFrame, Axis3D.Y);
         tempVector.scale(alpha * (traversibilitySamplePos.get(side).getValue() - traversibilitySampleNeg.get(side).getValue()));
         tempVector.changeFrame(ReferenceFrame.getWorldFrame());

         sidedTraversibility.set(tempVector);
         yoTraversibilityGradient.add(tempVector);
      }

      yoTraversibilityGradient.scale(traversibilityWeight);
      return yoTraversibilityGradient;
   }

   private AStarBodyPathSmootherWaypoint getNeighbor(int index)
   {
      return waypoints[MathTools.clamp(index, 0, pathSize - 1)];
   }

   private double computeTraversibility(RobotSide side, double signY, double nominalHeight, TDoubleArrayList xOffsets, TDoubleArrayList yOffsets)
   {
      int numberOfSampledCells = 0;

      double traversibilityScoreNumerator = 0.0;
      double minHeight = nominalHeight - heightWindow;
      double maxHeight = nominalHeight + heightWindow;

      for (int i = 0; i < xOffsets.size(); i++)
      {
         tempPoint.setToZero(nominalStepFrames.get(side));
         tempPoint.addX(xOffsets.get(i));
         tempPoint.addY(signY * yOffsets.get(i));

         tempPoint.changeFrame(ReferenceFrame.getWorldFrame());
         int xQuery = HeightMapTools.coordinateToIndex(tempPoint.getX(), heightMapData.getGridCenter().getX(), heightMapData.getGridResolutionXY(), heightMapData.getCenterIndex());
         int yQuery = HeightMapTools.coordinateToIndex(tempPoint.getY(), heightMapData.getGridCenter().getY(), heightMapData.getGridResolutionXY(), heightMapData.getCenterIndex());
         double heightQuery = heightMapData.getHeightAt(xQuery, yQuery);

         if (xQuery < 0 || yQuery < 0 || xQuery >= heightMapData.getCellsPerAxis() || yQuery >= heightMapData.getCellsPerAxis())
         {
            continue;
         }

         numberOfSampledCells++;
         if (heightQuery > minHeight && heightQuery < maxHeight)
         {
            if (heightMapData.isCellAtGroundPlane(xQuery, yQuery))
            {
               traversibilityScoreNumerator += 1.0;
            }
            else
            {
               double nonGroundAlpha = heightQuery - heightMapData.getEstimatedGroundHeight() < heightWindow ? nonGroundDiscount : 1.0;

               UnitVector3DBasics normal = ransacNormalCalculator.getSurfaceNormal(xQuery, yQuery);
               double incline = Math.acos(normal.getZ());
               double inclineAlpha = MathTools.clamp(EuclidCoreTools.interpolate(0.0, 1.0, (incline - minNormalToPenalize) / (maxNormalToPenalize - minNormalToPenalize)), 0.0, 1.0);
               traversibilityScoreNumerator += ((1.0 - inclineWeight) * nonGroundAlpha + inclineWeight * inclineAlpha);
            }
         }
      }

      if (numberOfSampledCells < 10)
      {
         return 0.0;
      }
      else
      {
         return traversibilityScoreNumerator / numberOfSampledCells;
      }
   }

   public void update(boolean firstTick)
   {
      // Cell key
      int currentKey = HeightMapTools.coordinateToKey(waypoint.getX(), waypoint.getY(), heightMapData.getGridCenter().getX(), heightMapData.getGridCenter().getY(), heightMapData.getGridResolutionXY(), heightMapData.getCenterIndex());
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
      this.waypoint.setYaw(AngleTools.computeAngleAverage(heading0, heading1));

      // Compute new height if shifted
      if (firstTick || cellKey != previousCellKey)
      {
         computeHeight();
      }

      // Update frames
      waypointFrame.setPoseAndUpdate(waypoint);
      for (RobotSide side : RobotSide.values)
      {
         nominalStepFrames.get(side).update();
         tempPose.setToZero(nominalStepFrames.get(side));
         tempPose.changeFrame(ReferenceFrame.getWorldFrame());
         yoNominalStepPoses.get(side).set(tempPose);

         tempPose.getPosition().addZ(0.2);
         yoElevatedStepPositions.get(side).set(tempPose.getPosition());
      }
   }

   private void computeHeight()
   {
      int centerIndex = heightMapData.getCenterIndex();
      int xIndex = HeightMapTools.coordinateToIndex(waypoint.getX(), heightMapData.getGridCenter().getX(), heightMapData.getGridResolutionXY(), centerIndex);
      int yIndex = HeightMapTools.coordinateToIndex(waypoint.getY(), heightMapData.getGridCenter().getY(), heightMapData.getGridResolutionXY(), centerIndex);

      double previousHeight = waypoint.getZ();
      double heightSampleDelta = 0.08;
      double minHeight = previousHeight - heightSampleDelta;
      double maxHeight = previousHeight + heightSampleDelta;

      double runningSum = 0.0;
      int numberOfSamples = 0;

      for (int i = 0; i < xSnapOffsets.size(); i++)
      {
         int xQuery = xIndex + xSnapOffsets.get(i);
         int yQuery = yIndex + ySnapOffsets.get(i);
         double heightQuery = heightMapData.getHeightAt(xQuery, yQuery);

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
