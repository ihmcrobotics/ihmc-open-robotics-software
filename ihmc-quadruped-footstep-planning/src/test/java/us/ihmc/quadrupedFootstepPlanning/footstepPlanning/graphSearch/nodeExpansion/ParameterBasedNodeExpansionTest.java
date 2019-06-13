package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.nodeExpansion;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.ContinuousIntegrationTools;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.pathPlanning.visibilityGraphs.tools.PlanarRegionTools;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.FootstepPlanningRandomTools;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapData;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapper;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.footstepSnapping.PlanarRegionsListPointSnapper;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapData;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapper;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.footstepSnapping.SimplePlanarRegionFootstepNodeSnapper;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.nodeChecking.SnapBasedNodeChecker;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettings;
import us.ihmc.robotics.geometry.ConvexPolygonTools;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

import java.util.HashSet;
import java.util.Random;

import static us.ihmc.robotics.Assert.*;

public class ParameterBasedNodeExpansionTest
{
   private static final int iters = 1000;
   private static final RobotQuadrant quadrantToCheck = RobotQuadrant.HIND_LEFT;

   private static final double epsilon = 1e-7;

   private static final double stanceLength = 1.0;
   private static final double stanceWidth = 0.5;
   private static final boolean visualize = true;
   private static final QuadrantDependentList<AppearanceDefinition> colorDefinitions = new QuadrantDependentList<>(YoAppearance.Red(), YoAppearance.Yellow(),
                                                                                                                   YoAppearance.Blue(),
                                                                                                                   YoAppearance.Beige());

   @Test
   public void testExpandNodeWithBaseAtOrigin()
   {
      FootstepPlannerParameters parameters = new DefaultFootstepPlannerParameters();
      QuadrupedXGaitSettings xGaitSettingsReadOnly = new QuadrupedXGaitSettings();
      xGaitSettingsReadOnly.setStanceWidth(stanceWidth);
      xGaitSettingsReadOnly.setStanceLength(stanceLength);

      FootstepNodeExpansion expansion = new ParameterBasedNodeExpansion(parameters, xGaitSettingsReadOnly);

      double yaw = 0.0;
      FootstepNode baseNode = new FootstepNode(quadrantToCheck, 0.5 * stanceLength, 0.5 * stanceWidth, 0.5 * stanceLength, -0.5 * stanceWidth,
                                               -0.5 * stanceLength, 0.5 * stanceWidth, -0.5 * stanceLength, -0.5 * stanceWidth, yaw, stanceLength, stanceWidth);

      HashSet<FootstepNode> expandedNodes = expansion.expandNode(baseNode);
      for (FootstepNode expandedNode : expandedNodes)
      {
         assertFalse("baseNode " + expandedNode + " is not supposed to be equal to " + baseNode, expandedNode.quadrantGeometricallyEquals(baseNode));
      }

      if (visualize)
         visualizeNodes(parameters, expandedNodes, baseNode);

      RobotQuadrant expectedNewQuadrant = quadrantToCheck.getNextRegularGaitSwingQuadrant();

      for (FootstepNode node : expandedNodes)
      {
         assertEquals(expectedNewQuadrant, node.getMovingQuadrant());

         for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         {
            if (expectedNewQuadrant == robotQuadrant)
            {
               Point2DReadOnly center = baseNode.getOrComputeXGaitCenterPoint();
               Point2D foot = new Point2D(center);
               foot.add(0.5 * expectedNewQuadrant.getEnd().negateIfHindEnd(baseNode.getNominalStanceLength()),
                        0.5 * expectedNewQuadrant.getSide().negateIfRightSide(baseNode.getNominalStanceWidth()));
               double lowerXBound = foot.getX() + (expectedNewQuadrant.isQuadrantInFront() ? parameters.getMinimumFrontStepLength() : parameters.getMinimumHindStepLength());
               double upperXBound = foot.getX() + (expectedNewQuadrant.isQuadrantInFront() ? parameters.getMaximumFrontStepLength() : parameters.getMaximumHindStepLength());
               double lowerYBound = foot.getY() - (robotQuadrant.getSide() == RobotSide.LEFT ? -parameters.getMinimumStepWidth() : parameters.getMaximumStepWidth());
               double upperYBound = foot.getY() + (robotQuadrant.getSide() == RobotSide.LEFT ? parameters.getMaximumStepWidth() : -parameters.getMinimumStepWidth());
               double xFoot = node.getX(robotQuadrant);
               double yFoot = node.getY(robotQuadrant);
               assertTrue(robotQuadrant.getCamelCaseName() + " foot X " + xFoot + " is not within bounds " + lowerXBound + " < x < " + upperXBound, MathTools.intervalContains(xFoot, lowerXBound, upperXBound, FootstepNode.gridSizeXY));
               assertTrue(robotQuadrant.getCamelCaseName() + " foot Y " + yFoot + " is not within bounds " + lowerYBound + " < x < " + upperYBound, MathTools.intervalContains(yFoot, lowerYBound, upperYBound, FootstepNode.gridSizeXY));
            }
            else
            {
               assertEquals(baseNode.getX(robotQuadrant), node.getX(robotQuadrant), epsilon);
               assertEquals(baseNode.getY(robotQuadrant), node.getY(robotQuadrant), epsilon);
            }
         }
      }
   }

   @Test
   public void testExpandNodeWithTranslatedAndRotated()
   {
      FootstepPlannerParameters parameters = new DefaultFootstepPlannerParameters();
      QuadrupedXGaitSettings xGaitSettingsReadOnly = new QuadrupedXGaitSettings();
      xGaitSettingsReadOnly.setStanceLength(stanceLength);
      xGaitSettingsReadOnly.setStanceWidth(stanceWidth);
      FootstepNodeExpansion expansion = new ParameterBasedNodeExpansion(parameters, xGaitSettingsReadOnly);

      PoseReferenceFrame centerFrame = new PoseReferenceFrame("centerFrame", ReferenceFrame.getWorldFrame());
      FramePoint2D frontLeft = new FramePoint2D(centerFrame, 0.5 * stanceLength, 0.5 * stanceWidth);
      FramePoint2D frontRight = new FramePoint2D(centerFrame, 0.5 * stanceLength, -0.5 * stanceWidth);
      FramePoint2D hindLeft = new FramePoint2D(centerFrame, -0.5 * stanceLength, 0.5 * stanceWidth);
      FramePoint2D hindRight = new FramePoint2D(centerFrame, -0.5 * stanceLength, -0.5 * stanceWidth);

      FramePose3D poseInWorld = new FramePose3D();
      poseInWorld.getPosition().set(1.1, -0.5, 0.0);
      poseInWorld.getOrientation().setToYawQuaternion(Math.PI / 4.0);
      centerFrame.setPoseAndUpdate(poseInWorld);

      frontLeft.changeFrame(ReferenceFrame.getWorldFrame());
      frontRight.changeFrame(ReferenceFrame.getWorldFrame());
      hindLeft.changeFrame(ReferenceFrame.getWorldFrame());
      hindRight.changeFrame(ReferenceFrame.getWorldFrame());

      double yaw = FootstepNode.computeNominalYaw(frontLeft.getX(), frontLeft.getY(), frontRight.getX(), frontRight.getY(), hindLeft.getX(), hindLeft.getY(),
                                                  hindRight.getX(), hindRight.getY());

      FootstepNode baseNode = new FootstepNode(quadrantToCheck, frontLeft, frontRight, hindLeft, hindRight, yaw, stanceLength, stanceWidth);

      HashSet<FootstepNode> expandedNodes = expansion.expandNode(baseNode);

      if (visualize)
         visualizeNodes(parameters, expandedNodes, baseNode);

      RobotQuadrant expectedNewQuadrant = quadrantToCheck.getNextRegularGaitSwingQuadrant();

      for (FootstepNode node : expandedNodes)
      {
         assertEquals(expectedNewQuadrant, node.getMovingQuadrant());

         for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         {
            if (expectedNewQuadrant == robotQuadrant)
               continue;

            assertEquals(baseNode.getX(robotQuadrant), node.getX(robotQuadrant), epsilon);
            assertEquals(baseNode.getY(robotQuadrant), node.getY(robotQuadrant), epsilon);
         }
      }
   }

   @Test
   public void testWeirdFeet()
   {
      FootstepPlannerParameters parameters = new DefaultFootstepPlannerParameters()
      {
         @Override
         public double getMaximumFrontStepReach()
         {
            return 0.7;
         }

         public double getMaximumFrontStepLength()
         {
            return 0.6;
         }

         @Override
         public double getMinimumFrontStepLength()
         {
            return -0.3;
         }

         @Override
         public double getMinimumStepWidth()
         {
            return -0.3;
         }

         @Override
         public double getMaximumStepWidth()
         {
            return 0.35;
         }
      };
      QuadrupedXGaitSettings xGaitSettingsReadOnly = new QuadrupedXGaitSettings();
      xGaitSettingsReadOnly.setStanceLength(stanceLength);
      xGaitSettingsReadOnly.setStanceWidth(stanceWidth);
      FootstepNodeExpansion expansion = new ParameterBasedNodeExpansion(parameters, xGaitSettingsReadOnly);

      double frontLeftX = 0.5;
      double frontLeftY = 0.5;
      double gridSizeXY = FootstepNode.gridSizeXY;

      FramePoint2D frontLeft = new FramePoint2D(ReferenceFrame.getWorldFrame(), frontLeftX, frontLeftY);
      FramePoint2D frontRight = new FramePoint2D(ReferenceFrame.getWorldFrame(), frontLeftX + 6 * gridSizeXY, frontLeftY - 10 * gridSizeXY);
      FramePoint2D hindLeft = new FramePoint2D(ReferenceFrame.getWorldFrame(), frontLeftX - 13 * gridSizeXY, frontLeftY - 2 * gridSizeXY);
      FramePoint2D hindRight = new FramePoint2D(ReferenceFrame.getWorldFrame(), frontLeftX - 8 * gridSizeXY, frontLeftY - 6 * gridSizeXY);

      RobotQuadrant expectedNewQuadrant = RobotQuadrant.HIND_LEFT;

      double yaw = FootstepNode.computeNominalYaw(frontLeft.getX(), frontLeft.getY(), frontRight.getX(), frontRight.getY(), hindLeft.getX(), hindLeft.getY(),
                                                  hindRight.getX(), hindRight.getY());

      FootstepNode baseNode = new FootstepNode(expectedNewQuadrant.getNextReversedRegularGaitSwingQuadrant(), frontLeft, frontRight, hindLeft, hindRight, yaw,
                                               stanceLength, stanceWidth);

      HashSet<FootstepNode> expandedNodes = expansion.expandNode(baseNode);

      if (visualize)
         visualizeNodes(parameters, expandedNodes, baseNode);


      for (FootstepNode node : expandedNodes)
      {
         assertEquals(expectedNewQuadrant, node.getMovingQuadrant());

         for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         {
            if (expectedNewQuadrant == robotQuadrant)
               continue;

            assertEquals(baseNode.getX(robotQuadrant), node.getX(robotQuadrant), epsilon);
            assertEquals(baseNode.getY(robotQuadrant), node.getY(robotQuadrant), epsilon);
         }
      }
   }


   @Test
   public void testCheckNodeIsFarEnoughFromOtherFoot()
   {
      Random random = new Random(1738L);

      for (int iter = 0; iter < iters; iter++)
      {
         Vector2D requiredClearance = EuclidCoreRandomTools.nextVector2D(random);
         requiredClearance.setX(Math.abs(requiredClearance.getX()));
         requiredClearance.setY(Math.abs(requiredClearance.getY()));

         Point2DReadOnly previousNodePoint = EuclidCoreRandomTools.nextPoint2D(random);

         String message = "iter = " + iter + " failed.";

         // check right on the edge
         Vector2D offset = new Vector2D(requiredClearance);

         Point2D positionToCheck = new Point2D();
         positionToCheck.set(previousNodePoint);
         positionToCheck.add(offset);

         assertFalse(message, ParameterBasedNodeExpansion
               .checkNodeIsFarEnoughFromOtherFoot(positionToCheck, requiredClearance, previousNodePoint.getX(), previousNodePoint.getY()));

         positionToCheck.set(previousNodePoint);
         positionToCheck.sub(offset);

         assertFalse(message, ParameterBasedNodeExpansion
               .checkNodeIsFarEnoughFromOtherFoot(positionToCheck, requiredClearance, previousNodePoint.getX(), previousNodePoint.getY()));

         // check slightly within the edge
         offset.set(requiredClearance);
         offset.scale(0.9);

         positionToCheck.set(previousNodePoint);
         positionToCheck.add(offset);

         assertFalse(message, ParameterBasedNodeExpansion
               .checkNodeIsFarEnoughFromOtherFoot(positionToCheck, requiredClearance, previousNodePoint.getX(), previousNodePoint.getY()));

         positionToCheck.set(previousNodePoint);
         positionToCheck.sub(offset);

         assertFalse(message, ParameterBasedNodeExpansion
               .checkNodeIsFarEnoughFromOtherFoot(positionToCheck, requiredClearance, previousNodePoint.getX(), previousNodePoint.getY()));

         // check slightly outside the edge
         offset.set(requiredClearance);
         offset.scale(1.1);

         positionToCheck.set(previousNodePoint);
         positionToCheck.add(offset);

         assertTrue(message, ParameterBasedNodeExpansion
               .checkNodeIsFarEnoughFromOtherFoot(positionToCheck, requiredClearance, previousNodePoint.getX(), previousNodePoint.getY()));

         positionToCheck.set(previousNodePoint);
         positionToCheck.sub(offset);

         assertTrue(message, ParameterBasedNodeExpansion
               .checkNodeIsFarEnoughFromOtherFoot(positionToCheck, requiredClearance, previousNodePoint.getX(), previousNodePoint.getY()));
      }
   }


   private void visualizeNodes(FootstepPlannerParameters parameters, HashSet<FootstepNode> neighboringNodes, FootstepNode baseNode)
   {
      if (!visualize || ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer())
         return;

      SimulationConstructionSet scs = new SimulationConstructionSet();
      FootstepNodeSnapper snapper = new SimplePlanarRegionFootstepNodeSnapper(parameters, parameters::getProjectInsideDistanceForExpansion,
                                                                              parameters::getProjectInsideUsingConvexHullDuringExpansion, true);
      SnapBasedNodeChecker nodeChecker = new SnapBasedNodeChecker(parameters, snapper);

      Graphics3DObject graphics3DObject = new Graphics3DObject();
      scs.setGroundVisible(false);

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         graphics3DObject.identity();
         graphics3DObject.translate(baseNode.getX(robotQuadrant), baseNode.getY(robotQuadrant), 0.0);
         graphics3DObject.translate(0.0, 0.0, 0.05);
         graphics3DObject.addCone(0.3, 0.05, colorDefinitions.get(robotQuadrant));
      }

      graphics3DObject.identity();
      graphics3DObject.translate(baseNode.getOrComputeXGaitCenterPoint().getX(), baseNode.getOrComputeXGaitCenterPoint().getY(), 0.0);
      graphics3DObject.translate(0.0, 0.0, 0.05);
      graphics3DObject.addSphere(0.04, YoAppearance.Orange());


      // do x gait regions for each guy
      QuadrantDependentList<PoseReferenceFrame> footFrames = getFootFrames(getSnappedStepPositions(baseNode, snapper), baseNode.getStepOrientation());

      ConvexPolygon2D frontReachableRegion = new ConvexPolygon2D();
      frontReachableRegion.addVertex(parameters.getMaximumFrontStepLength(), parameters.getMaximumStepWidth());
      frontReachableRegion.addVertex(parameters.getMaximumFrontStepLength(), parameters.getMinimumStepWidth());
      frontReachableRegion.addVertex(parameters.getMinimumFrontStepLength(), parameters.getMinimumStepWidth());
      frontReachableRegion.addVertex(parameters.getMinimumFrontStepLength(), parameters.getMaximumStepWidth());
      frontReachableRegion.update();
      ConvexPolygon2D hindReachableRegion = new ConvexPolygon2D();
      hindReachableRegion.addVertex(parameters.getMaximumHindStepLength(), parameters.getMaximumStepWidth());
      hindReachableRegion.addVertex(parameters.getMaximumHindStepLength(), parameters.getMinimumStepWidth());
      hindReachableRegion.addVertex(parameters.getMinimumHindStepLength(), parameters.getMinimumStepWidth());
      hindReachableRegion.addVertex(parameters.getMinimumHindStepLength(), parameters.getMaximumStepWidth());
      hindReachableRegion.update();

      RobotQuadrant movingQuadrant = baseNode.getMovingQuadrant().getNextRegularGaitSwingQuadrant();

      ConvexPolygonTools convexPolygonTools = new ConvexPolygonTools();
      ConvexPolygon2D intersectionPolygon = new ConvexPolygon2D();
      intersectionPolygon.addVertex(10.0, 10.0);
      intersectionPolygon.addVertex(10.0, -10.0);
      intersectionPolygon.addVertex(-10.0, -10.0);
      intersectionPolygon.addVertex(-10.0, 10.0);
      intersectionPolygon.update();

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         PoseReferenceFrame xGaitFrame = new PoseReferenceFrame(robotQuadrant.getShortName() + "XGaitFrame", footFrames.get(robotQuadrant));
         double forwardOffset = movingQuadrant.getEnd() == robotQuadrant.getEnd() ? 0.0 : movingQuadrant.isQuadrantInFront() ? baseNode.getNominalStanceLength() : -baseNode.getNominalStanceLength();
         double sideOffset = movingQuadrant.getSide() == robotQuadrant.getSide() ? 0.0 : movingQuadrant.isQuadrantOnLeftSide() ? baseNode.getNominalStanceWidth() : -baseNode.getNominalStanceWidth();
         xGaitFrame.setPositionAndUpdate(new FramePoint3D(footFrames.get(robotQuadrant), forwardOffset, sideOffset, 0.0));

         FrameConvexPolygon2D reachableFootRegion = new FrameConvexPolygon2D(xGaitFrame, (robotQuadrant.isQuadrantInFront() ? frontReachableRegion : hindReachableRegion));
         reachableFootRegion.changeFrame(ReferenceFrame.getWorldFrame());

         ConvexPolygon2D other = new ConvexPolygon2D(intersectionPolygon);

         convexPolygonTools.computeIntersectionOfPolygons(reachableFootRegion, other, intersectionPolygon);

         graphics3DObject.identity();
         for (int i = 0; i < reachableFootRegion.getNumberOfVertices(); i++)
         {
            ConvexPolygon2D polygon2D = new ConvexPolygon2D();
            polygon2D.addVertex(reachableFootRegion.getVertex(i));
            polygon2D.addVertex(reachableFootRegion.getNextVertex(i));
            polygon2D.update();
            graphics3DObject.addExtrudedPolygon(polygon2D, 0.02, colorDefinitions.get(robotQuadrant));
         }
      }

      graphics3DObject.identity();
      graphics3DObject.translate(0.0, 0.0, -0.03);
      graphics3DObject.addExtrudedPolygon(intersectionPolygon, 0.02, YoAppearance.Green());

      for (FootstepNode neighboringNode : neighboringNodes)
      {
         RobotQuadrant quadrant = neighboringNode.getMovingQuadrant();
         Point3D point = new Point3D(neighboringNode.getX(quadrant), neighboringNode.getY(quadrant), 0.0);

         graphics3DObject.identity();
         graphics3DObject.translate(point);

         AppearanceDefinition color;
         if (nodeChecker.isNodeValid(neighboringNode, baseNode))
            color = YoAppearance.Blue();
         else
            color = YoAppearance.Red();
         graphics3DObject.addSphere(0.02, color);
      }

      scs.addStaticLinkGraphics(graphics3DObject);

      scs.setCameraFix(0.0, 0.0, 0.0);
      scs.setCameraPosition(-0.001, 0.0, 15.0);
      scs.startOnAThread();

      ThreadTools.sleepForever();
   }

   private static QuadrantDependentList<Point3D> getSnappedStepPositions(FootstepNode node, FootstepNodeSnapper snapper)
   {
      QuadrantDependentList<Point3D> snappedStepPositions = new QuadrantDependentList<>();

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         FootstepNodeSnapData snapData = snapper.snapFootstepNode(node.getXIndex(robotQuadrant), node.getYIndex(robotQuadrant));
         RigidBodyTransform footSnapTransform = snapData.getSnapTransform();
         Point3D stepPosition = new Point3D(node.getX(robotQuadrant), node.getY(robotQuadrant), 0.0);
         footSnapTransform.transform(stepPosition);
         snappedStepPositions.put(robotQuadrant, stepPosition);
      }

      return snappedStepPositions;
   }


   private static QuadrantDependentList<PoseReferenceFrame> getFootFrames(QuadrantDependentList<Point3D> stepPositions, Orientation3DReadOnly orientation)
   {
      QuadrantDependentList<PoseReferenceFrame> footFrames = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         PoseReferenceFrame footFrame = new PoseReferenceFrame(robotQuadrant.getCamelCaseName() + "FootFrame", ReferenceFrame.getWorldFrame());
         footFrame.setPoseAndUpdate(stepPositions.get(robotQuadrant), orientation);

         footFrames.put(robotQuadrant, footFrame);
      }

      return footFrames;
   }
}
