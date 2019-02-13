package us.ihmc.commonWalkingControlModules.controlModules.foot.wobble;

import java.awt.Color;

import us.ihmc.commonWalkingControlModules.controlModules.foot.FeetManager;
import us.ihmc.commonWalkingControlModules.momentumBasedController.ParameterProvider;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLine2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoFrameConvexPolygon2D;

public class FootRotationHelper
{
   private static final boolean visualize = false;

   private final RobotSide side;
   private final FootRotationInformation rotationInformation;
   private final FixedFramePoint3DBasics desiredCop;

   private final FramePoint2DBasics intersection1 = new FramePoint2D();
   private final FramePoint2DBasics intersection2 = new FramePoint2D();
   private final FrameConvexPolygon2DBasics polygonA = new FrameConvexPolygon2D();
   private final FrameConvexPolygon2DBasics polygonB = new FrameConvexPolygon2D();
   private FrameConvexPolygon2DReadOnly bestPolygon;

   private final YoFrameConvexPolygon2D yoPolygonA;
   private final YoFrameConvexPolygon2D yoPolygonB;

   private final DoubleProvider areaPecentToSelect;
   private final YoBoolean sideSelected;

   @SuppressWarnings("unused")
   public FootRotationHelper(RobotSide side, ReferenceFrame soleFrame, FootRotationInformation rotationInformation, YoVariableRegistry parentRegistry,
                             YoGraphicsListRegistry graphicsRegistry)
   {
      this.side = side;
      this.rotationInformation = rotationInformation;

      desiredCop = new FramePoint3D(soleFrame);
      polygonA.clear(soleFrame);
      polygonB.clear(soleFrame);

      YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName() + side.getPascalCaseName());
      parentRegistry.addChild(registry);

      String feetManagerName = FeetManager.class.getSimpleName();
      String paramRegistryName = getClass().getSimpleName() + "Parameters";
      areaPecentToSelect = ParameterProvider.getOrCreateParameter(feetManagerName, paramRegistryName, "AreaPecentToSelect", registry, 0.7);
      sideSelected = new YoBoolean(side.getPascalCaseName() + "SelectedSide", registry);

      if (graphicsRegistry != null && visualize)
      {
         yoPolygonA = new YoFrameConvexPolygon2D(side.getPascalCaseName() + "PolygonA", ReferenceFrame.getWorldFrame(), 5, registry);
         yoPolygonB = new YoFrameConvexPolygon2D(side.getPascalCaseName() + "PolygonB", ReferenceFrame.getWorldFrame(), 5, registry);
         YoArtifactPolygon polygonAViz = new YoArtifactPolygon(side.getPascalCaseName() + "PolygonAViz", yoPolygonA, Color.BLUE, false);
         YoArtifactPolygon polygonBViz = new YoArtifactPolygon(side.getPascalCaseName() + "PolygonBViz", yoPolygonB, Color.CYAN, false);
         graphicsRegistry.registerArtifact(getClass().getSimpleName(), polygonAViz);
         graphicsRegistry.registerArtifact(getClass().getSimpleName(), polygonBViz);
      }
      else
      {
         yoPolygonA = null;
         yoPolygonB = null;
      }

      reset();
   }

   public void compute(FrameLine2DReadOnly lineOfRotation, FrameConvexPolygon2DReadOnly footPolygon, FramePoint2DReadOnly icp)
   {
      ReferenceFrame soleFrame = polygonA.getReferenceFrame();
      footPolygon.checkReferenceFrameMatch(soleFrame);
      lineOfRotation.checkReferenceFrameMatch(soleFrame);
      icp.checkReferenceFrameMatch(soleFrame);

      // Do not update the foothold continuously. If we have committed to a partial foothold stick with it.
      if (sideSelected.getValue())
      {
         return;
      }

      if (!computeIntersections(lineOfRotation, footPolygon, intersection1, intersection2))
      {
         hideViz();
         return;
      }

      int index1 = footPolygon.getClosestEdgeIndex(intersection1);
      int index2 = footPolygon.getClosestEdgeIndex(intersection2);
      extractPartialFoothold(footPolygon, intersection1, intersection2, index1, index2, polygonA);
      extractPartialFoothold(footPolygon, intersection1, intersection2, index2, index1, polygonB);
      bestPolygon = findBestPolygon(polygonA, polygonB, icp, areaPecentToSelect.getValue());

      updateViz();
      sideSelected.set(true);
   }

   public void adjustICPPlan()
   {
      if (!checkIfComputed())
      {
         return;
      }

      desiredCop.set(bestPolygon.getCentroid());
      rotationInformation.setRotating(side, desiredCop);
   }

   public void reduceFoothold()
   {
      if (!checkIfComputed())
      {
         return;
      }

      // TODO.
   }

   private boolean checkIfComputed()
   {
      if (!sideSelected.getValue())
      {
         LogTools.warn("Neet to compute adjusted polygon first.");
         return false;
      }
      return true;
   }

   private static FrameConvexPolygon2DReadOnly findBestPolygon(FrameConvexPolygon2DReadOnly polygonA, FrameConvexPolygon2DReadOnly polygonB,
                                                               FramePoint2DReadOnly icp, double areaPercentToSelect)
   {
      // 1. Check the polygon areas. If one is really large go with that.
      double totalArea = polygonA.getArea() + polygonB.getArea();
      if (polygonA.getArea() > areaPercentToSelect * totalArea)
      {
         return polygonA;
      }
      if (polygonB.getArea() > areaPercentToSelect * totalArea)
      {
         return polygonB;
      }

      // 2. Check if the ICP is in one of the polygons. If that is the case use that polygon.
      if (polygonA.isPointInside(icp))
      {
         return polygonA;
      }
      if (polygonB.isPointInside(icp))
      {
         return polygonB;
      }

      // 3. Use the polygon that is closer to the ICP.
      return polygonA.distance(icp) > polygonB.distance(icp) ? polygonB : polygonA;
   }

   private static boolean computeIntersections(FrameLine2DReadOnly lineOfRotation, FrameConvexPolygon2DReadOnly footPolygon,
                                               FramePoint2DBasics intersection1ToPack, FramePoint2DBasics intersection2ToPack)
   {
      if (footPolygon.intersectionWith(lineOfRotation, intersection1ToPack, intersection2ToPack) < 2)
      {
         LogTools.warn("Line of rotation does not fully intersect the support polygon.");
         return false;
      }
      if (intersection1ToPack.epsilonEquals(intersection2ToPack, 1.0e-5))
      {
         LogTools.warn("Line of rotation goes straight through a vertex.");
         return false;
      }
      return true;
   }

   private static void extractPartialFoothold(FrameConvexPolygon2DReadOnly footPolygon, FramePoint2DReadOnly intersection1, FramePoint2DReadOnly intersection2,
                                              int startIdx, int endIdx, FrameConvexPolygon2DBasics polygonToPack)
   {
      polygonToPack.clear();
      polygonToPack.addVertex(intersection1);
      polygonToPack.addVertex(intersection2);
      int adjustedEndIndex = endIdx > startIdx ? endIdx : endIdx + footPolygon.getNumberOfVertices();
      for (int i = startIdx + 1; i <= adjustedEndIndex; i++)
      {
         polygonToPack.addVertex(footPolygon.getVertex(i % footPolygon.getNumberOfVertices()));
      }
      polygonToPack.update();
   }

   private void updateViz()
   {
      if (yoPolygonA != null)
      {
         yoPolygonA.setMatchingFrame(polygonA, false);
         yoPolygonB.setMatchingFrame(polygonB, false);
      }
   }

   private void hideViz()
   {
      if (yoPolygonA != null)
      {
         yoPolygonA.setToNaN();
         yoPolygonB.setToNaN();
      }
   }

   public void reset()
   {
      hideViz();
      sideSelected.set(false);
      rotationInformation.reset(side);
   }
}
