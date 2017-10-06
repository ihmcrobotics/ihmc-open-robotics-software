package us.ihmc.footstepPlanning.graphSearch.footstepSnapping;

import java.util.ArrayList;

import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.footstepPlanning.graphSearch.BipedalFootstepPlannerNodeUtils;
import us.ihmc.footstepPlanning.graphSearch.BipedalFootstepPlannerParameters;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.math.frames.YoFrameVector;

public class PlanarRegionBaseOfCliffAvoider
{
   private boolean visualize = true;
   
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final YoFrameVector yoShiftVectorInSoleFrame = new YoFrameVector("shiftVectorInSoleFrame", null, registry);
   private final YoDouble maximumCliffXInSoleFrame = new YoDouble("maximumCliffXInSoleFrame", registry);
   private final YoDouble maximumCliffYInSoleFrame = new YoDouble("maximumCliffYInSoleFrame", registry);
   private final YoDouble maximumCliffZInSoleFrame = new YoDouble("maximumCliffZInSoleFrame", registry);

   private final YoGraphicPosition beforeAdjustmentPosition, afterAdjustmentPosition;

   //TODO: Put in parameters or use the actual footstep for these checks...
   private static final double forwardBackForSidewaysLook = 0.1;

   public PlanarRegionBaseOfCliffAvoider(YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      parentRegistry.addChild(registry);
      
      if (yoGraphicsListRegistry == null)
      {
         visualize = false;
      }

      if (visualize)
      {
         beforeAdjustmentPosition = new YoGraphicPosition("beforeAdjustmentPosition", "", registry, 0.02, YoAppearance.Red());
         afterAdjustmentPosition = new YoGraphicPosition("afterAdjustmentPosition", "", registry, 0.02, YoAppearance.Green());
         yoGraphicsListRegistry.registerYoGraphic(getClass().getSimpleName(), beforeAdjustmentPosition);
         yoGraphicsListRegistry.registerYoGraphic(getClass().getSimpleName(), afterAdjustmentPosition);
      }
      else
      {
         beforeAdjustmentPosition = null;
         afterAdjustmentPosition = null;
      }
   }

   public void shiftAwayFromCliffBottoms(BipedalFootstepPlannerParameters parameters, PlanarRegionsList planarRegionsList, RigidBodyTransform soleTransform)
   {
      double cliffHeightToShiftAwayFrom = parameters.getCliffHeightToShiftAwayFrom();
      double minimumDistanceFromCliffBottoms = parameters.getMinimumDistanceFromCliffBottoms();

      if ((cliffHeightToShiftAwayFrom <= 0.0) || (minimumDistanceFromCliffBottoms <= 0.0))
         return;

      RigidBodyTransform inverseSoleTransform = new RigidBodyTransform(soleTransform);
      inverseSoleTransform.invert();

      //TODO: Too many hard coded numbers around here...

      LineSegment2D middleToLeft = new LineSegment2D(0.0, 0.0, 0.0, minimumDistanceFromCliffBottoms);
      LineSegment2D middleToRight = new LineSegment2D(0.0, 0.0, 0.0, -minimumDistanceFromCliffBottoms);
      LineSegment2D frontToLeft = new LineSegment2D(forwardBackForSidewaysLook, 0.0, forwardBackForSidewaysLook, minimumDistanceFromCliffBottoms);
      LineSegment2D frontToRight = new LineSegment2D(forwardBackForSidewaysLook, 0.0, forwardBackForSidewaysLook, -minimumDistanceFromCliffBottoms);
      LineSegment2D backToLeft = new LineSegment2D(-forwardBackForSidewaysLook, 0.0, -forwardBackForSidewaysLook, minimumDistanceFromCliffBottoms);
      LineSegment2D backToRight = new LineSegment2D(-forwardBackForSidewaysLook, 0.0, -forwardBackForSidewaysLook, -minimumDistanceFromCliffBottoms);
      
      LineSegment2D backToBackFurther = new LineSegment2D(-0.0, 0.0, -minimumDistanceFromCliffBottoms, 0.0);
      LineSegment2D frontToFrontFurther = new LineSegment2D(-0.0, 0.0, minimumDistanceFromCliffBottoms, 0.0);

      ArrayList<LineSegment2D> lineSegmentsInSoleFrame = new ArrayList<>();
      lineSegmentsInSoleFrame.add(middleToLeft);
      lineSegmentsInSoleFrame.add(middleToRight);
      lineSegmentsInSoleFrame.add(frontToLeft);
      lineSegmentsInSoleFrame.add(frontToRight);
      lineSegmentsInSoleFrame.add(backToLeft);
      lineSegmentsInSoleFrame.add(backToRight);
      lineSegmentsInSoleFrame.add(backToBackFurther);
      lineSegmentsInSoleFrame.add(frontToFrontFurther);

      Point3D highestPointInSoleFrame = new Point3D();
      LineSegment2D highestLineSegmentInSoleFrame = new LineSegment2D();
      maximumCliffZInSoleFrame.set(findHighestPointInOriginalSoleFrame(planarRegionsList, soleTransform, inverseSoleTransform, lineSegmentsInSoleFrame, highestPointInSoleFrame, highestLineSegmentInSoleFrame));
      maximumCliffXInSoleFrame.set(highestPointInSoleFrame.getX());
      maximumCliffYInSoleFrame.set(highestPointInSoleFrame.getY());

      if (maximumCliffZInSoleFrame.getDoubleValue() > cliffHeightToShiftAwayFrom)
      {
         Vector2D shiftVectorInSoleFrame = new Vector2D(highestPointInSoleFrame.getX(), highestPointInSoleFrame.getY());
         shiftVectorInSoleFrame.sub(highestLineSegmentInSoleFrame.getFirstEndpointCopy());

         if (shiftVectorInSoleFrame.length() < minimumDistanceFromCliffBottoms)
         {
            double distanceToShift = minimumDistanceFromCliffBottoms - shiftVectorInSoleFrame.length();
            shiftVectorInSoleFrame.normalize();
            shiftVectorInSoleFrame.scale(-distanceToShift);
            
            yoShiftVectorInSoleFrame.set(shiftVectorInSoleFrame.getX(), shiftVectorInSoleFrame.getY(), 0.0);
            //TODO: This may shift a foot into another obstacle. Need something more here...

            if (visualize)
            {
               Point3D solePosition = new Point3D();
               soleTransform.getTranslation(solePosition);
               beforeAdjustmentPosition.setPosition(solePosition);
            }

            BipedalFootstepPlannerNodeUtils.shiftInSoleFrame(shiftVectorInSoleFrame, soleTransform);

            if(visualize)
            {
               Point3D solePosition = new Point3D();
               soleTransform.getTranslation(solePosition);
               afterAdjustmentPosition.setPosition(solePosition);
            }
         }
      }

      // The following is just to display the graph better...
      if (Double.isInfinite(maximumCliffZInSoleFrame.getDoubleValue()))
      {
         maximumCliffZInSoleFrame.set(-0.02);
      }
   }
   
   private double findHighestPointInOriginalSoleFrame(PlanarRegionsList planarRegionsList, RigidBodyTransform soleTransform, RigidBodyTransform inverseSoleTransform, ArrayList<LineSegment2D> lineSegmentsInSoleFrame,
                                                      Point3D highestPointInSoleFrameToPack, LineSegment2D highestLineSegmentInSoleFrameToPack)
     {
        double maxZInSoleFrame = Double.NEGATIVE_INFINITY;

        LineSegment2D lineSegmentInWorldFrame = new LineSegment2D();
        Point3D pointOneInWorldFrame = new Point3D();
        Point3D pointTwoInWorldFrame = new Point3D();

        for (LineSegment2D lineSegmentInSoleFrame : lineSegmentsInSoleFrame)
        {
           pointOneInWorldFrame.set(lineSegmentInSoleFrame.getFirstEndpointX(), lineSegmentInSoleFrame.getFirstEndpointY(), 0.0);
           pointTwoInWorldFrame.set(lineSegmentInSoleFrame.getSecondEndpointX(), lineSegmentInSoleFrame.getSecondEndpointY(), 0.0);

           soleTransform.transform(pointOneInWorldFrame);
           soleTransform.transform(pointTwoInWorldFrame);

           lineSegmentInWorldFrame.set(pointOneInWorldFrame.getX(), pointOneInWorldFrame.getY(), pointTwoInWorldFrame.getX(), pointTwoInWorldFrame.getY());

           ArrayList<PlanarRegion> intersectingRegionsToPack = new ArrayList<>();
           planarRegionsList.findPlanarRegionsIntersectingLineSegment(lineSegmentInWorldFrame, intersectingRegionsToPack);
           for (PlanarRegion intersectingRegion : intersectingRegionsToPack)
           {
              ArrayList<Point2D[]> intersectionsInPlaneFrameToPack = new ArrayList<>();
              intersectingRegion.getLineSegmentIntersectionsWhenProjectedVertically(lineSegmentInWorldFrame, intersectionsInPlaneFrameToPack);
              for (int i = 0; i < intersectionsInPlaneFrameToPack.size(); i++)
              {
                 Point2D[] points = intersectionsInPlaneFrameToPack.get(i);
                 for (int j = 0; j < points.length; j++)
                 {
                    Point2D point = points[j];
                    RigidBodyTransform regionTransformToWorld = new RigidBodyTransform();
                    intersectingRegion.getTransformToWorld(regionTransformToWorld);
                    Point3D pointInOriginalSoleFrame = new Point3D(point.getX(), point.getY(), 0.0);
                    regionTransformToWorld.transform(pointInOriginalSoleFrame);
                    inverseSoleTransform.transform(pointInOriginalSoleFrame);

                    if (pointInOriginalSoleFrame.getZ() > maxZInSoleFrame)
                    {
                       maxZInSoleFrame = pointInOriginalSoleFrame.getZ();
                       highestPointInSoleFrameToPack.set(pointInOriginalSoleFrame);
                       highestLineSegmentInSoleFrameToPack.set(lineSegmentInSoleFrame);
                    }
                 }
              }
           }
        }

        return maxZInSoleFrame;
     }

}
