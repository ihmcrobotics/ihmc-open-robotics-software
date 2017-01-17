package us.ihmc.footstepPlanning.graphSearch;

import java.util.ArrayList;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector2d;

import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.LineSegment2d;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.math.frames.YoFrameVector;

public class PlanarRegionBaseOfCliffAvoider
{
   private boolean visualize = true;
   
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final YoFrameVector yoShiftVectorInSoleFrame = new YoFrameVector("shiftVectorInSoleFrame", null, registry);
   private final DoubleYoVariable maximumCliffXInSoleFrame = new DoubleYoVariable("maximumCliffXInSoleFrame", registry);
   private final DoubleYoVariable maximumCliffYInSoleFrame = new DoubleYoVariable("maximumCliffYInSoleFrame", registry);
   private final DoubleYoVariable maximumCliffZInSoleFrame = new DoubleYoVariable("maximumCliffZInSoleFrame", registry);

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

   public void shiftAwayFromCliffBottoms(BipedalFootstepPlannerParameters parameters, PlanarRegionsList planarRegionsList, BipedalFootstepPlannerNode nodeToExpand)
   {
      double cliffHeightToShiftAwayFrom = parameters.getCliffHeightToShiftAwayFrom();
      double minimumDistanceFromCliffBottoms = parameters.getMinimumDistanceFromCliffBottoms();

      if ((cliffHeightToShiftAwayFrom <= 0.0) || (minimumDistanceFromCliffBottoms <= 0.0))
         return;

      RigidBodyTransform soleTransform = new RigidBodyTransform();
      nodeToExpand.getSoleTransform(soleTransform);
      RigidBodyTransform inverseSoleTransform = new RigidBodyTransform(soleTransform);
      inverseSoleTransform.invert();

      //TODO: Too many hard coded numbers around here...

      LineSegment2d middleToLeft = new LineSegment2d(0.0, 0.0, 0.0, minimumDistanceFromCliffBottoms);
      LineSegment2d middleToRight = new LineSegment2d(0.0, 0.0, 0.0, -minimumDistanceFromCliffBottoms);
      LineSegment2d frontToLeft = new LineSegment2d(forwardBackForSidewaysLook, 0.0, forwardBackForSidewaysLook, minimumDistanceFromCliffBottoms);
      LineSegment2d frontToRight = new LineSegment2d(forwardBackForSidewaysLook, 0.0, forwardBackForSidewaysLook, -minimumDistanceFromCliffBottoms);
      LineSegment2d backToLeft = new LineSegment2d(-forwardBackForSidewaysLook, 0.0, -forwardBackForSidewaysLook, minimumDistanceFromCliffBottoms);
      LineSegment2d backToRight = new LineSegment2d(-forwardBackForSidewaysLook, 0.0, -forwardBackForSidewaysLook, -minimumDistanceFromCliffBottoms);
      
      LineSegment2d backToBackFurther = new LineSegment2d(-0.0, 0.0, -minimumDistanceFromCliffBottoms, 0.0);
      LineSegment2d frontToFrontFurther = new LineSegment2d(-0.0, 0.0, minimumDistanceFromCliffBottoms, 0.0);

      ArrayList<LineSegment2d> lineSegmentsInSoleFrame = new ArrayList<>();
      lineSegmentsInSoleFrame.add(middleToLeft);
      lineSegmentsInSoleFrame.add(middleToRight);
      lineSegmentsInSoleFrame.add(frontToLeft);
      lineSegmentsInSoleFrame.add(frontToRight);
      lineSegmentsInSoleFrame.add(backToLeft);
      lineSegmentsInSoleFrame.add(backToRight);
      lineSegmentsInSoleFrame.add(backToBackFurther);
      lineSegmentsInSoleFrame.add(frontToFrontFurther);

      Point3d highestPointInSoleFrame = new Point3d();
      LineSegment2d highestLineSegmentInSoleFrame = new LineSegment2d();
      maximumCliffZInSoleFrame.set(findHighestPointInOriginalSoleFrame(planarRegionsList, soleTransform, inverseSoleTransform, lineSegmentsInSoleFrame, highestPointInSoleFrame, highestLineSegmentInSoleFrame));
      maximumCliffXInSoleFrame.set(highestPointInSoleFrame.getX());
      maximumCliffYInSoleFrame.set(highestPointInSoleFrame.getY());

      if (maximumCliffZInSoleFrame.getDoubleValue() > cliffHeightToShiftAwayFrom)
      {
         Vector2d shiftVectorInSoleFrame = new Vector2d(highestPointInSoleFrame.getX(), highestPointInSoleFrame.getY());
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
               beforeAdjustmentPosition.setPosition(nodeToExpand.getSolePosition());
            }
            nodeToExpand.shiftInSoleFrame(shiftVectorInSoleFrame);
            if(visualize)
            {
               afterAdjustmentPosition.setPosition(nodeToExpand.getSolePosition());
            }            
         }
      }

      // The following is just to display the graph better...
      if (Double.isInfinite(maximumCliffZInSoleFrame.getDoubleValue()))
      {
         maximumCliffZInSoleFrame.set(-0.02);
      }
   }
   
   private double findHighestPointInOriginalSoleFrame(PlanarRegionsList planarRegionsList, RigidBodyTransform soleTransform, RigidBodyTransform inverseSoleTransform, ArrayList<LineSegment2d> lineSegmentsInSoleFrame,
                                                      Point3d highestPointInSoleFrameToPack, LineSegment2d highestLineSegmentInSoleFrameToPack)
     {
        double maxZInSoleFrame = Double.NEGATIVE_INFINITY;

        LineSegment2d lineSegmentInWorldFrame = new LineSegment2d();
        Point3d pointOneInWorldFrame = new Point3d();
        Point3d pointTwoInWorldFrame = new Point3d();

        for (LineSegment2d lineSegmentInSoleFrame : lineSegmentsInSoleFrame)
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
              ArrayList<Point2d[]> intersectionsInPlaneFrameToPack = new ArrayList<>();
              intersectingRegion.getLineSegmentIntersectionsWhenProjectedVertically(lineSegmentInWorldFrame, intersectionsInPlaneFrameToPack);
              for (int i = 0; i < intersectionsInPlaneFrameToPack.size(); i++)
              {
                 Point2d[] points = intersectionsInPlaneFrameToPack.get(i);
                 for (int j = 0; j < points.length; j++)
                 {
                    Point2d point = points[j];
                    RigidBodyTransform regionTransformToWorld = new RigidBodyTransform();
                    intersectingRegion.getTransformToWorld(regionTransformToWorld);
                    Point3d pointInOriginalSoleFrame = new Point3d(point.getX(), point.getY(), 0.0);
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
