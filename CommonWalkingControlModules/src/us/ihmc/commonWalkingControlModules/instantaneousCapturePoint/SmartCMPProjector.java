package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

import javax.vecmath.Point2d;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicPosition;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicPosition.GraphicType;

import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.utilities.math.geometry.FrameConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FrameLine2d;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

public class SmartCMPProjector
{
   private boolean VISUALIZE = false;

   private final DynamicGraphicPosition closestEdgeViz, projectedToPointViz, preProjectedCMPViz, edgeOneViz, edgeTwoViz;
   private final BooleanYoVariable cmpProjected;
   private final DoubleYoVariable cmpEdgeProjectionInside;
   private final DoubleYoVariable minICPToCMPProjection;

   private final FrameLine2d icpToCMPLine = new FrameLine2d(ReferenceFrame.getWorldFrame(), new Point2d(), new Point2d(1.0, 0.0));

   
   public void setCMPEdgeProjectionInside(double cmpEdgeProjectionInside)
   {
      this.cmpEdgeProjectionInside.set(cmpEdgeProjectionInside);
   }
   
   public void setMinICPToCMPProjection(double minICPToCMPProjection)
   {
      this.minICPToCMPProjection.set(minICPToCMPProjection);
   }
   
   public SmartCMPProjector(YoVariableRegistry registry, DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
   {
      cmpProjected = new BooleanYoVariable("cmpProjected", registry);
      cmpEdgeProjectionInside = new DoubleYoVariable("cmpEdgeProjectionInside", registry);
      minICPToCMPProjection = new DoubleYoVariable("minICPToCMPProjection", registry);

      cmpEdgeProjectionInside.set(0.06);
      minICPToCMPProjection.set(0.04);
      
      if (dynamicGraphicObjectsListRegistry == null) VISUALIZE = false;

      if (VISUALIZE)
      {
         closestEdgeViz = new DynamicGraphicPosition("closestEdgeViz", "", registry, 0.01, YoAppearance.CadetBlue(), GraphicType.BALL_WITH_CROSS);
         dynamicGraphicObjectsListRegistry.registerDynamicGraphicObject("CMPProjection", closestEdgeViz);
         dynamicGraphicObjectsListRegistry.registerArtifact("CMPProjection", closestEdgeViz.createArtifact());
         
         projectedToPointViz = new DynamicGraphicPosition("projectedToPointViz", "", registry, 0.01, YoAppearance.Gold(), GraphicType.BALL_WITH_CROSS);
         dynamicGraphicObjectsListRegistry.registerDynamicGraphicObject("CMPProjection", projectedToPointViz);
         dynamicGraphicObjectsListRegistry.registerArtifact("CMPProjection", projectedToPointViz.createArtifact());
         
         preProjectedCMPViz = new DynamicGraphicPosition("preProjectedCMPViz", "", registry, 0.01, YoAppearance.Red(), GraphicType.BALL);
         dynamicGraphicObjectsListRegistry.registerDynamicGraphicObject("CMPProjection", preProjectedCMPViz);
         dynamicGraphicObjectsListRegistry.registerArtifact("CMPProjection", preProjectedCMPViz.createArtifact());
         
         edgeOneViz = new DynamicGraphicPosition("edgeOneViz", "", registry, 0.005, YoAppearance.Pink(), GraphicType.BALL);
         dynamicGraphicObjectsListRegistry.registerDynamicGraphicObject("CMPProjection", edgeOneViz);
         dynamicGraphicObjectsListRegistry.registerArtifact("CMPProjection", edgeOneViz.createArtifact());
         
         edgeTwoViz = new DynamicGraphicPosition("edgeTwoViz", "", registry, 0.005, YoAppearance.Beige(), GraphicType.BALL);
         dynamicGraphicObjectsListRegistry.registerDynamicGraphicObject("CMPProjection", edgeTwoViz);
         dynamicGraphicObjectsListRegistry.registerArtifact("CMPProjection", edgeTwoViz.createArtifact());
      }
      else
      {
         closestEdgeViz = null;
         projectedToPointViz = null;
         preProjectedCMPViz = null;
         edgeOneViz = null;
         edgeTwoViz = null;
      }
   }
   
   private final FramePoint2d moveAwayFromEdge = new FramePoint2d();
   private final FramePoint2d otherEdge = new FramePoint2d();
   private final FrameVector2d insideEdgeDirection = new FrameVector2d();

   private final FrameVector2d cmpToICPVector = new FrameVector2d();
   public void projectCMPIntoSupportPolygonIfOutside(FramePoint2d capturePoint, FrameConvexPolygon2d supportPolygon,
         FramePoint2d desiredCMP)
   {
      ReferenceFrame returnFrame = desiredCMP.getReferenceFrame();
      
      desiredCMP.changeFrame(supportPolygon.getReferenceFrame());
      capturePoint.changeFrame(supportPolygon.getReferenceFrame());
      cmpProjected.set(false);
      
      if (VISUALIZE)
      {
         projectedToPointViz.setPositionToNaN();
         edgeOneViz.setPositionToNaN();
         edgeTwoViz.setPositionToNaN();
         
         FramePoint2d desiredCMPInWorld = desiredCMP.changeFrameCopy(ReferenceFrame.getWorldFrame());
         preProjectedCMPViz.setPosition(desiredCMPInWorld.getX(), desiredCMPInWorld.getY(), 0.0);
      }
      
      boolean isCapturePointInside = supportPolygon.isPointInside(capturePoint);
      boolean isCMPInside = supportPolygon.isPointInside(desiredCMP);
      
         // Don't just project the cmp onto the support polygon.
         // Instead, find the first intersection from the cmp to the support polygon
         // along the line segment from the cmp to the capture point. 

         cmpToICPVector.setToZero(capturePoint.getReferenceFrame());
         cmpToICPVector.sub(capturePoint, desiredCMP);
         if (cmpToICPVector.lengthSquared() < 0.003 * 0.003) 
         {
            // If CMP And ICP are really close, do nothing. Not much you can do anyway.
            desiredCMP.changeFrame(returnFrame);
            return;
         }
        
         
         icpToCMPLine.setAndChangeFrame(capturePoint, desiredCMP);
         FramePoint2d[] intersections = supportPolygon.intersectionWith(icpToCMPLine);
         
         if ((intersections == null) || (intersections.length == 0))
         {
            // If no intersections, just give up. Point is outside and no idea how to project it.
            desiredCMP.changeFrame(returnFrame);
            return;
         }
         
         if (VISUALIZE)
         {
            if (intersections.length > 0)
            {
               FramePoint2d intersection0InWorld = intersections[0].changeFrameCopy(ReferenceFrame.getWorldFrame());
               edgeOneViz.setPosition(intersection0InWorld.getX(), intersection0InWorld.getY(), 0.0);
            }
            else
            {
               edgeOneViz.setPositionToNaN();
            }

            if (intersections.length > 1)
            {
               FramePoint2d intersection1InWorld = intersections[1].changeFrameCopy(ReferenceFrame.getWorldFrame());
               edgeTwoViz.setPosition(intersection1InWorld.getX(), intersection1InWorld.getY(), 0.0);
            }
            else
            {
               edgeTwoViz.setPositionToNaN();
            }
         }
         
         if (intersections.length == 1)
         {
            // Not much you can do here. Just set the cmp to the edge and be done with it.
            desiredCMP.set(intersections[0]);
            desiredCMP.changeFrame(returnFrame);
            return;
         }
         else
         {
            if (!isCapturePointInside)
            {
               if (capturePoint.distance(intersections[0]) > capturePoint.distance(intersections[1]))
               {
                  moveAwayFromEdge.setAndChangeFrame(intersections[0]);
                  otherEdge.setAndChangeFrame(intersections[1]);
               }
               else
               {
                  moveAwayFromEdge.setAndChangeFrame(intersections[1]);
                  otherEdge.setAndChangeFrame(intersections[0]);
               }
            }
            else
            {
               insideEdgeDirection.setToZero(intersections[0].getReferenceFrame());
               insideEdgeDirection.sub(intersections[0], intersections[1]);
               
               double dot = insideEdgeDirection.dot(cmpToICPVector);
               if (dot < 0.0)
               {
                  moveAwayFromEdge.setAndChangeFrame(intersections[0]);
                  otherEdge.setAndChangeFrame(intersections[1]);
                  insideEdgeDirection.negate();
               }
               else
               {
                  moveAwayFromEdge.setAndChangeFrame(intersections[1]);
                  otherEdge.setAndChangeFrame(intersections[0]);
               }
            }     
         }
         
         double distanceToEdge = moveAwayFromEdge.distance(desiredCMP);

         
         if (VISUALIZE)
         {
            FramePoint2d closestEdgeInWorld = moveAwayFromEdge.changeFrameCopy(ReferenceFrame.getWorldFrame());
            closestEdgeViz.setPosition(closestEdgeInWorld.getX(), closestEdgeInWorld.getY(), 0.0);
         }

         if ((isCMPInside) && (distanceToEdge > cmpEdgeProjectionInside.getDoubleValue()))
         {
         // Point is inside and far enough away from the edge. Don't project
            desiredCMP.changeFrame(returnFrame);
            return;
         }

         // Stay cmpEdgeProjectionDistance away from the edge if possible.
         // By possible, we mean if you were to move inside by cmpEdgeProjectionDistance,
         // Make sure you are still at least minCMPProjectionDistance from the ICP,
         // unless that would put you over the edge. Then just use the edge and hope for 
         // the best.
         cmpProjected.set(true);
         double distanceToICP = moveAwayFromEdge.distance(capturePoint);
         double distanceToMove = distanceToICP - minICPToCMPProjection.getDoubleValue();

         if (isCMPInside)
         {
            double distanceToCMP = moveAwayFromEdge.distance(desiredCMP);
            if (distanceToMove < distanceToCMP) distanceToMove = distanceToCMP;
         }
         
         if (distanceToMove < 0.0) distanceToMove = 0.0;
         if (distanceToMove > cmpEdgeProjectionInside.getDoubleValue()) distanceToMove = cmpEdgeProjectionInside.getDoubleValue();
         
         insideEdgeDirection.normalize();
         insideEdgeDirection.scale(distanceToMove);
         
         
         desiredCMP.setAndChangeFrame(moveAwayFromEdge);
         desiredCMP.add(insideEdgeDirection);
        
         if (VISUALIZE)
         {
            FramePoint2d desiredCMPInWorld = desiredCMP.changeFrameCopy(ReferenceFrame.getWorldFrame());
            projectedToPointViz.setPosition(desiredCMPInWorld.getX(), desiredCMPInWorld.getY(), 0.0);
         }
         
      desiredCMP.changeFrame(returnFrame);
   }
   public boolean getWasCMPProjected()
   {
      return cmpProjected.getBooleanValue();
   }

   
   
}
