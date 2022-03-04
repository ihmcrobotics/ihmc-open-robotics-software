package us.ihmc.commonWalkingControlModules.capturePoint.stepAdjustment;

import us.ihmc.commonWalkingControlModules.capturePoint.optimization.ICPOptimizationParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.humanoidRobotics.footstep.SimpleFootstep;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

import java.awt.*;

public class TwoStepCaptureRegionCalculator
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final FrameConvexPolygon2D twoStepRegion = new FrameConvexPolygon2D();
   private final FrameConvexPolygon2D safeTwoStepRegion = new FrameConvexPolygon2D();

   private final YoBoolean hasTwoStepRegion = new YoBoolean("hasTwoStepRegion", registry);
   private final YoFrameConvexPolygon2D yoTwoStepRegion = new YoFrameConvexPolygon2D("twoStepCaptureRegion", ReferenceFrame.getWorldFrame(), 30, registry);
   private final YoFrameConvexPolygon2D yoSafetyBiasedTwoStepRegion = new YoFrameConvexPolygon2D("safetyBiasedTwoStepCaptureRegion", ReferenceFrame.getWorldFrame(), 30, registry);

   public TwoStepCaptureRegionCalculator(YoRegistry parentRegistry,
                                         YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      if (yoGraphicsListRegistry != null)
      {
         YoArtifactPolygon polygonArtifact = new YoArtifactPolygon("Two Step Capture Region", yoTwoStepRegion, Color.GREEN, false);
         YoArtifactPolygon safePolygonArtifact = new YoArtifactPolygon("Safety Biased Two Step Capture Region", yoSafetyBiasedTwoStepRegion, Color.GREEN, false, true);
         yoGraphicsListRegistry.registerArtifact(getClass().getSimpleName(), polygonArtifact);
         yoGraphicsListRegistry.registerArtifact(getClass().getSimpleName(), safePolygonArtifact);
      }

      parentRegistry.addChild(registry);
   }

   private final FramePoint2D inversePoint = new FramePoint2D();
   private final FramePoint2D inverseGoal = new FramePoint2D();
   private final FramePoint2D stepPosition = new FramePoint2D();
   private final FramePoint2D touchdownPoint = new FramePoint2D();

   public void reset()
   {
      hasTwoStepRegion.set(false);
      yoTwoStepRegion.clear();
      yoSafetyBiasedTwoStepRegion.clear();
   }

   public void computeFromStepGoal(double minSwingAndTransferTime, SimpleFootstep stepGoal, FramePoint2DReadOnly stancePosition,
                                   double omega, FrameConvexPolygon2DReadOnly oneStepCaptureRegion)
   {
      hasTwoStepRegion.set(true);
      twoStepRegion.clear(oneStepCaptureRegion.getReferenceFrame());
      safeTwoStepRegion.clear(oneStepCaptureRegion.getReferenceFrame());
//      twoStepRegion.setMatchingFrame(oneStepCaptureRegion, false);
//      twoStepRegion.changeFrame(ReferenceFrame.getWorldFrame());

      stepPosition.setIncludingFrame(stepGoal.getSoleFramePose().getPosition());
      stepPosition.changeFrameAndProjectToXYPlane(oneStepCaptureRegion.getReferenceFrame());

      computeVisibiltyOfVerticesFromGoal(oneStepCaptureRegion, stepPosition);
      computeMiddleOfTheCaptureRegionLine(stancePosition, oneStepCaptureRegion);
      projectVisibibleVerticesTowardsTheMiddle(midRegionLine, 0.05);

      double minExponential = Math.exp(-omega * minSwingAndTransferTime);
      double radical = 1.0 / (1.0 - minExponential);

      inverseGoal.setIncludingFrame(stepPosition);
      inverseGoal.scale(minExponential);

      for (int i = 1; i < visibleVertices.size() - 1; i++)
      {
         twoStepRegion.addVertex(visibleVertices.get(i));
      }
      for (int i = 0; i < saferVisibleVertices.size(); i++)
      {
         safeTwoStepRegion.addVertex(saferVisibleVertices.get(i));
      }

      inversePoint.setReferenceFrame(oneStepCaptureRegion.getReferenceFrame());

      for (int i = 0; i < invisibleVertices.size(); i++)
      {
         touchdownPoint.setIncludingFrame(invisibleVertices.get(i));
         inversePoint.sub(touchdownPoint, inverseGoal);
         inversePoint.scale(radical);

         twoStepRegion.addVertex(inversePoint);
         safeTwoStepRegion.addVertex(inversePoint);
      }
      twoStepRegion.update();
      twoStepRegion.changeFrameAndProjectToXYPlane(ReferenceFrame.getWorldFrame());

      safeTwoStepRegion.update();
      safeTwoStepRegion.changeFrameAndProjectToXYPlane(ReferenceFrame.getWorldFrame());

      yoTwoStepRegion.set(twoStepRegion);
      yoSafetyBiasedTwoStepRegion.set(safeTwoStepRegion);
   }

   public FrameConvexPolygon2DReadOnly getCaptureRegion()
   {
      return yoTwoStepRegion;
   }

   public FrameConvexPolygon2DReadOnly getCaptureRegionWithSafetyMargin()
   {
      return yoSafetyBiasedTwoStepRegion;
   }

   public boolean hasTwoStepRegion()
   {
      return hasTwoStepRegion.getBooleanValue();
   }

   private final RecyclingArrayList<FramePoint2D> visibleVertices = new RecyclingArrayList<>(FramePoint2D::new);
   private final RecyclingArrayList<FramePoint2D> invisibleVertices = new RecyclingArrayList<>(FramePoint2D::new);

   private boolean computeVisibiltyOfVerticesFromGoal(FrameConvexPolygon2DReadOnly oneStepCaptureRegion, FramePoint2DReadOnly stepPosition)
   {
      visibleVertices.clear();
      invisibleVertices.clear();

      oneStepCaptureRegion.checkReferenceFrameMatch(stepPosition);
      int lineOfSightStartIndex = oneStepCaptureRegion.lineOfSightStartIndex(stepPosition);
      int lineOfSightEndIndex = oneStepCaptureRegion.lineOfSightEndIndex(stepPosition);
      if (lineOfSightStartIndex == -1 || lineOfSightEndIndex == -1)
         return false;

      int index = lineOfSightEndIndex;


      while (true)
      {
         visibleVertices.add().setIncludingFrame(oneStepCaptureRegion.getVertex(index));
         index = oneStepCaptureRegion.getPreviousVertexIndex(index);
         if (index == lineOfSightStartIndex)
         {
            visibleVertices.add().setIncludingFrame(oneStepCaptureRegion.getVertex(index));
            break;
         }
      }

      while (true)
      {
         invisibleVertices.add().setIncludingFrame(oneStepCaptureRegion.getVertex(index));
         index = oneStepCaptureRegion.getPreviousVertexIndex(index);

         if (index == lineOfSightEndIndex)
         {
            invisibleVertices.add().setIncludingFrame(oneStepCaptureRegion.getVertex(index));
            break;
         }
      }

      return true;
   }

   private final FrameLineSegment2D midRegionLine = new FrameLineSegment2D();

   private final FramePoint2D stanceFootCenterInStanceFrame = new FramePoint2D();
   private final FramePoint2D closestCapturePointToStance = new FramePoint2D();

   private final FrameLine2D lineOfMinimalAction = new FrameLine2D();

   private void computeMiddleOfTheCaptureRegionLine(FramePoint2DReadOnly stanceFoot, FrameConvexPolygon2DReadOnly captureRegion)
   {
      stanceFootCenterInStanceFrame.setIncludingFrame(stanceFoot);
      stanceFootCenterInStanceFrame.changeFrame(captureRegion.getReferenceFrame());
      closestCapturePointToStance.setReferenceFrame(captureRegion.getReferenceFrame());
      midRegionLine.setReferenceFrame(captureRegion.getReferenceFrame());

      captureRegion.orthogonalProjection(stanceFootCenterInStanceFrame, closestCapturePointToStance);

      lineOfMinimalAction.setIncludingFrame(stanceFootCenterInStanceFrame, closestCapturePointToStance);

      captureRegion.intersectionWith(lineOfMinimalAction, midRegionLine.getFirstEndpoint(), midRegionLine.getSecondEndpoint());
   }

   private final FramePoint2D projectedPoint = new FramePoint2D();
   private final RecyclingArrayList<FramePoint2D> saferVisibleVertices = new RecyclingArrayList<>(FramePoint2D::new);

   private void projectVisibibleVerticesTowardsTheMiddle(FrameLineSegment2D midRegionLine, double maxDistanceToProject)
   {
      saferVisibleVertices.clear();
      projectedPoint.setReferenceFrame(midRegionLine.getReferenceFrame());

      for (int i = 1; i < visibleVertices.size() - 1; i++)
      {
         midRegionLine.orthogonalProjection(visibleVertices.get(i), projectedPoint);

         FramePoint2DBasics safeVertex = saferVisibleVertices.add();

         safeVertex.setReferenceFrame(midRegionLine.getReferenceFrame());

         double projectionDistance = visibleVertices.get(i).distance(projectedPoint);
         if (projectionDistance < maxDistanceToProject)
            safeVertex.set(projectedPoint);
         else
            safeVertex.interpolate(visibleVertices.get(i), projectedPoint, maxDistanceToProject / projectionDistance);

      }
   }

}
