package us.ihmc.commonWalkingControlModules.capturePoint.stepAdjustment;

import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactLine2d;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.humanoidRobotics.footstep.SimpleFootstep;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameLine2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint2D;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

import java.awt.*;
import java.util.ArrayList;
import java.util.List;

public class TwoStepCaptureRegionCalculator
{
   private static final boolean visualizeExtra = false;
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final FrameConvexPolygon2D twoStepRegion = new FrameConvexPolygon2D();
   private final FrameConvexPolygon2D safeTwoStepRegion = new FrameConvexPolygon2D();

   private final YoBoolean hasTwoStepRegion = new YoBoolean("hasTwoStepRegion", registry);
   private final YoFrameLine2D yoLineOfMinimalAction = new YoFrameLine2D("yoLineOfMinimalAction", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameConvexPolygon2D yoTwoStepRegion = new YoFrameConvexPolygon2D("twoStepCaptureRegion", ReferenceFrame.getWorldFrame(), 30, registry);
   private final YoFrameConvexPolygon2D yoSafetyBiasedTwoStepRegion = new YoFrameConvexPolygon2D("safetyBiasedTwoStepCaptureRegion",
                                                                                                 ReferenceFrame.getWorldFrame(),
                                                                                                 30,
                                                                                                 registry);

   private final DoubleParameter distanceToProjectIntoCaptureRegion = new DoubleParameter("distanceToProjectIntoCaptureRegion", registry, 0.08);
   private final DoubleParameter extraDistanceToStepFromStanceFoot = new DoubleParameter("extraDistanceToStepFromStanceFoot", registry, 0.05);

   private final List<YoFramePoint2D> yoSaferVisibleVertices = new ArrayList<>();
   private final List<YoFramePoint2D> yoVisibleVertices = new ArrayList<>();
   private final List<YoFramePoint2D> yoInvisibleVertices = new ArrayList<>();

   public TwoStepCaptureRegionCalculator(YoRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {

      if (yoGraphicsListRegistry != null)
      {
         YoArtifactPolygon polygonArtifact = new YoArtifactPolygon("Two Step Capture Region", yoTwoStepRegion, Color.GREEN, false);
         YoArtifactPolygon safePolygonArtifact = new YoArtifactPolygon("Safety Biased Two Step Capture Region",
                                                                       yoSafetyBiasedTwoStepRegion,
                                                                       Color.GREEN,
                                                                       false,
                                                                       true);
         yoGraphicsListRegistry.registerArtifact(getClass().getSimpleName(), polygonArtifact);
         yoGraphicsListRegistry.registerArtifact(getClass().getSimpleName(), safePolygonArtifact);

         if (visualizeExtra)
         {
            YoArtifactLine2d lineOfMinimalAction = new YoArtifactLine2d("Line of Minimal Action", yoLineOfMinimalAction, Color.RED);
            yoGraphicsListRegistry.registerArtifact(getClass().getSimpleName(), lineOfMinimalAction);

            for (int i = 0; i < 10; i++)
            {
               YoFramePoint2D invisiblePoint = new YoFramePoint2D("invisibleVertex" + i, ReferenceFrame.getWorldFrame(), registry);
               invisiblePoint.setToNaN();
               YoFramePoint2D point = new YoFramePoint2D("visibleVertex" + i, ReferenceFrame.getWorldFrame(), registry);
               point.setToNaN();
               YoFramePoint2D saferPoint = new YoFramePoint2D("saferVisibleVertex" + i, ReferenceFrame.getWorldFrame(), registry);
               saferPoint.setToNaN();
               yoVisibleVertices.add(point);
               yoInvisibleVertices.add(invisiblePoint);
               yoSaferVisibleVertices.add(saferPoint);
               YoGraphicPosition invisibleVertex = new YoGraphicPosition("Invisible Vertex " + i,
                                                                         yoInvisibleVertices.get(i),
                                                                         0.005,
                                                                         YoAppearance.Blue(),
                                                                         YoGraphicPosition.GraphicType.BALL);
               yoGraphicsListRegistry.registerArtifact(getClass().getSimpleName(), invisibleVertex.createArtifact());
               YoGraphicPosition visibleVertex = new YoGraphicPosition("Visible Vertex " + i,
                                                                       yoVisibleVertices.get(i),
                                                                       0.007,
                                                                       YoAppearance.Green(),
                                                                       YoGraphicPosition.GraphicType.BALL);
               yoGraphicsListRegistry.registerArtifact(getClass().getSimpleName(), visibleVertex.createArtifact());
               YoGraphicPosition saferVertex = new YoGraphicPosition("Safer Visibile Vertex " + i,
                                                                     yoSaferVisibleVertices.get(i),
                                                                     0.005,
                                                                     YoAppearance.Teal(),
                                                                     YoGraphicPosition.GraphicType.SOLID_BALL);
               yoGraphicsListRegistry.registerArtifact(getClass().getSimpleName(), saferVertex.createArtifact());
            }
         }
      }

      parentRegistry.addChild(registry);
   }

   private final FramePoint2D inversePoint = new FramePoint2D();
   private final FramePoint2D inverseGoal = new FramePoint2D();
   private final FramePoint2D stepPositionAfterCurrentStep = new FramePoint2D();
   private final FramePoint2D touchdownPoint = new FramePoint2D();

   public void reset()
   {
      hasTwoStepRegion.set(false);
      yoTwoStepRegion.clear();
      yoSafetyBiasedTwoStepRegion.clear();
      yoLineOfMinimalAction.setToNaN();
      for (int i = 0; i < yoVisibleVertices.size(); i++)
      {
         yoVisibleVertices.get(i).setToNaN();
         yoInvisibleVertices.get(i).setToNaN();
         yoSaferVisibleVertices.get(i).setToNaN();
      }
   }

   public void computeFromStepGoal(double minSwingAndTransferTime,
                                   SimpleFootstep stepGoalAfterCurrentStep,
                                   FramePoint2DReadOnly currentICP,
                                   FramePoint2DReadOnly stancePosition,
                                   double omega,
                                   FrameConvexPolygon2DReadOnly oneStepCaptureRegion)
   {
      hasTwoStepRegion.set(true);
      twoStepRegion.clear(oneStepCaptureRegion.getReferenceFrame());
      safeTwoStepRegion.clear(oneStepCaptureRegion.getReferenceFrame());

      stepPositionAfterCurrentStep.setIncludingFrame(stepGoalAfterCurrentStep.getSoleFramePose().getPosition());
      stepPositionAfterCurrentStep.changeFrameAndProjectToXYPlane(oneStepCaptureRegion.getReferenceFrame());

      if (!computeVisibiltyOfVerticesFromGoal(oneStepCaptureRegion, stepPositionAfterCurrentStep))
      {
         for (int i = 0; i < oneStepCaptureRegion.getNumberOfVertices(); i++)
            verticesVisibleFromGoal.add().setIncludingFrame(oneStepCaptureRegion.getVertex(i));
      }
      lineOfMinimalAction.setIncludingFrame(stancePosition, currentICP);
      yoLineOfMinimalAction.setMatchingFrame(lineOfMinimalAction);

      lineOfMinimalAction.changeFrame(oneStepCaptureRegion.getReferenceFrame());
      projectVerticesFacingTheGoalTowardsTheMiddle();

      double minExponential = Math.exp(-omega * minSwingAndTransferTime);
      double radical = 1.0 / (1.0 - minExponential);

      inverseGoal.setIncludingFrame(stepPositionAfterCurrentStep);
      inverseGoal.scale(minExponential);

      for (int i = 0; i < verticesVisibleFromGoal.size(); i++)
      {
         twoStepRegion.addVertex(verticesVisibleFromGoal.get(i));
      }
      for (int i = 0; i < saferVerticesVisibleFromGoal.size(); i++)
      {
         safeTwoStepRegion.addVertex(saferVerticesVisibleFromGoal.get(i));
      }

      inversePoint.setReferenceFrame(oneStepCaptureRegion.getReferenceFrame());

      for (int i = 0; i < verticesInvisibleFromGoal.size(); i++)
      {
         touchdownPoint.setIncludingFrame(verticesInvisibleFromGoal.get(i));
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

   private final RecyclingArrayList<FramePoint2D> verticesVisibleFromGoal = new RecyclingArrayList<>(FramePoint2D::new);
   private final RecyclingArrayList<FramePoint2D> verticesInvisibleFromGoal = new RecyclingArrayList<>(FramePoint2D::new);

   private boolean computeVisibiltyOfVerticesFromGoal(FrameConvexPolygon2DReadOnly oneStepCaptureRegion, FramePoint2DReadOnly stepPosition)
   {
      verticesVisibleFromGoal.clear();
      verticesInvisibleFromGoal.clear();

      oneStepCaptureRegion.checkReferenceFrameMatch(stepPosition);
      int lineOfSightStartIndex = oneStepCaptureRegion.lineOfSightStartIndex(stepPosition);
      int lineOfSightEndIndex = oneStepCaptureRegion.lineOfSightEndIndex(stepPosition);
      if (lineOfSightStartIndex == -1 || lineOfSightEndIndex == -1)
         return false;

      int index = lineOfSightEndIndex;

      while (true)
      {
         verticesVisibleFromGoal.add().setIncludingFrame(oneStepCaptureRegion.getVertex(index));
         index = oneStepCaptureRegion.getPreviousVertexIndex(index);
         if (index == lineOfSightStartIndex)
         {
            verticesVisibleFromGoal.add().setIncludingFrame(oneStepCaptureRegion.getVertex(index));
            break;
         }
      }

      while (true)
      {
         verticesInvisibleFromGoal.add().setIncludingFrame(oneStepCaptureRegion.getVertex(index));
         index = oneStepCaptureRegion.getPreviousVertexIndex(index);

         if (index == lineOfSightEndIndex)
         {
            verticesInvisibleFromGoal.add().setIncludingFrame(oneStepCaptureRegion.getVertex(index));
            break;
         }
      }

      int i = 0;
      for (; i < Math.min(verticesVisibleFromGoal.size(), yoVisibleVertices.size()); i++)
         yoVisibleVertices.get(i).setMatchingFrame(verticesVisibleFromGoal.get(i));
      for (; i < yoVisibleVertices.size(); i++)
         yoVisibleVertices.get(i).setToNaN();
      i = 0;
      for (; i < Math.min(verticesInvisibleFromGoal.size(), yoInvisibleVertices.size()); i++)
         yoInvisibleVertices.get(i).setMatchingFrame(verticesInvisibleFromGoal.get(i));
      for (; i < yoVisibleVertices.size(); i++)
         yoInvisibleVertices.get(i).setToNaN();

      return true;
   }

   private final FrameLine2D lineOfMinimalAction = new FrameLine2D();

   private final FramePoint2D projectedPoint = new FramePoint2D();
   private final RecyclingArrayList<FramePoint2D> saferVerticesVisibleFromGoal = new RecyclingArrayList<>(FramePoint2D::new);

   private void projectVerticesFacingTheGoalTowardsTheMiddle()
   {
      saferVerticesVisibleFromGoal.clear();
      projectedPoint.setReferenceFrame(lineOfMinimalAction.getReferenceFrame());

      boolean goalIsOnTheLeft = lineOfMinimalAction.isPointOnLeftSideOfLine(stepPositionAfterCurrentStep);

      for (int i = 0; i < verticesVisibleFromGoal.size(); i++)
      {
         if (goalIsOnTheLeft != lineOfMinimalAction.isPointOnLeftSideOfLine(verticesVisibleFromGoal.get(i)))
         {
            saferVerticesVisibleFromGoal.add().setIncludingFrame(verticesVisibleFromGoal.get(i));
            continue;
         }

         lineOfMinimalAction.orthogonalProjection(verticesVisibleFromGoal.get(i), projectedPoint);

         FramePoint2DBasics safeVertex = saferVerticesVisibleFromGoal.add();

         safeVertex.setReferenceFrame(lineOfMinimalAction.getReferenceFrame());

         double projectionDistance = verticesVisibleFromGoal.get(i).distance(projectedPoint);
         if (projectionDistance < distanceToProjectIntoCaptureRegion.getValue())
            safeVertex.set(projectedPoint);
         else
            safeVertex.interpolate(verticesVisibleFromGoal.get(i), projectedPoint, distanceToProjectIntoCaptureRegion.getValue() / projectionDistance);
      }

      for (int i = 0; i < verticesInvisibleFromGoal.size(); i++)
      {
         if (goalIsOnTheLeft != lineOfMinimalAction.isPointOnLeftSideOfLine(verticesInvisibleFromGoal.get(i)))
            continue;

         lineOfMinimalAction.orthogonalProjection(verticesInvisibleFromGoal.get(i), projectedPoint);

         double projectionDistance = verticesInvisibleFromGoal.get(i).distance(projectedPoint);
         if (projectionDistance < distanceToProjectIntoCaptureRegion.getValue())
            verticesInvisibleFromGoal.get(i).set(projectedPoint);
         else
            verticesInvisibleFromGoal.get(i).interpolate(projectedPoint, distanceToProjectIntoCaptureRegion.getValue() / projectionDistance);
      }

      int i = 0;
      for (; i < Math.min(saferVerticesVisibleFromGoal.size(), yoSaferVisibleVertices.size()); i++)
         yoSaferVisibleVertices.get(i).setMatchingFrame(saferVerticesVisibleFromGoal.get(i));
      for (; i < yoSaferVisibleVertices.size(); i++)
         yoSaferVisibleVertices.get(i).setToNaN();
   }
}
