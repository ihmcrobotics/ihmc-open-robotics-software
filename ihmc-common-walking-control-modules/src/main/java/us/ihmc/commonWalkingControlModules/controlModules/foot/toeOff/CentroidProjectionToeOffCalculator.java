package us.ihmc.commonWalkingControlModules.controlModules.foot.toeOff;

import java.awt.Color;
import java.util.List;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoContactPoint;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.configurations.ToeOffParameters;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FrameLine2D;
import us.ihmc.euclid.referenceFrame.FrameLineSegment2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLineSegment2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.plotting.artifact.Artifact;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactLineSegment2d;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory.DefaultPoint2DGraphic;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint2D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class CentroidProjectionToeOffCalculator implements ToeOffCalculator
{
   private static final boolean visualize = false;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private static final String namePrefix = "centProj";

   private final SideDependentList<List<YoContactPoint>> contactPoints = new SideDependentList<>();

   private final FramePoint2D toeOffContactPoint2d = new FramePoint2D();
   private final FramePoint2D closestPointOnRay = new FramePoint2D();
   private final FrameLineSegment2D toeOffContactLine2d = new FrameLineSegment2D();

   private final FramePoint3D exitCMP = new FramePoint3D();
   private final FramePoint2D exitCMP2d = new FramePoint2D();
   private final FrameVector2D exitCMPRayDirection2d = new FrameVector2D();

   private final FramePoint2D tmpPoint2d = new FramePoint2D();
   private final FramePoint2D tmpPoint2d2 = new FramePoint2D();
   private final FrameLine2D rayThroughExitCMP = new FrameLine2D();

   private final SideDependentList<ReferenceFrame> soleFrames = new SideDependentList<>();

   private final FrameConvexPolygon2D footPolygon = new FrameConvexPolygon2D();

   private final YoDouble toeOffContactInterpolation;
   private final YoBoolean hasComputedToeOffContactPoint;
   private final YoBoolean hasComputedToeOffContactLine;

   private final YoFramePoint2D rayOrigin;
   private final YoFramePoint2D rayEnd;

   public CentroidProjectionToeOffCalculator(SideDependentList<YoPlaneContactState> contactStates,
                                             SideDependentList<? extends ContactablePlaneBody> feet,
                                             ToeOffParameters toeOffParameters,
                                             YoRegistry parentRegistry)
   {
      this(contactStates, feet, toeOffParameters, parentRegistry, null);
   }

   public CentroidProjectionToeOffCalculator(SideDependentList<YoPlaneContactState> contactStates,
                                             SideDependentList<? extends ContactablePlaneBody> feet,
                                             ToeOffParameters toeOffParameters,
                                             YoRegistry parentRegistry,
                                             YoGraphicsListRegistry graphicsListRegistry)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         ReferenceFrame soleFrame = feet.get(robotSide).getSoleFrame();
         soleFrames.put(robotSide, soleFrame);

         contactPoints.put(robotSide, contactStates.get(robotSide).getContactPoints());
      }

      toeOffContactInterpolation = new YoDouble(namePrefix + "ToeOffContactInterpolation", registry);
      toeOffContactInterpolation.set(toeOffParameters.getToeOffContactInterpolation());

      hasComputedToeOffContactPoint = new YoBoolean(namePrefix + "HasComputedToeOffContactPoint", registry);
      hasComputedToeOffContactLine = new YoBoolean(namePrefix + "HasComputedToeOffContactLine", registry);

      parentRegistry.addChild(registry);

      if (visualize && graphicsListRegistry != null)
      {
         rayOrigin = new YoFramePoint2D("toeOffRayOrigin", worldFrame, registry);
         rayEnd = new YoFramePoint2D("toeOffRayEnd", worldFrame, registry);

         YoGraphicPosition originViz = new YoGraphicPosition("originViz", rayOrigin, 0.005, YoAppearance.Blue(), GraphicType.SOLID_BALL);
         YoGraphicPosition endViz = new YoGraphicPosition("endViz", rayEnd, 0.005, YoAppearance.Red(), GraphicType.SOLID_BALL);
         Artifact lineArtifact = new YoArtifactLineSegment2d("toeOffLine", rayOrigin, rayEnd, Color.RED, 0.005, 0.01);
         graphicsListRegistry.registerArtifact(getClass().getSimpleName(), lineArtifact);
         graphicsListRegistry.registerArtifact(getClass().getSimpleName(), originViz.createArtifact());
         graphicsListRegistry.registerArtifact(getClass().getSimpleName(), endViz.createArtifact());
      }
      else
      {
         rayOrigin = null;
         rayEnd = null;
      }
   }

   @Override
   public ToeOffEnum getEnum()
   {
      return ToeOffEnum.CENTROID_PROJECTION;
   }

   @Override
   public void clear()
   {
      exitCMP2d.setToNaN();
      hasComputedToeOffContactPoint.set(false);
      hasComputedToeOffContactLine.set(false);
   }

   @Override
   public void setExitCMP(FramePoint3DReadOnly exitCMP, RobotSide trailingLeg)
   {
      ReferenceFrame soleFrame = soleFrames.get(trailingLeg);
      this.exitCMP.setIncludingFrame(exitCMP);
      this.exitCMP.changeFrame(soleFrame);
      exitCMP2d.setToZero(soleFrame);
      exitCMP2d.setIncludingFrame(this.exitCMP);
   }

   private final FramePoint2D desiredCMP = new FramePoint2D();

   @Override
   public void computeToeOffContactPoint(FramePoint2DReadOnly desiredCMP, RobotSide trailingLeg)
   {
      ReferenceFrame soleFrame = soleFrames.get(trailingLeg);

      computeFootPolygon(trailingLeg);

      FramePoint2DReadOnly rayOrigin = footPolygon.getCentroid();
      exitCMPRayDirection2d.setIncludingFrame(soleFrame, 1.0, 0.0);

      if (!exitCMP2d.containsNaN() && footPolygon.isPointInside(exitCMP2d))
      {
         exitCMPRayDirection2d.setToZero(soleFrame);
         exitCMPRayDirection2d.sub(exitCMP2d, footPolygon.getCentroid());
      }
      else
      {
         exitCMPRayDirection2d.setIncludingFrame(soleFrame, 1.0, 0.0);
      }

      rayThroughExitCMP.setToZero(soleFrame);

      if (desiredCMP != null && !desiredCMP.containsNaN())
      {
         tmpPoint2d.setToZero(soleFrame);
         this.desiredCMP.setIncludingFrame(desiredCMP);
         this.desiredCMP.changeFrameAndProjectToXYPlane(soleFrame);
         tmpPoint2d.interpolate(rayOrigin, this.desiredCMP, toeOffContactInterpolation.getDoubleValue());

         if (footPolygon.isPointInside(tmpPoint2d))
            rayThroughExitCMP.set(tmpPoint2d, exitCMPRayDirection2d);
         else
            rayThroughExitCMP.set(rayOrigin, exitCMPRayDirection2d);
      }
      else
      {
         rayThroughExitCMP.set(rayOrigin, exitCMPRayDirection2d);
      }

      computeToeOffContactLine(desiredCMP, trailingLeg);

      if (!toeOffContactLine2d.intersectionWith(rayThroughExitCMP, toeOffContactPoint2d))
      {
         tmpPoint2d2.setToZero(soleFrame);
         closestPointOnRay.setToZero(soleFrame);
         rayThroughExitCMP.getTwoPointsOnLine(tmpPoint2d, tmpPoint2d2);
         EuclidGeometryTools.closestPoint2DsBetweenTwoLineSegment2Ds(toeOffContactLine2d.getFirstEndpoint(),
                                                                     toeOffContactLine2d.getSecondEndpoint(),
                                                                     tmpPoint2d,
                                                                     tmpPoint2d2,
                                                                     toeOffContactPoint2d,
                                                                     closestPointOnRay);
      }

      if (this.rayOrigin != null)
      {
         tmpPoint2d.setIncludingFrame(rayOrigin);
         tmpPoint2d.changeFrameAndProjectToXYPlane(worldFrame);
         this.rayOrigin.set(tmpPoint2d);
      }
      if (this.rayEnd != null)
      {
         tmpPoint2d.setIncludingFrame(toeOffContactPoint2d);
         tmpPoint2d.changeFrameAndProjectToXYPlane(worldFrame);
         this.rayEnd.set(tmpPoint2d);
      }

      hasComputedToeOffContactPoint.set(true);
   }

   @Override
   public void getToeOffContactPoint(FramePoint2DBasics contactPointToPack, RobotSide trailingLeg)
   {
      if (!hasComputedToeOffContactPoint.getBooleanValue())
         computeToeOffContactPoint(null, trailingLeg);

      contactPointToPack.setIncludingFrame(toeOffContactPoint2d);
   }

   @Override
   public void computeToeOffContactLine(FramePoint2DReadOnly desiredCMP, RobotSide trailingLeg)
   {
      computeFootPolygon(trailingLeg);

      ReferenceFrame referenceFrame = footPolygon.getReferenceFrame();
      toeOffContactLine2d.setToZero(referenceFrame);
      toeOffContactLine2d.getFirstEndpoint().set(referenceFrame, Double.NEGATIVE_INFINITY, 0.0);
      toeOffContactLine2d.getSecondEndpoint().set(referenceFrame, Double.NEGATIVE_INFINITY, 0.0);

      // gets the leading two toe points
      for (int i = 0; i < footPolygon.getNumberOfVertices(); i++)
      {
         tmpPoint2d.setIncludingFrame(footPolygon.getVertex(i));
         if (tmpPoint2d.getX() > toeOffContactLine2d.getFirstEndpoint().getX())
         { // further ahead than leading point
            toeOffContactLine2d.flipDirection();
            toeOffContactLine2d.getFirstEndpoint().set(tmpPoint2d);
         }
         else if (tmpPoint2d.getX() > toeOffContactLine2d.getSecondEndpoint().getX())
         { // further ahead than second leading point
            toeOffContactLine2d.getSecondEndpoint().set(tmpPoint2d);
         }
      }

      hasComputedToeOffContactLine.set(true);
   }

   @Override
   public void getToeOffContactLine(FrameLineSegment2DBasics contactLineToPack, RobotSide trailingLeg)
   {
      if (!hasComputedToeOffContactLine.getBooleanValue())
         computeToeOffContactLine(null, trailingLeg);

      contactLineToPack.set(toeOffContactLine2d);
   }

   private void computeFootPolygon(RobotSide trailingLeg)
   {
      ReferenceFrame soleFrame = soleFrames.get(trailingLeg);
      footPolygon.clear(soleFrame);
      for (int i = 0; i < contactPoints.get(trailingLeg).size(); i++)
      {
         footPolygon.addVertex(contactPoints.get(trailingLeg).get(i));
      }
      footPolygon.update();
   }

   @Override
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      if (!visualize)
         return null;
      if (rayOrigin == null)
         return null;
      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(getClass().getSimpleName());
      group.addChild(YoGraphicDefinitionFactory.newYoGraphicPoint2D("origin", rayOrigin, 0.01, ColorDefinitions.Blue(), DefaultPoint2DGraphic.CIRCLE_FILLED));
      group.addChild(YoGraphicDefinitionFactory.newYoGraphicPoint2D("end", rayEnd, 0.01, ColorDefinitions.Red(), DefaultPoint2DGraphic.CIRCLE_FILLED));
      group.addChild(YoGraphicDefinitionFactory.newYoGraphicLineSegment2DDefinition("toeOffList", rayOrigin, rayEnd, ColorDefinitions.Red()));
      return group;
   }
}
