package us.ihmc.commonWalkingControlModules.capturePoint.stepAdjustment;

import java.awt.Color;
import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.configurations.SteppingParameters;
import us.ihmc.commons.InterpolationTools;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.robotics.SCS2YoGraphicHolder;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint2D;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoInteger;

public class StepAdjustmentReachabilityConstraint implements SCS2YoGraphicHolder
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private static final int numberOfVertices = 9;
   private static final int numberOfCrossOverVertices = 8;

   private final SideDependentList<List<YoFramePoint2D>> reachabilityVertices = new SideDependentList<>();
   private final SideDependentList<YoFrameConvexPolygon2D> reachabilityPolygons = new SideDependentList<>();
   private final SideDependentList<YoFrameConvexPolygon2D> forwardReachabilityPolygons = new SideDependentList<>();
   private final SideDependentList<YoFrameConvexPolygon2D> backwardReachabilityPolygons = new SideDependentList<>();
   private final SideDependentList<FrameConvexPolygon2D> totalReachabilityHulls = new SideDependentList<>();

   private final YoFrameConvexPolygon2D reachabilityPolygon;
   private final YoFrameConvexPolygon2D forwardCrossOverReachability;
   private final YoFrameConvexPolygon2D backwardCrossOverReachability;

   private final DoubleProvider forwardCrossOverDistance;
   private final DoubleProvider forwardCrossOverClearanceAngle;
   private final DoubleProvider backwardCrossOverDistance;
   private final DoubleProvider backwardCrossOverClearanceAngle;

   private final DoubleProvider lengthLimit;
   private final DoubleProvider lengthBackLimit;
   private final DoubleProvider innerLimit;
   private final DoubleProvider outerLimit;
   private final DoubleProvider inPlaceWidth;

   public StepAdjustmentReachabilityConstraint(SideDependentList<? extends ReferenceFrame> soleZUpFrames,
                                               SteppingParameters steppingParameters,
                                               StepAdjustmentParameters.CrossOverReachabilityParameters crossOverParameters,
                                               String yoNamePrefix,
                                               boolean visualize,
                                               YoRegistry registry,
                                               YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this(soleZUpFrames,
           new DoubleParameter(yoNamePrefix + "MaxReachabilityLength", registry, steppingParameters.getMaxStepLength()),
           new DoubleParameter(yoNamePrefix + "MaxReachabilityBackwardLength", registry, steppingParameters.getMaxBackwardStepLength()),
           new DoubleParameter(yoNamePrefix + "MinReachabilityWidth", registry, steppingParameters.getMinStepWidth()),
           new DoubleParameter(yoNamePrefix + "MaxReachabilityWidth", registry, steppingParameters.getMaxStepWidth()),
           new DoubleParameter(yoNamePrefix + "InPlaceWidth", registry, steppingParameters.getInPlaceWidth()),
           crossOverParameters,
           yoNamePrefix,
           visualize,
           registry,
           yoGraphicsListRegistry);
   }

   public StepAdjustmentReachabilityConstraint(SideDependentList<? extends ReferenceFrame> soleZUpFrames,
                                               DoubleProvider lengthLimit,
                                               DoubleProvider lengthBackLimit,
                                               DoubleProvider innerLimit,
                                               DoubleProvider outerLimit,
                                               DoubleProvider inPlaceWidth,
                                               StepAdjustmentParameters.CrossOverReachabilityParameters crossOverParameters,
                                               String yoNamePrefix,
                                               boolean visualize,
                                               YoRegistry registry,
                                               YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.lengthLimit = lengthLimit;
      this.lengthBackLimit = lengthBackLimit;
      this.innerLimit = innerLimit;
      this.outerLimit = outerLimit;
      this.inPlaceWidth = inPlaceWidth;

      this.forwardCrossOverDistance = new DoubleParameter("forwardCrossOverDistance", registry, crossOverParameters.getForwardCrossOverDistance());
      this.backwardCrossOverDistance = new DoubleParameter("backwardCrossOverDistance", registry, crossOverParameters.getBackwardCrossOverDistance());
      this.forwardCrossOverClearanceAngle = new DoubleParameter("forwardCrossOverClearanceAngle",
                                                                registry,
                                                                crossOverParameters.getForwardCrossOverClearanceAngle());
      this.backwardCrossOverClearanceAngle = new DoubleParameter("backwardCrossOverClearanceAngle",
                                                                 registry,
                                                                 crossOverParameters.getBackwardCrossOverClearanceAngle());


      for (RobotSide robotSide : RobotSide.values)
      {

         ReferenceFrame supportSoleFrame = soleZUpFrames.get(robotSide);

         YoInteger yoNumberOfReachabilityVertices = new YoInteger(robotSide.getLowerCaseName() + "NumberOfReachabilityVertices", registry);
         yoNumberOfReachabilityVertices.set(numberOfVertices);

         String prefix = yoNamePrefix + robotSide.getSideNameFirstLetter();

         List<YoFramePoint2D> reachabilityVertices = new ArrayList<>();
         for (int i = 0; i < yoNumberOfReachabilityVertices.getValue(); i++)
         {
            YoFramePoint2D vertex = new YoFramePoint2D(prefix + "ReachabilityVertex" + i, supportSoleFrame, registry);
            reachabilityVertices.add(vertex);
         }
         YoFrameConvexPolygon2D reachabilityPolygon = new YoFrameConvexPolygon2D(reachabilityVertices, yoNumberOfReachabilityVertices, supportSoleFrame);

         this.reachabilityVertices.put(robotSide, reachabilityVertices);
         this.reachabilityPolygons.put(robotSide, reachabilityPolygon);

         YoFrameConvexPolygon2D forwardReachabilityPolygon = new YoFrameConvexPolygon2D(prefix + "ForwardReachability", supportSoleFrame, numberOfCrossOverVertices, registry);
         YoFrameConvexPolygon2D backwardReachabilityPolygon = new YoFrameConvexPolygon2D(prefix + "BackwardReachability", supportSoleFrame, numberOfCrossOverVertices, registry);
         forwardReachabilityPolygons.put(robotSide, forwardReachabilityPolygon);
         backwardReachabilityPolygons.put(robotSide, backwardReachabilityPolygon);
         totalReachabilityHulls.put(robotSide, new FrameConvexPolygon2D(supportSoleFrame));
      }

      reachabilityPolygon = new YoFrameConvexPolygon2D(yoNamePrefix + "ReachabilityRegion", "", worldFrame, numberOfVertices, registry);
      forwardCrossOverReachability = new YoFrameConvexPolygon2D(yoNamePrefix + "ForwardReachabilityRegion", "", worldFrame, numberOfCrossOverVertices, registry);
      backwardCrossOverReachability = new YoFrameConvexPolygon2D(yoNamePrefix + "BackwardReachabilityRegion", "", worldFrame, numberOfCrossOverVertices, registry);

      if (yoGraphicsListRegistry != null)
      {
         YoArtifactPolygon reachabilityGraphic = new YoArtifactPolygon("ReachabilityRegionViz", reachabilityPolygon, Color.BLUE, false);
         YoArtifactPolygon forwardReachabilityGraphic = new YoArtifactPolygon("ForwardReachabilityRegionViz", forwardCrossOverReachability, Color.BLUE, false);
         YoArtifactPolygon backwardReachabilityGraphic = new YoArtifactPolygon("BackwardReachabilityRegionViz", backwardCrossOverReachability, Color.BLUE, false);

         reachabilityGraphic.setVisible(visualize);
         forwardReachabilityGraphic.setVisible(visualize);
         backwardReachabilityGraphic.setVisible(visualize);

         yoGraphicsListRegistry.registerArtifact(getClass().getSimpleName(), reachabilityGraphic);
         yoGraphicsListRegistry.registerArtifact(getClass().getSimpleName(), forwardReachabilityGraphic);
         yoGraphicsListRegistry.registerArtifact(getClass().getSimpleName(), backwardReachabilityGraphic);
      }
   }

   /**
    * Initializes the reachability constraint for the double support state.
    * This is the constraint that determines where the robot can step to kinematically.
    * It is a simple convex rectangle determined by the stepping parameters.
    */
   public void reset()
   {
      reachabilityPolygon.clear();
      forwardCrossOverReachability.clear();
      backwardCrossOverReachability.clear();
   }

   /**
    * Initializes the reachability constraint for the single support state.
    * This is the constraint that determines where the robot can step to kinematically.
    * It is a simple convex rectangle determined by the stepping parameters.
    *
    * @param  supportSide the current support side of the robot
    */
   public FrameConvexPolygon2DReadOnly initializeReachabilityConstraint(RobotSide supportSide)
   {
      reachabilityPolygon.setMatchingFrame(updateReachabilityPolygon(supportSide), false);
      forwardCrossOverReachability.setMatchingFrame(updateForwardCrossOverPolygon(supportSide), false);
      backwardCrossOverReachability.setMatchingFrame(updateBackwardCrossOverPolygon(supportSide), false);
      updateTotalReachability(supportSide);
      
      updateReachabilityPolygon(supportSide.getOppositeSide());
      updateForwardCrossOverPolygon(supportSide.getOppositeSide());
      updateBackwardCrossOverPolygon(supportSide.getOppositeSide());
      updateTotalReachability(supportSide.getOppositeSide());

      return reachabilityPolygon;
   }

   FrameConvexPolygon2D tempPolygon = new FrameConvexPolygon2D();

   private FrameConvexPolygon2DReadOnly updateReachabilityPolygon(RobotSide supportSide)
   {
      List<YoFramePoint2D> vertices = reachabilityVertices.get(supportSide);
      YoFrameConvexPolygon2D polygon = reachabilityPolygons.get(supportSide);

      tempPolygon.clear(polygon.getReferenceFrame());

      // create an ellipsoid around the center of the forward and backward reachable limits
      double innerRadius = inPlaceWidth.getValue() - innerLimit.getValue();
      double outerRadius = outerLimit.getValue() - inPlaceWidth.getValue();

      // compute the vertices on the edge of the ellipsoid
      for (int vertexIdx = 0; vertexIdx < vertices.size(); vertexIdx++)
      {
         double angle = 2.0 * Math.PI * vertexIdx / (vertices.size() - 1);
         double x, y;
         if (angle < Math.PI / 2.0)
         {
            x = lengthLimit.getValue() * Math.cos(angle);
            y = supportSide.negateIfLeftSide(inPlaceWidth.getValue() - innerRadius * Math.sin(angle));
         }
         else if (angle < Math.PI)
         {
            x = lengthBackLimit.getValue() * Math.cos(angle);
            y = supportSide.negateIfLeftSide(inPlaceWidth.getValue() - innerRadius * Math.sin(angle));
         }
         else if (angle < 1.5 * Math.PI)
         {
            x = lengthBackLimit.getValue() * Math.cos(angle);
            y = supportSide.negateIfLeftSide(inPlaceWidth.getValue() - outerRadius * Math.sin(angle));
         }
         else
         {
            x = lengthLimit.getValue() * Math.cos(angle);
            y = supportSide.negateIfLeftSide(inPlaceWidth.getValue() - outerRadius * Math.sin(angle));
         }

         tempPolygon.addVertex(x, y);
      }

      tempPolygon.update();
      polygon.set(tempPolygon);

      return polygon;
   }

   private FrameConvexPolygon2DReadOnly updateForwardCrossOverPolygon(RobotSide supportSide)
   {
      YoFrameConvexPolygon2D forwardPolygon = forwardReachabilityPolygons.get(supportSide);

      double forwardInnerRadius = (inPlaceWidth.getValue() + forwardCrossOverDistance.getValue()) / Math.cos(forwardCrossOverClearanceAngle.getValue());
      double outerRadius = outerLimit.getValue() - inPlaceWidth.getValue();

      forwardPolygon.clear();

      forwardPolygon.addVertex(0.0, supportSide.negateIfLeftSide(inPlaceWidth.getValue()));
      for (int vertexIdx = 0; vertexIdx < forwardPolygon.getMaxNumberOfVertices() - 1; vertexIdx++)
      {
         double alpha = (double) vertexIdx / (forwardPolygon.getMaxNumberOfVertices() - 2);
         double angle = InterpolationTools.linearInterpolate(-Math.PI / 2.0 + forwardCrossOverClearanceAngle.getValue(), Math.PI / 2.0, alpha);
         double x, y;
         if (angle < 0)
         {
            x = lengthLimit.getValue() * Math.cos(angle);
            y = supportSide.negateIfLeftSide(inPlaceWidth.getValue() + forwardInnerRadius * Math.sin(angle));
         }
         else
         {
            x = lengthLimit.getValue() * Math.cos(angle);
            y = supportSide.negateIfLeftSide(inPlaceWidth.getValue() + outerRadius * Math.sin(angle));
         }

         forwardPolygon.addVertex(x, y);
      }

      forwardPolygon.update();

      return forwardPolygon;
   }

   private FrameConvexPolygon2DReadOnly updateBackwardCrossOverPolygon(RobotSide supportSide)
   {
      YoFrameConvexPolygon2D backwardPolygon = backwardReachabilityPolygons.get(supportSide);

      double backwardInnerRadius = (inPlaceWidth.getValue() + backwardCrossOverDistance.getValue()) / Math.cos(backwardCrossOverClearanceAngle.getValue());
      double outerRadius = outerLimit.getValue() - inPlaceWidth.getValue();

      backwardPolygon.clear();

      backwardPolygon.addVertex(0.0, supportSide.negateIfLeftSide(inPlaceWidth.getValue()));


      for (int vertexIdx = 0; vertexIdx < backwardPolygon.getMaxNumberOfVertices() - 1; vertexIdx++)
      {
         double alpha = (double) vertexIdx / (backwardPolygon.getMaxNumberOfVertices() - 2);
         double angle = InterpolationTools.linearInterpolate(-Math.PI / 2.0 + backwardCrossOverClearanceAngle.getValue(), Math.PI / 2.0, alpha);
         double x, y;
         if (angle < 0)
         {
            x = -lengthBackLimit.getValue() * Math.cos(angle);
            y = supportSide.negateIfLeftSide(inPlaceWidth.getValue() + backwardInnerRadius * Math.sin(angle));
         }
         else
         {
            x = -lengthBackLimit.getValue() * Math.cos(angle);
            y = supportSide.negateIfLeftSide(inPlaceWidth.getValue() + outerRadius * Math.sin(angle));
         }

         backwardPolygon.addVertex(x, y);
      }

      backwardPolygon.update();

      return backwardPolygon;

   }

   private void updateTotalReachability(RobotSide supportSide)
   {
      FrameConvexPolygon2D totalReachabilityHull = totalReachabilityHulls.get(supportSide);
      totalReachabilityHull.clear();
      totalReachabilityHull.addVertices(reachabilityPolygons.get(supportSide));
      totalReachabilityHull.addVertices(forwardReachabilityPolygons.get(supportSide));
      totalReachabilityHull.addVertices(backwardReachabilityPolygons.get(supportSide));
      totalReachabilityHull.update();
   }

   /**
    * Get the polygon that describes the reachable region for the step position.
    */
   public FrameConvexPolygon2DReadOnly getReachabilityConstraint()
   {
      return reachabilityPolygon;
   }

   public FrameConvexPolygon2DReadOnly getForwardCrossOverPolygon()
   {
      return forwardCrossOverReachability;
   }

   public FrameConvexPolygon2DReadOnly getBackwardCrossOverPolygon()
   {
      return backwardCrossOverReachability;
   }

   public FrameConvexPolygon2DReadOnly getReachabilityPolygonInFootFrame(RobotSide robotSide)
   {
      return reachabilityPolygons.get(robotSide);
   }

   public FrameConvexPolygon2DReadOnly getForwardCrossOverPolygonInFootFrame(RobotSide robotSide)
   {
      return forwardReachabilityPolygons.get(robotSide);
   }

   public FrameConvexPolygon2DReadOnly getBackwardCrossOverPolygonInFootFrame(RobotSide robotSide)
   {
      return backwardReachabilityPolygons.get(robotSide);
   }

   public FrameConvexPolygon2DReadOnly getTotalReachabilityHull(RobotSide supportSide)
   {
      return totalReachabilityHulls.get(supportSide);
   }

   @Override
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(getClass().getSimpleName());
      group.addChild(YoGraphicDefinitionFactory.newYoGraphicPolygon2D("ReachabilityRegion", reachabilityPolygon, ColorDefinitions.Blue()));
      group.addChild(YoGraphicDefinitionFactory.newYoGraphicPolygon2D("ForwardReachabilityRegion", forwardCrossOverReachability, ColorDefinitions.Blue()));
      group.addChild(YoGraphicDefinitionFactory.newYoGraphicPolygon2D("BackwardReachabilityRegion", backwardCrossOverReachability, ColorDefinitions.Blue()));
      return group;
   }
}
