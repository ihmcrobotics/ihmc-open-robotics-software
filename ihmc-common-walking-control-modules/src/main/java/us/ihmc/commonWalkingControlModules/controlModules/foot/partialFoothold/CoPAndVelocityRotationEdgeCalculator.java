package us.ihmc.commonWalkingControlModules.controlModules.foot.partialFoothold;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotics.math.filters.AlphaFilteredYoFramePoint2d;
import us.ihmc.robotics.math.filters.AlphaFilteredYoFrameVector2d;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFrameLine2D;
import us.ihmc.yoVariables.variable.YoFramePoint2D;
import us.ihmc.yoVariables.variable.YoFrameVector2D;

import java.awt.*;

public class CoPAndVelocityRotationEdgeCalculator implements RotationEdgeCalculator
{
   private final RotationEdgeCalculator copHistoryEdgeCalculator;
   private final RotationEdgeCalculator velocityEdgeCalculator;

   private final YoFramePoint2D pointOfRotation;
   private final YoFrameVector2D axisOfRotation;
   private final FixedFrameLine2DBasics lineOfRotationInSole;

   private final boolean otherEdgesAreMembersOnly;

   private final EdgeVisualizer edgeVisualizer;

   public CoPAndVelocityRotationEdgeCalculator(RobotSide side, MovingReferenceFrame soleFrame, double dt, YoVariableRegistry parentRegistry,
                                               YoGraphicsListRegistry graphicsListRegistry)
   {
      this(side, soleFrame, new CoPHistoryRotationEdgeCalculator(side, soleFrame, parentRegistry, null),
           new VelocityRotationEdgeCalculator(side, soleFrame, dt, parentRegistry, null), false, graphicsListRegistry);
   }

   public CoPAndVelocityRotationEdgeCalculator(RobotSide side, ReferenceFrame soleFrame, RotationEdgeCalculator copHistoryEdgeCalculator, RotationEdgeCalculator velocityEdgeCalculator,
                                               boolean otherEdgesAreMembersOnly, YoGraphicsListRegistry graphicsListRegistry)
   {
      this.copHistoryEdgeCalculator = copHistoryEdgeCalculator;
      this.velocityEdgeCalculator = velocityEdgeCalculator;
      this.otherEdgesAreMembersOnly = otherEdgesAreMembersOnly;

      String namePrefix = side.getLowerCaseName() + "CoPAndVelocity";
      YoVariableRegistry registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());

      pointOfRotation = new YoFramePoint2D(namePrefix + "PointOfRotation", soleFrame, registry);
      axisOfRotation = new YoFrameVector2D(namePrefix + "AxisOfRotation", soleFrame, registry);

      lineOfRotationInSole = new YoFrameLine2D(pointOfRotation, axisOfRotation);

      if (graphicsListRegistry != null)
         edgeVisualizer = new EdgeVisualizer(namePrefix, Color.RED, registry, graphicsListRegistry);
      else
         edgeVisualizer = null;
   }

   @Override
   public void compute(FramePoint2DReadOnly measuredCoP)
   {
      if (!otherEdgesAreMembersOnly)
      {
         copHistoryEdgeCalculator.compute(measuredCoP);
         velocityEdgeCalculator.compute(measuredCoP);
      }

      lineOfRotationInSole.set(copHistoryEdgeCalculator.getLineOfRotation().getPoint(), velocityEdgeCalculator.getLineOfRotation().getDirection());

      if (edgeVisualizer != null)
         edgeVisualizer.updateGraphics(lineOfRotationInSole);
   }

   @Override
   public void reset()
   {
      if (!otherEdgesAreMembersOnly)
      {
         copHistoryEdgeCalculator.reset();
         velocityEdgeCalculator.reset();
      }

      lineOfRotationInSole.setToNaN();

      if (edgeVisualizer != null)
         edgeVisualizer.reset();
   }

   @Override
   public FrameLine2DReadOnly getLineOfRotation()
   {
      return lineOfRotationInSole;
   }
}
