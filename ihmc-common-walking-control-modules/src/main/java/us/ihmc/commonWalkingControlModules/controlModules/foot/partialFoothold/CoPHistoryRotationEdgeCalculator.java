package us.ihmc.commonWalkingControlModules.controlModules.foot.partialFoothold;

import us.ihmc.euclid.referenceFrame.FrameLine2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLine2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.graphicsDescription.plotting.artifact.Artifact;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactLineSegment2d;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotics.functionApproximation.OnlineLine2DLinearRegression;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFrameLine2D;
import us.ihmc.yoVariables.variable.YoFramePoint2D;

import java.awt.*;

/**
 * This class aims to calculate the edge of rotation of the foot by fitting a line to the the CoP history when the foot is rotating.
 * It likely will not produce a good estimate of the actual direction of the rotation, because the measured CoP is likely going to be similar
 * to the same location, unless the foot rotates for a while. However, it will provide a good estimate of the location of the line.
 */
public class CoPHistoryRotationEdgeCalculator implements RotationEdgeCalculator
{
   private final OnlineLine2DLinearRegression lineCalculator;
   private final FrameLine2D lineOfRotationInWorld = new FrameLine2D();
   private final YoFrameLine2D lineOfRotationInSole;

   private final EdgeVisualizer edgeVisualizer;

   public CoPHistoryRotationEdgeCalculator(RobotSide side, MovingReferenceFrame soleFrame, YoVariableRegistry parentRegistry, YoGraphicsListRegistry graphicsListRegistry)
   {
      YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName() + side.getPascalCaseName());

      lineCalculator = new OnlineLine2DLinearRegression("FootRotation", registry);
      lineOfRotationInSole = new YoFrameLine2D(side.getCamelCaseName() + "LineOfRotation", "", soleFrame, registry);

      if (graphicsListRegistry != null)
         edgeVisualizer = new EdgeVisualizer(side.getLowerCaseName() + "CoPHistory", Color.RED, registry, graphicsListRegistry);
      else
         edgeVisualizer = null;

      parentRegistry.addChild(registry);
   }

   private final FramePoint2D tempPoint = new FramePoint2D();

   public void compute(FramePoint2DReadOnly measuredCoP)
   {
      tempPoint.setMatchingFrame(measuredCoP);
      lineCalculator.update(tempPoint);
      lineOfRotationInWorld.set(ReferenceFrame.getWorldFrame(), lineCalculator.getMeanLine());
      lineOfRotationInSole.setMatchingFrame(lineOfRotationInWorld);

      if (edgeVisualizer != null)
         edgeVisualizer.updateGraphics(lineOfRotationInSole);
   }

   public void reset()
   {
      if (edgeVisualizer != null)
         edgeVisualizer.reset();

      lineCalculator.reset();
   }

   public FrameLine2DReadOnly getLineOfRotation()
   {
      return lineOfRotationInSole;
   }

}
