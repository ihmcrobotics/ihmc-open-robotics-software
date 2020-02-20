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

public class CoPHistoryRotationEdgeCalculator implements RotationEdgeCalculator
{
   private final YoVariableRegistry registry;

   private final YoFramePoint2D linePointA;
   private final YoFramePoint2D linePointB;

   private final OnlineLine2DLinearRegression lineCalculator;
   private final FrameLine2D lineOfRotationInWorld = new FrameLine2D();
   private final YoFrameLine2D lineOfRotationInSole;

   public CoPHistoryRotationEdgeCalculator(RobotSide side, MovingReferenceFrame soleFrame, YoVariableRegistry parentRegistry, YoGraphicsListRegistry graphicsListRegistry)
   {
      registry = new YoVariableRegistry(getClass().getSimpleName() + side.getPascalCaseName());
      linePointA = new YoFramePoint2D("FootRotationPointA", ReferenceFrame.getWorldFrame(), registry);
      linePointB = new YoFramePoint2D("FootRotationPointB", ReferenceFrame.getWorldFrame(), registry);

      lineCalculator = new OnlineLine2DLinearRegression("FootRotation", registry);
      lineOfRotationInSole = new YoFrameLine2D(side.getCamelCaseName() + "LineOfRotation", "", soleFrame, registry);

      parentRegistry.addChild(registry);
      if (graphicsListRegistry != null)
      {
         Artifact lineArtifact = new YoArtifactLineSegment2d(side.getLowerCaseName() + "LineOfRotation", linePointA, linePointB, Color.RED, 0.005, 0.01);
         graphicsListRegistry.registerArtifact(getClass().getSimpleName(), lineArtifact);
      }
   }

   private final FramePoint2D tempPoint = new FramePoint2D();

   public void compute(FramePoint2DReadOnly measuredCoP)
   {
      tempPoint.setMatchingFrame(measuredCoP);
      lineCalculator.update(tempPoint);
      lineOfRotationInWorld.set(ReferenceFrame.getWorldFrame(), lineCalculator.getMeanLine());
      lineOfRotationInSole.setMatchingFrame(lineOfRotationInWorld);

      updateGraphics();
   }

   public void reset()
   {
      linePointA.setToNaN();
      linePointB.setToNaN();

      lineCalculator.reset();
   }


   private void updateGraphics()
   {
      linePointA.set(lineOfRotationInWorld.getDirection());
      linePointA.scale(-0.05);
      linePointA.add(lineOfRotationInWorld.getPoint());

      linePointB.set(lineOfRotationInWorld.getDirection());
      linePointB.scale(0.05);
      linePointB.add(lineOfRotationInWorld.getPoint());
   }

   public FrameLine2DReadOnly getLineOfRotation()
   {
      return lineOfRotationInSole;
   }

}
