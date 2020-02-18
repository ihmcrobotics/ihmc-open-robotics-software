package us.ihmc.commonWalkingControlModules.controlModules.foot;

import us.ihmc.euclid.referenceFrame.FrameLine3D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLine3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.graphicsDescription.plotting.artifact.Artifact;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactLineSegment2d;
import us.ihmc.robotics.functionApproximation.OnlineLine2DLinearRegression;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFramePoint2D;

import java.awt.*;

public class CoPHistoryRotationEdgeCalculator implements RotationEdgeCalculator
{
   private final YoVariableRegistry registry;

   private final YoFramePoint2D linePointA;
   private final YoFramePoint2D linePointB;

   private final OnlineLine2DLinearRegression lineCalculator;

   public CoPHistoryRotationEdgeCalculator(RobotSide side, YoVariableRegistry parentRegistry, YoGraphicsListRegistry graphicsListRegistry)
   {
      registry = new YoVariableRegistry(getClass().getSimpleName() + side.getPascalCaseName());
      linePointA = new YoFramePoint2D("FootRotationPointA", ReferenceFrame.getWorldFrame(), registry);
      linePointB = new YoFramePoint2D("FootRotationPointB", ReferenceFrame.getWorldFrame(), registry);

      lineCalculator = new OnlineLine2DLinearRegression("FootRotation", registry);

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

      updateGraphics();
   }

   public void reset()
   {
      linePointA.setToNaN();
      linePointB.setToNaN();

      lineCalculator.reset();
   }

   private final FrameLine3DBasics tempLineOfRotationInWorld = new FrameLine3D();

   private void updateGraphics()
   {
      tempLineOfRotationInWorld.getPoint().set(lineCalculator.getMeanLine().getPoint());
      tempLineOfRotationInWorld.getDirection().set(lineCalculator.getMeanLine().getDirection());
      tempLineOfRotationInWorld.changeFrame(ReferenceFrame.getWorldFrame());

      linePointA.set(tempLineOfRotationInWorld.getDirection());
      linePointA.scale(-0.05);
      linePointA.add(tempLineOfRotationInWorld.getPointX(), tempLineOfRotationInWorld.getPointY());

      linePointB.set(tempLineOfRotationInWorld.getDirection());
      linePointB.scale(0.05);
      linePointB.add(tempLineOfRotationInWorld.getPointX(), tempLineOfRotationInWorld.getPointY());
   }
}
