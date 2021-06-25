package us.ihmc.commonWalkingControlModules.captureRegion;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint2D;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.awt.*;
import java.util.ArrayList;
import java.util.List;

public interface MultiStepPushRecoveryCalculatorVisualizer
{
   void reset();

   void visualize(MultiStepRecoveryStepCalculator pushRecoveryCalculator);
}
