package us.ihmc.commonWalkingControlModules.controlModules.foot.partialFoothold;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.InterpolationTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTestTools;
import us.ihmc.euclid.referenceFrame.FrameLine2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLine2DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.robotSide.RobotSide;

import static us.ihmc.robotics.Assert.assertTrue;

public class CoPAndVelocityRotationEdgeCalculatorTest extends RotationEdgeCalculatorTest
{
   @Override
   protected RotationEdgeCalculator getEdgeCalculator()
   {
      double breakFrequency = 1.0;
      double copBreakFrequency = 30.0;
      double stableAngleThreshold = 3.0;
      double stablePositionThreshold = 0.5;
      int stableWindowSize = 5;
      return new CoPAndVelocityRotationEdgeCalculator(RobotSide.LEFT,
                                                      soleFrame,
                                                      () -> breakFrequency,
                                                      () -> copBreakFrequency,
                                                      () -> stableAngleThreshold,
                                                      () -> stablePositionThreshold,
                                                      () -> stableWindowSize,
                                                      dt,
                                                      registry,
                                                      null,
                                                      null);
   }


}
