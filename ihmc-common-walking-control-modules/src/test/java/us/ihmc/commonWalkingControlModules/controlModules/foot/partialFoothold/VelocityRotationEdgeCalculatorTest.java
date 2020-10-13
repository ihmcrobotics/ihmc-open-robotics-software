package us.ihmc.commonWalkingControlModules.controlModules.foot.partialFoothold;

import org.junit.jupiter.api.Test;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FeetManager;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootRotationDetector;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTestTools;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLine2DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.parameters.DefaultParameterReader;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.Random;

public class VelocityRotationEdgeCalculatorTest extends RotationEdgeCalculatorTest
{
   @Override
   public RotationEdgeCalculator getEdgeCalculator()
   {
      double breakFrequency = 1.0;
      double stableAngleThreshold = 3.0;
      double stablePositionThreshold = 0.5;
      int stableWindowSize = 5;
      return new VelocityRotationEdgeCalculator(side,
                                                soleFrame,
                                                () -> breakFrequency,
                                                () -> stableAngleThreshold,
                                                () -> stablePositionThreshold,
                                                () -> stableWindowSize,
                                                dt,
                                                registry,
                                                null);
   }
}
