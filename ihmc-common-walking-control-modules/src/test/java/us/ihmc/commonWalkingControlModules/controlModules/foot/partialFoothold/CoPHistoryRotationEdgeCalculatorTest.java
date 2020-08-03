package us.ihmc.commonWalkingControlModules.controlModules.foot.partialFoothold;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.math.functionGenerator.YoFunctionGenerator;
import us.ihmc.robotics.math.functionGenerator.YoFunctionGeneratorMode;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.registry.YoRegistry;

public class CoPHistoryRotationEdgeCalculatorTest extends RotationEdgeCalculatorTest
{
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   @Override
   protected RotationEdgeCalculator getEdgeCalculator()
   {
      double stableAngleThreshold = 3.0;
      double stablePositionThreshold = 0.5;
      int stableWindowSize = 5;
      return new CoPHistoryRotationEdgeCalculator(RobotSide.LEFT,
                                                  worldFrame,
                                                  () -> stableAngleThreshold,
                                                  () -> stablePositionThreshold,
                                                  () -> stableWindowSize,
                                                  () -> 0.01,
                                                  () -> 0.001,
                                                  dt,
                                                  registry,
                                                  null,
                                                  null);
   }

   // method relies on history, so that's bad
   @Disabled
   @Test
   public void testRotationDetectionMath()
   {
   }

   @Test
   public void testCoPHistory()
   {
      double dt = 0.001;
      YoRegistry registry = new YoRegistry("Dummy");
      double stableAngleThreshold = 3.0;
      double stablePositionThreshold = 0.5;
      int stableWindowSize = 5;
      CoPHistoryRotationEdgeCalculator edgeCalculator = new CoPHistoryRotationEdgeCalculator(RobotSide.LEFT,
                                                                                             worldFrame,
                                                                                             () -> stableAngleThreshold,
                                                                                             () -> stablePositionThreshold,
                                                                                             () -> stableWindowSize,
                                                                                             () -> 0.01,
                                                                                             () -> 0.001,
                                                                                             dt,
                                                                                             registry,
                                                                                             null,
                                                                                             null);

      Point2D linePosition = new Point2D(0.05, -0.07);
      Vector2D lineDirection = new Vector2D(-0.6, 0.15);
      lineDirection.normalize();
      double lineAngle = Math.atan2(lineDirection.getY(), lineDirection.getX());
      Quaternion lineRotation = new Quaternion();
      lineRotation.setYawPitchRoll(lineAngle, 0.0, 0.0);

      PoseReferenceFrame lineFrame = new PoseReferenceFrame("lineFrame", worldFrame);
      lineFrame.setPoseAndUpdate(new Point3D(linePosition), lineRotation);

      YoFunctionGenerator positionFunctionGenerator = new YoFunctionGenerator("positionFunctionGenerator", registry);
      positionFunctionGenerator.setAmplitude(2.0 * stablePositionThreshold);
      positionFunctionGenerator.setFrequency(1.0);
      positionFunctionGenerator.setMode(YoFunctionGeneratorMode.TRIANGLE);

      //      YoFunctionGenerator positionFunctionGenerator = new YoFunctionGenerator("positionFunctionGenerator", registry);
      //      positionFunctionGenerator.setAmplitude(2.0 * stablePositionThreshold);
      //      positionFunctionGenerator.setFrequency(1.0);
      //      positionFunctionGenerator.setMode(YoFunctionGeneratorMode.TRIANGLE);

      double timeForEstimate = 10.0;
      for (double time = 0.0; time <= timeForEstimate; time += dt)
      {
         FramePoint2D copPosition = new FramePoint2D(lineFrame);
      }
   }
}
