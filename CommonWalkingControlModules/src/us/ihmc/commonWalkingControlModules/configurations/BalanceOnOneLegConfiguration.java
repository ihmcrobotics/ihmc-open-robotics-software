package us.ihmc.commonWalkingControlModules.configurations;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Random;

import us.ihmc.tools.ArrayTools;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;

public class BalanceOnOneLegConfiguration
{
   private final double[] yawPitchRoll;
   private final FramePoint desiredCapturePoint;
   private final FramePoint desiredSwingFootPosition;
   private final double kneeBendSupportLeg;

   public BalanceOnOneLegConfiguration(double[] yawPitchRoll, FramePoint desiredCapturePoint, FramePoint desiredSwingFootPosition, double kneeBendSupportLeg)
   {
      this.yawPitchRoll = ArrayTools.copyArray(yawPitchRoll);
      this.desiredCapturePoint = new FramePoint(desiredCapturePoint);
      this.desiredSwingFootPosition = new FramePoint(desiredSwingFootPosition);
      this.kneeBendSupportLeg = kneeBendSupportLeg;
   }

   public double[] getYawPitchRoll()
   {
      return yawPitchRoll;
   }

   public FramePoint getDesiredCapturePoint()
   {
      return desiredCapturePoint;
   }

   public FramePoint getDesiredSwingFootPosition()
   {
      return desiredSwingFootPosition;
   }

   public double getKneeBendSupportLeg()
   {
      return kneeBendSupportLeg;
   }

   public String toString()
   {
      return Arrays.toString(yawPitchRoll) + ", " + desiredCapturePoint + ", " + desiredSwingFootPosition;
   }

   public static ArrayList<BalanceOnOneLegConfiguration> generateABunch(int desiredNumberOfConfigurations, RobotSide supportSide,
         ReferenceFrame supportFootZUpFrame, boolean randomOrientation) // int xyCapturePositions, int yawPitchRollPositions, int swingPositions)
   {
      ArrayList<BalanceOnOneLegConfiguration> ret = new ArrayList<BalanceOnOneLegConfiguration>(desiredNumberOfConfigurations);

      double captureMinX = 0.02;
      double captureMaxX = 0.06;

      double captureMinY = -0.01;
      double captureMaxY = 0.01;

      double[] yawPitchRollMin;
      double[] yawPitchRollMax;
      if (randomOrientation)
      {
         yawPitchRollMin = new double[] {-0.1, -0.1, -0.1};
         yawPitchRollMax = new double[] {0.1, 0.1, 0.1};
      }
      else
      {
         yawPitchRollMin = new double[] {0.0, 0.0, 0.0};
         yawPitchRollMax = new double[] {0.0, 0.0, 0.0};
      }

      double[] swingMin = new double[] { -0.2, 0.2, 0.10 };
      double[] swingMax = new double[] { 0.2, 0.4, 0.30 };

      double kneeBendSupportLegMin = 0.0 * Math.PI / 180;
      double kneeBendSupportLegMax = 60.0 * Math.PI / 180;

      Random random = new Random(101L);

      for (int i = 1; i <= desiredNumberOfConfigurations; i++)
      {
         double captureX = createRandomDoubleInRange(captureMinX, captureMaxX, random);
         double captureY = createRandomDoubleInRange(captureMinY, captureMaxY, random);

         double yaw = createRandomDoubleInRange(yawPitchRollMin[0], yawPitchRollMax[0], random);
         double pitch = createRandomDoubleInRange(yawPitchRollMin[1], yawPitchRollMax[1], random);
         double roll = createRandomDoubleInRange(yawPitchRollMin[2], yawPitchRollMax[2], random);

         double swingX = createRandomDoubleInRange(swingMin[0], swingMax[0], random);
         double swingY = createRandomDoubleInRange(swingMin[1], swingMax[1], random);
         double swingZ = createRandomDoubleInRange(swingMin[2], swingMax[2], random);

         swingY = supportSide.negateIfLeftSide(swingY);

         double kneeBendSupportLeg = createRandomDoubleInRange(kneeBendSupportLegMin, kneeBendSupportLegMax, random);

         double[] yawPitchRoll = new double[] { yaw, pitch, roll };
         FramePoint desiredCapturePoint = new FramePoint(supportFootZUpFrame, captureX, captureY, 0.0);
         FramePoint desiredSwingFootPosition = new FramePoint(supportFootZUpFrame, swingX, swingY, swingZ);

         BalanceOnOneLegConfiguration configuration = new BalanceOnOneLegConfiguration(yawPitchRoll, desiredCapturePoint, desiredSwingFootPosition,
               kneeBendSupportLeg);

         ret.add(configuration);
      }

      return ret;
   }

   public static ArrayList<BalanceOnOneLegConfiguration> generateForwardBackward(RobotSide supportSide, ReferenceFrame supportFootZUpFrame)
   {
      ArrayList<BalanceOnOneLegConfiguration> ret = new ArrayList<BalanceOnOneLegConfiguration>(2);

      double[] yawPitchRoll = new double[] {0.0, 0.0, supportSide.negateIfRightSide(0.05)};

      double kneeBendSupportLeg = 0.0; // not used right now

      FramePoint desiredCapturePoint0 = new FramePoint(supportFootZUpFrame, 0.08, 0.0, 0.0); // not used right now
      FramePoint desiredCapturePoint1 = new FramePoint(supportFootZUpFrame, 0.02, 0.0, 0.0); // not used right now


      double swingForwardX = 0.3;
      double swingBackwardX = -0.1;
      double swingY = supportSide.negateIfLeftSide(0.25);
      double swingZ = 0.03;
      FramePoint desiredSwingFootPosition0 = new FramePoint(supportFootZUpFrame, swingBackwardX, swingY, swingZ);
      FramePoint desiredSwingFootPosition1 = new FramePoint(supportFootZUpFrame, swingForwardX, swingY, swingZ);

      ret.add(new BalanceOnOneLegConfiguration(yawPitchRoll, desiredCapturePoint0, desiredSwingFootPosition0, kneeBendSupportLeg));
      ret.add(new BalanceOnOneLegConfiguration(yawPitchRoll, desiredCapturePoint1, desiredSwingFootPosition1, kneeBendSupportLeg));

      return ret;
   }

   private static double createRandomDoubleInRange(Double start, Double end, Random random)
   {
      if (start > end)
      {
         throw new IllegalArgumentException("Start cannot exceed End.");
      }

      Double range = end - start;

      // compute a fraction of the range, 0 <= fraction < range
      Double fraction = (range * random.nextDouble());

      return (fraction + start);
   }

   public static void main(String[] args)
   {
      RobotSide supportSide = RobotSide.LEFT;
      ReferenceFrame supportFootFrame = ReferenceFrame.getWorldFrame();

      ArrayList<BalanceOnOneLegConfiguration> configurations = generateABunch(100000, supportSide, supportFootFrame, true);

      for (BalanceOnOneLegConfiguration configuration : configurations)
      {
         System.out.println(configuration);
      }

   }
}
