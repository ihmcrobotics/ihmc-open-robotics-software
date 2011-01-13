package us.ihmc.commonWalkingControlModules.configurations;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Iterator;
import java.util.Random;

import us.ihmc.commonWalkingControlModules.RobotSide;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

public class BalanceOnOneLegConfiguration
{
   private final double[] yawPitchRoll;
   private final FramePoint desiredCapturePoint;
   private final FramePoint desiredSwingFootPosition;
   private final double kneeBendSupportLeg;

   public BalanceOnOneLegConfiguration(double[] yawPitchRoll, FramePoint desiredCapturePoint, FramePoint desiredSwingFootPosition, double kneeBendSupportLeg)
   {
      this.yawPitchRoll = yawPitchRoll;
      this.desiredCapturePoint = desiredCapturePoint;
      this.desiredSwingFootPosition = desiredSwingFootPosition;
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

//   public static ArrayList<BalanceOnOneLegConfiguration> generateABunch(RobotSide supportSide, ReferenceFrame supportFootFrame)    // int xyCapturePositions, int yawPitchRollPositions, int swingPositions)
//   {
//      ArrayList<BalanceOnOneLegConfiguration> ret = new ArrayList<BalanceOnOneLegConfiguration>();
//
//      double captureMinX = 0.02;
//      double captureMaxX = 0.06;
//
//      double captureMinY = -0.01;
//      double captureMaxY = 0.01;
//
//      double[] yawPitchRollMin = new double[] {-0.1, -0.1, -0.1};
//      double[] yawPitchRollMax = new double[] {0.1, 0.1, 0.1};
//
//      double[] swingMin = new double[] {-0.2, 0.2, 0.10};
//      double[] swingMax = new double[] {0.2, 0.4, 0.30};
//
//      int xyCaptureIterations = 4;
//      int yawPitchRollIterations = 4;
//      int swingXYZIterations = 4;
//
//      EvenDistributor captureXs = new EvenDistributor(captureMinX, captureMaxX, xyCaptureIterations);
//      EvenDistributor captureYs = new EvenDistributor(captureMinY, captureMaxY, xyCaptureIterations);
//
//      EvenDistributor yaws = new EvenDistributor(yawPitchRollMin[0], yawPitchRollMax[0], yawPitchRollIterations);
//      EvenDistributor pitches = new EvenDistributor(yawPitchRollMin[1], yawPitchRollMax[1], yawPitchRollIterations);
//      EvenDistributor rolls = new EvenDistributor(yawPitchRollMin[2], yawPitchRollMax[2], yawPitchRollIterations);
//
//      EvenDistributor swingXs = new EvenDistributor(swingMin[0], swingMax[0], swingXYZIterations);
//      EvenDistributor swingYs = new EvenDistributor(swingMin[1], swingMax[1], swingXYZIterations);
//      EvenDistributor swingZs = new EvenDistributor(swingMin[2], swingMax[2], swingXYZIterations);
//
//      for (Double captureX : captureXs)
//      {
//         for (Double captureY : captureYs)
//         {
//            for (Double yaw : yaws)
//            {
//               for (Double pitch : pitches)
//               {
//                  for (Double roll : rolls)
//                  {
//                     for (Double swingX : swingXs)
//                     {
//                        for (Double swingY : swingYs)
//                        {
//                           swingY = supportSide.negateIfLeftSide(swingY);
//
//                           for (Double swingZ : swingZs)
//                           {
//                              double[] yawPitchRoll = new double[] {yaw, pitch, roll};
//                              FramePoint desiredCapturePoint = new FramePoint(supportFootFrame, captureX, captureY, 0.0);
//                              FramePoint desiredSwingFootPosition = new FramePoint(supportFootFrame, swingX, swingY, swingZ);
//
//                              BalanceOnOneLegConfiguration configuration = new BalanceOnOneLegConfiguration(yawPitchRoll, desiredCapturePoint,
//                                                                              desiredSwingFootPosition);
//
//                              ret.add(configuration);
//                           }
//                        }
//                     }
//                  }
//               }
//            }
//         }
//      }
//
//      return ret;
//   }


   public static ArrayList<BalanceOnOneLegConfiguration> generateABunch(int desiredNumberOfConfigurations, RobotSide supportSide,
           ReferenceFrame supportFootFrame)    // int xyCapturePositions, int yawPitchRollPositions, int swingPositions)
   {
      ArrayList<BalanceOnOneLegConfiguration> ret = new ArrayList<BalanceOnOneLegConfiguration>();

      double captureMinX = 0.02;
      double captureMaxX = 0.06;

      double captureMinY = -0.01;
      double captureMaxY = 0.01;

      double[] yawPitchRollMin = new double[] {-0.1, -0.1, -0.1};
      double[] yawPitchRollMax = new double[] {0.1, 0.1, 0.1};

      double[] swingMin = new double[] {-0.2, 0.2, 0.10};
      double[] swingMax = new double[] {0.2, 0.4, 0.30};

      double kneeBendSupportLegMin = 0.0 * Math.PI / 180;
      double kneeBendSupportLegMax = 60.0 * Math.PI / 180;

//    int xyCaptureIterations = 4;
//    int yawPitchRollIterations = 4;
//    int swingXYZIterations = 4;
//
//    EvenDistributor captureXs = new EvenDistributor(captureMinX, captureMaxX, xyCaptureIterations);
//    EvenDistributor captureYs = new EvenDistributor(captureMinY, captureMaxY, xyCaptureIterations);
//
//    EvenDistributor yaws = new EvenDistributor(yawPitchRollMin[0], yawPitchRollMax[0], yawPitchRollIterations);
//    EvenDistributor pitches = new EvenDistributor(yawPitchRollMin[1], yawPitchRollMax[1], yawPitchRollIterations);
//    EvenDistributor rolls = new EvenDistributor(yawPitchRollMin[2], yawPitchRollMax[2], yawPitchRollIterations);
//
//    EvenDistributor swingXs = new EvenDistributor(swingMin[0], swingMax[0], swingXYZIterations);
//    EvenDistributor swingYs = new EvenDistributor(swingMin[1], swingMax[1], swingXYZIterations);
//    EvenDistributor swingZs = new EvenDistributor(swingMin[2], swingMax[2], swingXYZIterations);
//
//    EnumMap<DistributorType, EvenDistributor> distributors = new EnumMap<DistributorType, EvenDistributor>(DistributorType.class);
//    distributors.put(DistributorType.CAPTURE_X, captureXs);
//    distributors.put(DistributorType.CAPTURE_Y, captureYs);
//    distributors.put(DistributorType.YAW, yaws);
//    distributors.put(DistributorType.PITCH, pitches);
//    distributors.put(DistributorType.ROLL, rolls);
//    distributors.put(DistributorType.SWING_X, swingXs);
//    distributors.put(DistributorType.SWING_Y, swingYs);
//    distributors.put(DistributorType.SWING_Z, swingZs);


      for (int i = 1; i <= desiredNumberOfConfigurations; i++)
      {
         Random random = new Random();

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

         double[] yawPitchRoll = new double[] {yaw, pitch, roll};
         FramePoint desiredCapturePoint = new FramePoint(supportFootFrame, captureX, captureY, 0.0);
         FramePoint desiredSwingFootPosition = new FramePoint(supportFootFrame, swingX, swingY, swingZ);

         BalanceOnOneLegConfiguration configuration = new BalanceOnOneLegConfiguration(yawPitchRoll, desiredCapturePoint, desiredSwingFootPosition,
                                                          kneeBendSupportLeg);

         ret.add(configuration);
      }

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

      ArrayList<BalanceOnOneLegConfiguration> configurations = generateABunch(100000, supportSide, supportFootFrame);

      for (BalanceOnOneLegConfiguration configuration : configurations)
      {
         System.out.println(configuration);
      }

   }

   private static class EvenDistributor implements Iterable<Double>
   {
      private final int number;

      private final double min, max;

      public EvenDistributor(double min, double max, int number)
      {
         this.min = min;
         this.max = max;
         this.number = number;
      }

      public Iterator<Double> iterator()
      {
         Itr ret = new Itr(min, max, number);

         return ret;
      }
   }


   private static class Itr implements Iterator<Double>
   {
      private int index = 0;
      private final int number;

      private final double min, max;

      public Itr(double min, double max, int number)
      {
         this.number = number;
         this.min = min;
         this.max = max;
      }

      public boolean hasNext()
      {
         if (index < number)
            return true;

         return false;
      }

      public Double next()
      {
         double ret = min + (max - min) * ((double) index) / ((double) (number - 1));

         index++;

         return ret;
      }

      public void remove()
      {
         throw new UnsupportedOperationException("Cannot remove elements.");
      }

   }
}
