package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator;

import java.util.ArrayList;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class NewDoubleSupportICPComputer
{
   public static Point3d[] computeICPCornerPoints(int numberOfCornerPoints, ArrayList<Point3d> constantEquivalentCoPs, double steppingTime, double omega0)
   {
      Point3d[] icpCornerPoints = new Point3d[numberOfCornerPoints];

      Point3d icpCornerPoint = JojosICPutilities.extrapolateDCMpos(constantEquivalentCoPs.get(numberOfCornerPoints - 1), -steppingTime, omega0,
            constantEquivalentCoPs.get(numberOfCornerPoints));

      icpCornerPoints[numberOfCornerPoints - 1] = icpCornerPoint;

      for (int i = numberOfCornerPoints - 1; i > 0; i--)
      {
         icpCornerPoint = JojosICPutilities.extrapolateDCMpos(constantEquivalentCoPs.get(i - 1), -steppingTime, omega0, icpCornerPoints[i]);
         icpCornerPoints[i - 1] = icpCornerPoint;
      }

      return icpCornerPoints;
   }

   public static void computeSingleSupportStartICPAndVelocity(Point3d singleSupportICPToPack, Vector3d singleSupportICPVelocityToPack,
         Point3d constantCenterOfPressure, Point3d cornerPoint0, double doubleSupportDuration, double doubleSupportFirstStepFraction, double omega0)
   {
      double initialDoubleSupportDuration = doubleSupportDuration * (1.0 - doubleSupportFirstStepFraction);

      JojosICPutilities.extrapolateDCMposAndVel(singleSupportICPToPack, singleSupportICPVelocityToPack, constantCenterOfPressure, initialDoubleSupportDuration,
            omega0, cornerPoint0);
   }

   public static void computeSingleSupportEndICPAndVelocity(Point3d singleSupportICPToPack, Vector3d singleSupportICPVelocityToPack,
         Point3d constantCenterOfPressure, Point3d cornerPoint0, double doubleSupportDuration, double doubleSupportFirstStepFraction,
         double singleSupportDuration, double omega0)
   {
      double initialDoubleSupportDuration = doubleSupportDuration * (1.0 - doubleSupportFirstStepFraction);

      JojosICPutilities.extrapolateDCMposAndVel(singleSupportICPToPack, singleSupportICPVelocityToPack, constantCenterOfPressure, initialDoubleSupportDuration
            + singleSupportDuration, omega0, cornerPoint0);
   }

   public static Point3d computeSingleSupportStartICP(Point3d constantCenterOfPressure, Point3d cornerPoint0, double doubleSupportDuration,
         double doubleSupportFirstStepFraction, double omega0)
   {
      Point3d singleSupportStateICP = new Point3d();
      Vector3d singleSupportStateICPVelocity = new Vector3d();

      double initialDoubleSupportDuration = doubleSupportDuration * (1.0 - doubleSupportFirstStepFraction);

      JojosICPutilities.extrapolateDCMposAndVel(singleSupportStateICP, singleSupportStateICPVelocity, constantCenterOfPressure, initialDoubleSupportDuration,
            omega0, cornerPoint0);

      return singleSupportStateICP;
   }

   public static Point3d computeSingleSupportEndICP(Point3d constantCenterOfPressure, Point3d cornerPoint0, double doubleSupportDuration,
         double doubleSupportFirstStepFraction, double singleSupportDuration, double omega0)
   {
      Point3d singleSupportStateICP = new Point3d();
      Vector3d singleSupportStateICPVelocity = new Vector3d();

      double initialDoubleSupportDuration = doubleSupportDuration * (1.0 - doubleSupportFirstStepFraction);

      JojosICPutilities.extrapolateDCMposAndVel(singleSupportStateICP, singleSupportStateICPVelocity, constantCenterOfPressure, initialDoubleSupportDuration
            + singleSupportDuration, omega0, cornerPoint0);

      return singleSupportStateICP;
   }

   public static void computeSingleSupportICPPositionAndVelocity(Point3d icpPositionToPack, Vector3d icpVelocityToPack, Point3d constantCenterOfPressure,
         Point3d singleSupportStartICP, double omega0, double time)
   {
      JojosICPutilities.extrapolateDCMposAndVel(icpPositionToPack, icpVelocityToPack, constantCenterOfPressure, time, omega0, singleSupportStartICP);
   }

   public static void computeConstantCentersOfPressure(ArrayList<YoFramePoint> constantCentersOfPressureToModify, ArrayList<FramePoint> footLocations,
         int maxNumberOfConsideredFootsteps, boolean isInitialTransfer)
   {
      boolean putFirstCenterOfPressureInMiddle = isInitialTransfer;

      int numberInFootlist = footLocations.size();
      Point3d positionToHoldAt = new Point3d();

      YoFramePoint centerOfPressureLocation;

      for (int i = 0; i < maxNumberOfConsideredFootsteps; i++)
      {
         centerOfPressureLocation = constantCentersOfPressureToModify.get(i);

         if (i == 0)
         {
            if (putFirstCenterOfPressureInMiddle)
            {
               centerOfPressureLocation.set(footLocations.get(0).getPoint());
               centerOfPressureLocation.add(footLocations.get(1).getPoint());
               centerOfPressureLocation.scale(0.5);
            }
            else
            {
               centerOfPressureLocation.set(footLocations.get(0).getPoint());
            }
         }

         else if (i < numberInFootlist - 1)
         {
            centerOfPressureLocation.set(footLocations.get(i).getPoint());
         }

         else
         {
            if (i == numberInFootlist - 1)
            {
               positionToHoldAt.set(footLocations.get(i).getPoint());
               positionToHoldAt.add(footLocations.get(i - 1).getPoint());
               positionToHoldAt.scale(0.5);
            }

            centerOfPressureLocation.set(positionToHoldAt);
         }
      }
   }

   public static void computeConstantCentersOfPressureAndCornerPointsForFootCenterAndToe(ArrayList<YoFramePoint> constantFootCenterCentersOfPressureToModify,
         ArrayList<YoFramePoint> constantToeCentersOfPressureToModify, ArrayList<YoFramePoint> footCenterCornerPointsToModify,
         ArrayList<YoFramePoint> toeCornerPointsToModify, double maxFrontalToeOffset, ArrayList<FramePoint> footLocations,
         ArrayList<ReferenceFrame> soleFrameList, int maxNumberOfConsideredFootsteps, boolean isInitialTransfer, double omega0,
         double toeToFootCenterShiftDuration, double footCenterToToeShiftDuration)
   {
      computeConstantCentersOfPressure(constantFootCenterCentersOfPressureToModify, footLocations, maxNumberOfConsideredFootsteps, isInitialTransfer);

      computeConstantToeCentersOfPressure(constantToeCentersOfPressureToModify, constantFootCenterCentersOfPressureToModify, footLocations, soleFrameList,
            maxNumberOfConsideredFootsteps, maxFrontalToeOffset);

      int numberOfCornerPoints = maxNumberOfConsideredFootsteps - 1;

      Point3d icpToeCornerPoint = JojosICPutilities.extrapolateDCMpos(constantToeCentersOfPressureToModify.get(numberOfCornerPoints - 1).getPoint3dCopy(),
            -toeToFootCenterShiftDuration, omega0, constantFootCenterCentersOfPressureToModify.get(numberOfCornerPoints).getPoint3dCopy());

      toeCornerPointsToModify.get(numberOfCornerPoints - 1).set(icpToeCornerPoint);

      Point3d icpFootCenterCornerPoint = JojosICPutilities.extrapolateDCMpos(constantFootCenterCentersOfPressureToModify.get(numberOfCornerPoints - 1)
            .getPoint3dCopy(), -footCenterToToeShiftDuration, omega0, icpToeCornerPoint);
      footCenterCornerPointsToModify.get(numberOfCornerPoints - 1).set(icpFootCenterCornerPoint);

      for (int i = numberOfCornerPoints - 1; i > 0; i--)
      {
         icpToeCornerPoint = JojosICPutilities.extrapolateDCMpos(constantToeCentersOfPressureToModify.get(i - 1).getPoint3dCopy(),
               -toeToFootCenterShiftDuration, omega0, footCenterCornerPointsToModify.get(i).getPoint3dCopy());

         toeCornerPointsToModify.get(i - 1).set(icpToeCornerPoint);

         icpFootCenterCornerPoint = JojosICPutilities.extrapolateDCMpos(constantFootCenterCentersOfPressureToModify.get(i - 1).getPoint3dCopy(),
               -footCenterToToeShiftDuration, omega0, icpToeCornerPoint);
         footCenterCornerPointsToModify.get(i - 1).set(icpFootCenterCornerPoint);
      }
   }

   private static void computeConstantToeCentersOfPressure(ArrayList<YoFramePoint> constantToeCentersOfPressureToModify,
         ArrayList<YoFramePoint> constantFootCenterCentersOfPressure, ArrayList<FramePoint> footLocations, ArrayList<ReferenceFrame> soleFrameList,
         int maxNumberOfConsideredFootsteps, double maxFrontalToeOffset)
   {
      for (int i = 0; i < maxNumberOfConsideredFootsteps; i++)
      {
         double localFrontalToeOffset;
         double desiredFrontalToeOffset;

         int finalShiftIsFromFootCenter = 1; // set to zero to have the final icp shift going over the toe-ICP-waypoint

         //         if (i >= constantFootCenterCentersOfPressure.size() - 1 - finalShiftIsFromFootCenter) 
         if (i >= footLocations.size() - 1 - finalShiftIsFromFootCenter)
         {
            desiredFrontalToeOffset = 0.0;
         }
         else
         {
            FramePoint localCurrentFramePoint = new FramePoint();
            localCurrentFramePoint.set(constantFootCenterCentersOfPressure.get(i).getFramePointCopy());
            localCurrentFramePoint.changeFrame(soleFrameList.get(Math.min(i, soleFrameList.size() - 1)));

            FramePoint localNextFramePoint = new FramePoint();
            localNextFramePoint.set(constantFootCenterCentersOfPressure.get(i + 1).getFramePointCopy());
            localNextFramePoint.changeFrame(soleFrameList.get(Math.min(i, soleFrameList.size() - 1)));
            desiredFrontalToeOffset = localNextFramePoint.getX() - localCurrentFramePoint.getX();
         }

         localFrontalToeOffset = Math.max(Math.min(desiredFrontalToeOffset, maxFrontalToeOffset), 0);

         FrameVector toeOffsetFromFootCenter = new FrameVector(soleFrameList.get(Math.min(i, soleFrameList.size() - 1)));
         toeOffsetFromFootCenter.set(localFrontalToeOffset, 0.0, 0.0);
         toeOffsetFromFootCenter.changeFrame(constantFootCenterCentersOfPressure.get(i).getReferenceFrame());

         constantToeCentersOfPressureToModify.get(i).set(constantFootCenterCentersOfPressure.get(i));
         constantToeCentersOfPressureToModify.get(i).add(toeOffsetFromFootCenter);
      }
   }
}
