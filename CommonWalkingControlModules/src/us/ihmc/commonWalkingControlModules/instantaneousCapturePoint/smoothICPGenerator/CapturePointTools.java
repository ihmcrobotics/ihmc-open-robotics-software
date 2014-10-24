package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator;

import java.util.ArrayList;

import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.yoUtilities.math.frames.YoFramePoint;
import us.ihmc.yoUtilities.math.frames.YoFrameVector;

public class CapturePointTools
{
   /**
    * Compute constant centers of pressure, placing the initial COP between the
    * feet and all of the rest on the footsteps.
    * 
    * @param arrayToPack ArrayList that will be packed with the constant center
    *            of pressure locations
    * @param footstepList ArrayList containing the footsteps
    * @param numberFootstepsToConsider Integer describing the number of
    *            footsteps to consider when laying out the COP's
    */
   public static void computeConstantCentersOfPressureWithStartBetweenFeetAndRestOnFeet(ArrayList<YoFramePoint> arrayToPack,
         ArrayList<FramePoint> footstepList, int numberFootstepsToConsider)
   {
      arrayToPack.get(0).setAndMatchFrame(footstepList.get(0));
      arrayToPack.get(0).add(footstepList.get(1));
      arrayToPack.get(0).scale(0.5);

      int numberFootstepsInList = footstepList.size();

      for (int i = 1; i < numberFootstepsToConsider; i++)
      {
         if (i < numberFootstepsInList - 1)
         {
            arrayToPack.get(i).setAndMatchFrame(footstepList.get(i));
         }
         else
         {
            arrayToPack.get(i).setAndMatchFrame(footstepList.get(numberFootstepsInList - 1));
            arrayToPack.get(i).add(footstepList.get(numberFootstepsInList - 2));
            arrayToPack.get(i).scale(0.5);
         }
      }
   }

   /**
    * Put the constant COP'S except the last one on the footsteps. Put the last
    * COP between the feet.
    * 
    * @param arrayToPack ArrayList that will be packed with the constant center
    *            of pressure locations
    * @param footstepList ArrayList containing the footsteps
    * @param numberFootstepsToConsider Integer describing the number of
    *            footsteps to consider when laying out the COP's
    */
   public static void computeConstantCentersOfPressuresOnFeetWithEndBetweenFeet(ArrayList<YoFramePoint> arrayToPack, ArrayList<FramePoint> footstepList,
         int numberFootstepsToConsider)
   {
      int numberFootstepsInList = footstepList.size();

      for (int i = 0; i < numberFootstepsToConsider - 1; i++)
      {
         if (i < numberFootstepsInList - 1)
         {
            arrayToPack.get(i).set(footstepList.get(i));
         }
         else
         {
            arrayToPack.get(i).setAndMatchFrame(footstepList.get(numberFootstepsInList - 1));
            arrayToPack.get(i).add(footstepList.get(numberFootstepsInList - 2));
            arrayToPack.get(i).scale(0.5);
         }
      }

      arrayToPack.get(numberFootstepsToConsider - 1).setAndMatchFrame(footstepList.get(numberFootstepsToConsider - 2));
      arrayToPack.get(numberFootstepsToConsider - 1).add(footstepList.get(numberFootstepsToConsider - 1));
      arrayToPack.get(numberFootstepsToConsider - 1).scale(0.5);
   }

   /**
    * Put the first and last COP between the footsteps, all the rest go on the
    * footsteps.
    * 
    * @param arrayToPack ArrayList that will be packed with the constant center
    *            of pressure locations
    * @param footstepList ArrayList containing the footsteps
    * @param numberFootstepsToConsider Integer describing the number of
    *            footsteps to consider when laying out the COP's
    */
   public static void computeConstantCentersOfPressuresWithBeginningAndEndBetweenFeetRestOnFeet(ArrayList<YoFramePoint> arrayToPack,
         ArrayList<FramePoint> footstepList, int numberFootstepsToConsider)
   {
      arrayToPack.get(0).setAndMatchFrame(footstepList.get(0));
      arrayToPack.get(0).add(footstepList.get(1));
      arrayToPack.get(0).scale(0.5);

      int numberFootstepsInList = footstepList.size();

      for (int i = 1; i < numberFootstepsToConsider - 1; i++)
      {
         if (i < numberFootstepsInList - 1)
         {
            arrayToPack.get(i).setAndMatchFrame(footstepList.get(i));
         }
         else
         {
            arrayToPack.get(i).setAndMatchFrame(footstepList.get(numberFootstepsInList - 1));
            arrayToPack.get(i).add(footstepList.get(numberFootstepsInList - 2));
            arrayToPack.get(i).scale(0.5);
         }
      }

      arrayToPack.get(numberFootstepsToConsider - 1).set(footstepList.get(numberFootstepsToConsider - 2));
      arrayToPack.get(numberFootstepsToConsider - 1).add(footstepList.get(numberFootstepsToConsider - 1));
      arrayToPack.get(numberFootstepsToConsider - 1).scale(0.5);
   }

   /**
    * Put the constant COP'S on the footsteps.
    * 
    * @param arrayToPack ArrayList that will be packed with the constant center
    *            of pressure locations
    * @param footstepList ArrayList containing the footsteps
    * @param numberFootstepsToConsider Integer describing the number of
    *            footsteps to consider when laying out the COP's
    */
   public static void computeConstantCentersOfPressuresOnFeet(ArrayList<YoFramePoint> arrayToPack, ArrayList<FramePoint> footstepList,
         int numberFootstepsToConsider)
   {

      int numberFootstepsInList = footstepList.size();

      for (int i = 0; i < numberFootstepsToConsider; i++)
      {
         if (i < numberFootstepsInList - 1)
         {
            arrayToPack.get(i).setAndMatchFrame(footstepList.get(i));
         }
         else
         {
            arrayToPack.get(i).setAndMatchFrame(footstepList.get(numberFootstepsInList - 1));
            arrayToPack.get(i).add(footstepList.get(numberFootstepsInList - 2));
            arrayToPack.get(i).scale(0.5);
         }
      }
   }

   /**
    * Put the constant COP'S on the feet except the first one. The first one 
    * goes unset. This is useful for single support push recovery.
    * 
    * @param arrayToPack ArrayList that will be packed with the constant center
    *            of pressure locations
    * @param footstepList ArrayList containing the footsteps
    * @param numberFootstepsToConsider Integer describing the number of
    *            footsteps to consider when laying out the COP's
    */
   public static void computeConstantCentersOfPressuresExceptFirstOnFeet(ArrayList<YoFramePoint> arrayToPack, ArrayList<FramePoint> footstepList,
         int numberFootstepsToConsider)
   {

      int numberFootstepsInList = footstepList.size();

      for (int i = 1; i < numberFootstepsToConsider; i++)
      {
         if (i < numberFootstepsInList - 1)
         {
            arrayToPack.get(i).setAndMatchFrame(footstepList.get(i));
         }
         else
         {
            arrayToPack.get(i).setAndMatchFrame(footstepList.get(numberFootstepsInList - 1));
            arrayToPack.get(i).add(footstepList.get(numberFootstepsInList - 2));
            arrayToPack.get(i).scale(0.5);
         }
      }
   }

   /**
    * Backward calculation of desired end of step capture point locations.
    * 
    * @param constantCentersOfPressure
    * @param capturePointsToPack
    * @param stepTime
    * @param omega0
    */
   public static void computeDesiredEndOfStepCapturePointLocations(ArrayList<YoFramePoint> constantCentersOfPressure,
         ArrayList<YoFramePoint> capturePointsToPack, double stepTime, double omega0)
   {
      capturePointsToPack.get(capturePointsToPack.size() - 1).checkReferenceFrameMatch(constantCentersOfPressure.get(capturePointsToPack.size()));
      
      computeInitialCapturePointFromDesiredCapturePointAndInitialCenterOfPressure(omega0, stepTime, constantCentersOfPressure.get(capturePointsToPack.size()),
            constantCentersOfPressure.get(capturePointsToPack.size() - 1), capturePointsToPack.get(capturePointsToPack.size() - 1));

      for (int i = capturePointsToPack.size() - 1; i > 0; i--)
      {
         capturePointsToPack.get(i).checkReferenceFrameMatch(constantCentersOfPressure.get(i-1));
         
         double tmpX = (capturePointsToPack.get(i).getX() - constantCentersOfPressure.get(i - 1).getX()) * (1 / Math.exp(omega0 * stepTime))
               + constantCentersOfPressure.get(i - 1).getX();
         double tmpY = (capturePointsToPack.get(i).getY() - constantCentersOfPressure.get(i - 1).getY()) * (1 / Math.exp(omega0 * stepTime))
               + constantCentersOfPressure.get(i - 1).getY();

         capturePointsToPack.get(i - 1).setX(tmpX);
         capturePointsToPack.get(i - 1).setY(tmpY);
      }
   }

   /**
    * Backward calculation of desired end of step capture point locations.
    * 
    * @param constantCentersOfPressure
    * @param capturePointsToPack
    * @param stepTime
    * @param omega0
    */
   public static void computeDesiredEndOfStepCapturePointLocationsWithFirstLeftUnset(ArrayList<YoFramePoint> constantCentersOfPressure,
         ArrayList<YoFramePoint> capturePointsToPack, double stepTime, double omega0)
   {
      constantCentersOfPressure.get(capturePointsToPack.size()).checkReferenceFrameMatch(capturePointsToPack.get(capturePointsToPack.size() - 1).getReferenceFrame());
            
      computeInitialCapturePointFromDesiredCapturePointAndInitialCenterOfPressure(omega0, stepTime, constantCentersOfPressure.get(capturePointsToPack.size()),
            constantCentersOfPressure.get(capturePointsToPack.size() - 1), capturePointsToPack.get(capturePointsToPack.size() - 1));

      for (int i = capturePointsToPack.size() - 1; i > 1; i--)
      {
         capturePointsToPack.get(i).checkReferenceFrameMatch(constantCentersOfPressure.get(i-1));
         
         double tmpX = (capturePointsToPack.get(i).getX() - constantCentersOfPressure.get(i - 1).getX()) * (1 / Math.exp(omega0 * stepTime))
               + constantCentersOfPressure.get(i - 1).getX();
         double tmpY = (capturePointsToPack.get(i).getY() - constantCentersOfPressure.get(i - 1).getY()) * (1 / Math.exp(omega0 * stepTime))
               + constantCentersOfPressure.get(i - 1).getY();

         capturePointsToPack.get(i - 1).setX(tmpX);
         capturePointsToPack.get(i - 1).setY(tmpY);
      }
   }

   /**
    * Given a desired capturePoint location and an initial position of the capture point,
    * compute the constant center of pressure that will drive the capture point from the 
    * initial position to the final position.
    * 
    * @param finalDesiredCapturePoint
    * @param initialCapturePoint
    * @param centerOfPressureToPack
    * @param omega0
    * @param stepTime
    */
   public static void computeConstantCenterOfPressureFromInitialAndFinalCapturePointLocations(YoFramePoint finalDesiredCapturePoint,
         YoFramePoint initialCapturePoint, YoFramePoint centerOfPressureToPack, double omega0, double stepTime)
   {
      initialCapturePoint.checkReferenceFrameMatch(finalDesiredCapturePoint.getReferenceFrame());
      
      double exponentialTerm = Math.exp(omega0 * stepTime);
      
      double x = (finalDesiredCapturePoint.getX() - initialCapturePoint.getX() * exponentialTerm) / (1 - exponentialTerm);
      double y = (finalDesiredCapturePoint.getY() - initialCapturePoint.getY() * exponentialTerm) / (1 - exponentialTerm);
      
      centerOfPressureToPack.set(initialCapturePoint.getReferenceFrame(), x, y, 0.0);
   }

   /**
    * Given an initial center of pressure location, a final capture point location, 
    * and the step time, compute the initial capture point location. 
    * 
    * @param omega0
    * @param time
    * @param finalCapturePoint
    * @param initialCenterOfPressure
    * @param positionToPack
    */
   public static void computeInitialCapturePointFromDesiredCapturePointAndInitialCenterOfPressure(double omega0, double time, YoFramePoint finalCapturePoint,
         YoFramePoint initialCenterOfPressure, YoFramePoint positionToPack)
   {
      finalCapturePoint.checkReferenceFrameMatch(initialCenterOfPressure.getReferenceFrame());

      double exponentialTerm = Math.exp(omega0 * -time);

      double x = finalCapturePoint.getX()*exponentialTerm + (1 - exponentialTerm) * initialCenterOfPressure.getX();
      double y = finalCapturePoint.getY()*exponentialTerm + (1 - exponentialTerm) * initialCenterOfPressure.getY();
      
      positionToPack.set(initialCenterOfPressure.getReferenceFrame(),x,y,0.0);

   }

   /**
    * Compute the desired capture point position at a given time. ICP_d =
    * e^{w0*t}*ICP_0 + (1-e^{w0*t})*p0
    * 
    * @param omega0
    * @param time
    * @param initialCapturePoint
    * @param initialCenterOfPressure
    * @param positionToPack
    */
   public static void computeDesiredCapturePointPosition(double omega0, double time, YoFramePoint initialCapturePoint, YoFramePoint initialCenterOfPressure,
         YoFramePoint positionToPack)
   {
      initialCapturePoint.checkReferenceFrameMatch(initialCenterOfPressure.getReferenceFrame());

      double exponentialTerm = Math.exp(omega0 * time);
      
      double x = initialCenterOfPressure.getX()*(1-exponentialTerm) + initialCapturePoint.getX()*exponentialTerm;
      double y = initialCenterOfPressure.getY()*(1-exponentialTerm) + initialCapturePoint.getY()*exponentialTerm;
      
      positionToPack.set(initialCenterOfPressure.getReferenceFrame(),x,y,0.0);
   }
   
   /**
    * Compute the desired capture point position at a given time. ICP_d =
    * e^{w0*t}*ICP_0 + (1-e^{w0*t})*p0
    * 
    * @param omega0
    * @param time
    * @param initialCapturePoint
    * @param initialCenterOfPressure
    * @param positionToPack
    */
   public static void computeDesiredCapturePointPosition(double omega0, double time, FramePoint initialCapturePoint, FramePoint initialCenterOfPressure,
         YoFramePoint positionToPack)
   {
      initialCapturePoint.checkReferenceFrameMatch(initialCenterOfPressure.getReferenceFrame());

      double exponentialTerm = Math.exp(omega0 * time);
      
      double x = initialCenterOfPressure.getX()*(1-exponentialTerm) + initialCapturePoint.getX()*exponentialTerm;
      double y = initialCenterOfPressure.getY()*(1-exponentialTerm) + initialCapturePoint.getY()*exponentialTerm;
      
      positionToPack.set(initialCenterOfPressure.getReferenceFrame(),x,y,0.0);
   }

   /**
    * Compute the desired capture point velocity at a given time. ICPv_d = w *
    * e^{w*t} * ICP0 - p0 * w * e^{w*t}
    * 
    * @param omega0
    * @param time
    * @param initialCapturePoint
    * @param initialCenterOfPressure
    * @param velocityToPack
    */
   public static void computeDesiredCapturePointVelocity(double omega0, double time, YoFramePoint initialCapturePoint, YoFramePoint initialCenterOfPressure,
         YoFrameVector velocityToPack)
   {
      initialCapturePoint.checkReferenceFrameMatch(initialCenterOfPressure.getReferenceFrame());

      double exponentialTerm = Math.exp(omega0 * time);

      double x = omega0 * exponentialTerm * (initialCapturePoint.getX() - initialCenterOfPressure.getX());
      double y = omega0 * exponentialTerm * (initialCapturePoint.getY() - initialCenterOfPressure.getY());

      velocityToPack.set(initialCenterOfPressure.getReferenceFrame(),x,y,0.0);
   }
   
   /**
    * Compute the desired capture point velocity at a given time. ICPv_d = w *
    * e^{w*t} * ICP0 - p0 * w * e^{w*t}
    * 
    * @param omega0
    * @param time
    * @param initialCapturePoint
    * @param initialCenterOfPressure
    * @param velocityToPack
    */
   public static void computeDesiredCapturePointVelocity(double omega0, double time, FramePoint initialCapturePoint, FramePoint initialCenterOfPressure,
         YoFrameVector velocityToPack)
   {
      initialCapturePoint.checkReferenceFrameMatch(initialCenterOfPressure.getReferenceFrame());

      double exponentialTerm = Math.exp(omega0 * time);

      double x = omega0 * exponentialTerm * (initialCapturePoint.getX() - initialCenterOfPressure.getX());
      double y = omega0 * exponentialTerm * (initialCapturePoint.getY() - initialCenterOfPressure.getY());

      velocityToPack.set(initialCenterOfPressure.getReferenceFrame(),x,y,0.0);
   }

   /**
    * Compute the desired capture point acceleration given the desired capture
    * point velocity
    * 
    * @param omega0
    * @param desiredCapturePointVelocity
    * @param accelerationToPack
    */
   public static void computeDesiredCapturePointAcceleration(double omega0, YoFrameVector desiredCapturePointVelocity, YoFrameVector accelerationToPack)
   {
      accelerationToPack.setAndMatchFrame(desiredCapturePointVelocity.getFrameTuple());
      accelerationToPack.scale(omega0);
   }

   /**
    * Compute the desired capture point velocity at a given time. ICPv_d = w^2
    * * e^{w*t} * ICP0 - p0 * w^2 * e^{w*t}
    * 
    * @param omega0
    * @param time
    * @param initialCapturePoint
    * @param initialCenterOfPressure
    * @param accelerationToPack
    */
   public static void computeDesiredCapturePointAcceleration(double omega0, double time, YoFramePoint initialCapturePoint,
         YoFramePoint initialCenterOfPressure, YoFrameVector accelerationToPack)
   {
      initialCapturePoint.checkReferenceFrameMatch(initialCenterOfPressure.getReferenceFrame());

      double exponentialTerm = Math.exp(omega0 * time);

      double x = omega0 * omega0 * exponentialTerm * (initialCapturePoint.getX() - initialCenterOfPressure.getX());
      double y = omega0 * omega0 * exponentialTerm * (initialCapturePoint.getY() - initialCenterOfPressure.getY());

      accelerationToPack.set(initialCenterOfPressure.getReferenceFrame(),x,y,0.0);
   }
   
   /**
    * Compute the desired capture point velocity at a given time. ICPv_d = w^2
    * * e^{w*t} * ICP0 - p0 * w^2 * e^{w*t}
    * 
    * @param omega0
    * @param time
    * @param initialCapturePoint
    * @param initialCenterOfPressure
    * @param accelerationToPack
    */
   public static void computeDesiredCapturePointAcceleration(double omega0, double time, FramePoint initialCapturePoint,
         FramePoint initialCenterOfPressure, YoFrameVector accelerationToPack)
   {
      initialCapturePoint.checkReferenceFrameMatch(initialCenterOfPressure.getReferenceFrame());

      double exponentialTerm = Math.exp(omega0 * time);

      double x = omega0 * omega0 * exponentialTerm * (initialCapturePoint.getX() - initialCenterOfPressure.getX());
      double y = omega0 * omega0 * exponentialTerm * (initialCapturePoint.getY() - initialCenterOfPressure.getY());

      accelerationToPack.set(initialCenterOfPressure.getReferenceFrame(),x,y,0.0);
   }

   /**
    * Computes the desired centroidal momentum pivot by,
    * CMP_{d} = ICP_{d} - \dot{ICP}_{d}/omega0
    * 
    * @param desiredCapturePointPosition
    * @param desiredCapturePointVelocity
    * @param omega0
    * @param desiredCentroidalMomentumPivotToPack
    */
   public static void computeDesiredCentroidalMomentumPivot(YoFramePoint desiredCapturePointPosition, YoFrameVector desiredCapturePointVelocity, double omega0,
         YoFramePoint desiredCentroidalMomentumPivotToPack)
   {
      desiredCentroidalMomentumPivotToPack.setAndMatchFrame(desiredCapturePointVelocity.getFrameTuple());
      desiredCentroidalMomentumPivotToPack.scale(-1 / omega0);
      desiredCentroidalMomentumPivotToPack.add(desiredCapturePointPosition);
   }
   
   /**
    * Computes the desired centroidal momentum pivot by,
    * CMP_{d} = ICP_{d} - \dot{ICP}_{d}/omega0
    * 
    * @param desiredCapturePointPosition
    * @param desiredCapturePointVelocity
    * @param omega0
    * @param desiredCentroidalMomentumPivotToPack
    */
   public static void computeDesiredCentroidalMomentumPivot(FramePoint desiredCapturePointPosition, FrameVector desiredCapturePointVelocity, double omega0,
         YoFramePoint desiredCentroidalMomentumPivotToPack)
   {
      desiredCentroidalMomentumPivotToPack.setAndMatchFrame(desiredCapturePointVelocity);
      desiredCentroidalMomentumPivotToPack.scale(-1 / omega0);
      desiredCentroidalMomentumPivotToPack.add(desiredCapturePointPosition);
   }

   /**
    * Compute the distance along the capture point guide line from the 
    * current capture point position to the desired capture point position.
    * 
    * @param currentCapturePointPosition
    * @param desiredCapturePointPosition
    * @param desiredCapturePointVelocity
    * @return
    */
   public static double computeDistanceToCapturePointFreezeLine(FramePoint currentCapturePointPosition, FramePoint desiredCapturePointPosition,
         FrameVector desiredCapturePointVelocity)
   {
      currentCapturePointPosition.checkReferenceFrameMatch(desiredCapturePointPosition.getReferenceFrame());
      desiredCapturePointVelocity.checkReferenceFrameMatch(desiredCapturePointPosition.getReferenceFrame());
      
      double desiredCapturePointVelocityMagnitude = Math
            .sqrt(desiredCapturePointVelocity.getX() * desiredCapturePointVelocity.getX() + desiredCapturePointVelocity.getY()
                  * desiredCapturePointVelocity.getY() + desiredCapturePointVelocity.getZ() * desiredCapturePointVelocity.getZ());

      if(desiredCapturePointVelocityMagnitude == 0.0)
      {
         return Double.NaN;
      }
      else
      {
         double normalizedCapturePointVelocityX = desiredCapturePointVelocity.getX() / desiredCapturePointVelocityMagnitude;
         double normalizedCapturePointVelocityY = desiredCapturePointVelocity.getY() / desiredCapturePointVelocityMagnitude;
         double normalizedCapturePointVelocityZ = desiredCapturePointVelocity.getZ() / desiredCapturePointVelocityMagnitude;
         
   
         double capturePointErrorX = currentCapturePointPosition.getX() - desiredCapturePointPosition.getX();
         double capturePointErrorY = currentCapturePointPosition.getY() - desiredCapturePointPosition.getY();
         double capturePointErrorZ = currentCapturePointPosition.getZ() - desiredCapturePointPosition.getZ();
   
         return -(normalizedCapturePointVelocityX * capturePointErrorX + normalizedCapturePointVelocityY * capturePointErrorY + normalizedCapturePointVelocityZ
               * capturePointErrorZ);
      }
   }
}
