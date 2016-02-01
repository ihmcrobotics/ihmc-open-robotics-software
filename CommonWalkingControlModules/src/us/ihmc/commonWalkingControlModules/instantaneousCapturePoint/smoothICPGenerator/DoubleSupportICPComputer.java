package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator;

import java.util.ArrayList;

import javax.vecmath.Point3d;
import javax.vecmath.Tuple3d;
import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;


public class DoubleSupportICPComputer
{
   private FramePoint tempFramePointI = new FramePoint();
   private FramePoint tempFramePointIminus1 = new FramePoint();

   private final DenseMatrix64F doubleSupportParameterMatrix = new DenseMatrix64F(3, 4);

   private final Point3d desiredDCMposOfTime = new Point3d();
   private final Vector3d desiredDCMvelOfTime = new Vector3d();
   private final Point3d desiredECMPofTime = new Point3d();

   private final Point3d initialDoubleSupportICPpos = new Point3d();
   private final Vector3d initialDoubleSupportICPvel = new Vector3d();
   private final Point3d finalDoubleSupportICPpos = new Point3d();
   private final Vector3d finalDoubleSupportICPvel = new Vector3d();


   private boolean finalStepReached;
   private int finalCounter;


   public DoubleSupportICPComputer(YoVariableRegistry registryExt)
   {
//    tempFramePointI.set("pointTemp1","",ReferenceFrame.getWorldFrame(), registry);
//    tempFramePointIminus1 =  new YoFramePoint("pointTemp2","",ReferenceFrame.getWorldFrame(), registry);

      finalStepReached = false;
      finalCounter = 0;

   }

   public Point3d getInitialDoubleSupportICPpos()
   {
      return initialDoubleSupportICPpos;
   }

// public DenseMatrix64F getInitialDoubleSupportICPvel()
// {
//    return initialDoubleSupportICPvel;
// }

   public Point3d getFinalDoubleSupportICPpos()
   {
      return finalDoubleSupportICPpos;
   }

// public DenseMatrix64F getFinalDoubleSupportICPvel()
// {
//    return finalDoubleSupportICPvel;
// }

   public Point3d getDesiredDCMposOfTime()
   {
      return desiredDCMposOfTime;
   }

   public Vector3d getDesiredDCMvelOfTime()
   {
      return desiredDCMvelOfTime;
   }

   public Point3d getDesiredECMPofTime()
   {
      return desiredECMPofTime;
   }

// public DenseMatrix64F getPolynomialParamMatrix()
// {
//    return paramMatrix;
// }

// public DenseMatrix64F getPolynomialParamMatrixColumn(int colIndex)
// {
//    return EnhancedMatrixManipulator.getMatrixColumn(colIndex, paramMatrix);
// }

   public void reset()
   {
      finalStepReached = false;
      finalCounter = 0;

   }

   private void updateDCMCornerPoints(ArrayList<Point3d> initialICPsToPack, ArrayList<Point3d> constantEquivalentCoPs, double omega0,
                                      double steppingTime)
   {
      int initialICPsSize = initialICPsToPack.size();

      initialICPsToPack.get(initialICPsSize - 1).set(JojosICPutilities.extrapolateDCMpos(constantEquivalentCoPs.get(initialICPsSize - 1), -steppingTime, omega0,
            constantEquivalentCoPs.get(initialICPsSize)));

      for (int i = initialICPsToPack.size() - 1; i > 0; i--)
      {
         initialICPsToPack.get(i - 1).set(JojosICPutilities.extrapolateDCMpos(constantEquivalentCoPs.get(i - 1), -steppingTime, omega0, initialICPsToPack.get(i)));
      }
   }


   public void computeDoubleSupportPolynomialParams(ArrayList<Point3d> constantEquivalentCoPs, ArrayList<YoFramePoint> consideredFootStepLocationsFramePoints, double omega0, double steppingTime,
           double doubleSupportFirstStepFraction, ArrayList<Point3d> initialICPs, boolean isFirstStep, double initialTransferSupportTime,
           double doubleSupportTime)
   {
      double doubleSupportDuration;

      if (isFirstStep)
      {
         doubleSupportDuration = initialTransferSupportTime;
      }
      else
      {
         doubleSupportDuration = doubleSupportTime;
      }


      double doubleSupportTimeCurrentStep = -doubleSupportFirstStepFraction * doubleSupportDuration;
      double doubleSupportTimeNextStep = (1 - doubleSupportFirstStepFraction) * doubleSupportTime;
      

      updateDCMCornerPoints(initialICPs, constantEquivalentCoPs, omega0, steppingTime);

      // Calculate DCM position and velocity at beginning of Double Support phase

      if (isFirstStep)    // (false) //
      {
//         FramePoint tempFramePointI = new FramePoint(ReferenceFrame.getWorldFrame());
//         FramePoint tempFramePointIminus1 = new FramePoint(ReferenceFrame.getWorldFrame());
//
//
//         tempFramePointI.set(constantEquivalentCoPs.get(0).get(0), constantEquivalentCoPs.get(0).get(1), constantEquivalentCoPs.get(0).get(2));
//
//         tempFramePointIminus1.set(constantEquivalentCoPs.get(1).get(0), constantEquivalentCoPs.get(1).get(1), constantEquivalentCoPs.get(1).get(2));
//         tempFramePointI.add(tempFramePointIminus1);
//         tempFramePointI.scale(0.5);
         
         
         FramePoint tempFramePointI =  new FramePoint(ReferenceFrame.getWorldFrame());
         FramePoint tempFramePointIminus1 =  new FramePoint(ReferenceFrame.getWorldFrame());
         
         
         tempFramePointI.set(consideredFootStepLocationsFramePoints.get(0).getX(), consideredFootStepLocationsFramePoints.get(0).getY(), consideredFootStepLocationsFramePoints.get(0).getZ());  
         
         tempFramePointIminus1.set(consideredFootStepLocationsFramePoints.get(1).getX(), consideredFootStepLocationsFramePoints.get(1).getY(), consideredFootStepLocationsFramePoints.get(1).getZ());
         tempFramePointI.add(tempFramePointIminus1);
         tempFramePointI.scale(0.5);

         initialDoubleSupportICPpos.set(tempFramePointI.getPoint());
         initialDoubleSupportICPvel.set(0.0, 0.0, 0.0);
      }
      else
      {
         
         JojosICPutilities.extrapolateDCMposAndVel(initialDoubleSupportICPpos, initialDoubleSupportICPvel, 
               constantEquivalentCoPs.get(0), steppingTime + doubleSupportTimeCurrentStep, omega0, initialICPs.get(0));
      }



      // Calculate DCM position and velocity at end of Double Support phase

      JojosICPutilities.extrapolateDCMposAndVel(finalDoubleSupportICPpos, finalDoubleSupportICPvel, 
            constantEquivalentCoPs.get(1), doubleSupportTimeNextStep, omega0, initialICPs.get(1));
      
      computeThirdOrderPolynomialParameterMatrix(doubleSupportParameterMatrix, doubleSupportDuration, initialDoubleSupportICPpos, initialDoubleSupportICPvel, finalDoubleSupportICPpos, finalDoubleSupportICPvel);
   }
   
   
   public static void computeThirdOrderPolynomialParameterMatrix(DenseMatrix64F parameterMatrixToPack, double doubleSupportDuration, Point3d initialDoubleSupportICPpos,
   Vector3d initialDoubleSupportICPvel, Point3d finalDoubleSupportICPpos, Vector3d finalDoubleSupportICPvel)
   {
      double doubleSupportTimePow2 = Math.pow(doubleSupportDuration, 2);
      double doubleSupportTimePow3 = Math.pow(doubleSupportDuration, 3);
      
      // Calculate time-dependency matrix for polynomial calculation (part of inversion problem)
      DenseMatrix64F TimeBoundaryConditionMatrix = new DenseMatrix64F(4, 4, true, -2.0, 1.0, 2.0, 1.0, 3.0 * doubleSupportDuration, -2.0 * doubleSupportDuration,
                                                      -3.0 * doubleSupportDuration, -doubleSupportDuration, 0.0, doubleSupportTimePow2, 0.0, 0.0,
                                                      -doubleSupportTimePow3, 0.0, 0.0, 0.0);

      // TimeBoundaryConditionMatrix.print();

      // Base = [                -2,         1,          2,              1; ...
      // 3*t2,       -2*t2,      -3*t2,     -t2; ...
      // 0,          t2_2,       0,         0; ...
      // -t2_3,      0,          0,       0];

      // Calculate DenominatorMatrix, so that T =  TimeBoundaryConditionMatrix*DenominatorMatrix, so that PolynomialParams = T*StateBoundaryConditionMatrix
      double denominator1 = -1.0 / doubleSupportTimePow3;
      double denominator2 = 1.0 / doubleSupportTimePow2;
      DenseMatrix64F DenominatorMatrix = CommonOps.diag(denominator1, denominator2, denominator1, denominator2);

      // System.out.println("DenominatorMatrix: ");
      // DenominatorMatrix.print();

      // Calculate PolynomialParams via inversion (see description above)
      DenseMatrix64F StateBoundaryConditionMatrix = new DenseMatrix64F(4, 3);
      EnhancedMatrixManipulator.setMatrixRowToTuple3d(0, StateBoundaryConditionMatrix, initialDoubleSupportICPpos);
      EnhancedMatrixManipulator.setMatrixRowToTuple3d(1, StateBoundaryConditionMatrix, initialDoubleSupportICPvel);
      EnhancedMatrixManipulator.setMatrixRowToTuple3d(2, StateBoundaryConditionMatrix, finalDoubleSupportICPpos);
      EnhancedMatrixManipulator.setMatrixRowToTuple3d(3, StateBoundaryConditionMatrix, finalDoubleSupportICPvel);

      
      
//      EnhancedMatrixManipulator.setMatrixRowToVector(0, StateBoundaryConditionMatrix, initialDoubleSupportICPpos);
//      EnhancedMatrixManipulator.setMatrixRowToVector(1, StateBoundaryConditionMatrix, initialDoubleSupportICPvel);
//      EnhancedMatrixManipulator.setMatrixRowToVector(2, StateBoundaryConditionMatrix, finalDoubleSupportICPpos);
//      EnhancedMatrixManipulator.setMatrixRowToVector(3, StateBoundaryConditionMatrix, finalDoubleSupportICPvel);

      // System.out.println("StateBoundaryConditionMatrix: ");
      // StateBoundaryConditionMatrix.print();

      DenseMatrix64F tempResultMatrix = new DenseMatrix64F(4, 3);
      DenseMatrix64F tempParamMatrix = new DenseMatrix64F(4, 3);
      CommonOps.mult(DenominatorMatrix, StateBoundaryConditionMatrix, tempResultMatrix);
      CommonOps.mult(TimeBoundaryConditionMatrix, tempResultMatrix, tempParamMatrix);
      CommonOps.transpose(tempParamMatrix);
      parameterMatrixToPack.set(tempParamMatrix);
   }

   public void calcDCMandECMPofTime(ArrayList<Point3d> constantEquivalentCoPs, ArrayList<YoFramePoint> consideredFootStepLocationsFramePoints, double doubleSupportFirstStepFraction, double omega0,
                                    ArrayList<Point3d> initialICPs, boolean isFirstStep, double initialTransferSupportTime, double doubleSupportTime,
                                    boolean isSingleSupport, double currentTime, double steppingTime)
   {
//    double currentTime = supportState.getCurrentTime();
//    double steppingTime = supportState.getSteppingTime();


      if (isSingleSupport)
      {
         double singleSupportComputationTime = currentTime + (1 - doubleSupportFirstStepFraction) * doubleSupportTime;
         JojosICPutilities.extrapolateDCMposAndVel(desiredDCMposOfTime, desiredDCMvelOfTime, constantEquivalentCoPs.get(0), singleSupportComputationTime, omega0, initialICPs.get(0));
         desiredECMPofTime.set(constantEquivalentCoPs.get(0));
      }
      else
      {
         computeDoubleSupportPolynomialParams(constantEquivalentCoPs, consideredFootStepLocationsFramePoints, omega0, steppingTime, doubleSupportFirstStepFraction, initialICPs, isFirstStep,
                 initialTransferSupportTime, doubleSupportTime);

         double timePow3 = Math.pow(currentTime, 3.0);
         double timePow2 = Math.pow(currentTime, 2.0);
         Vector3d tempVector = new Vector3d();

         DenseMatrix64F dcmPositionTimeVector = new DenseMatrix64F(4, 1, true, timePow3, timePow2, currentTime, 1);
         DenseMatrix64F dcmVelocityTimeVector = new DenseMatrix64F(4, 1, true, 3 * timePow2, 2 * currentTime, 1, 0);

         
         multiplyMatricesAndPutInPoint3d(desiredDCMposOfTime, doubleSupportParameterMatrix, dcmPositionTimeVector);
         multiplyMatricesAndPutInPoint3d(desiredDCMvelOfTime, doubleSupportParameterMatrix, dcmVelocityTimeVector);
         tempVector.set(desiredDCMvelOfTime);
         tempVector.scale(-1.0/omega0);
         desiredECMPofTime.set(desiredDCMposOfTime);
         desiredECMPofTime.add(tempVector);
      }
   }
   
   private void multiplyMatricesAndPutInPoint3d(Tuple3d tuple3dToPack, DenseMatrix64F matrixOne, DenseMatrix64F matrixTwo)
   {
      DenseMatrix64F tempMatrix = new DenseMatrix64F(3, 1);
      CommonOps.mult(matrixOne, matrixTwo, tempMatrix);
      
      if (tempMatrix.getNumCols() != 1) throw new RuntimeException("tempMatrix.getNumCols() != 1");
      if (tempMatrix.getNumRows() != 3) throw new RuntimeException("tempMatrix.getNumRows() != 3");
      
      tuple3dToPack.setX(tempMatrix.get(0, 0));
      tuple3dToPack.setY(tempMatrix.get(1, 0));
      tuple3dToPack.setZ(tempMatrix.get(2, 0));
   }

   public void updateSubFootListForSmoothICPTrajectory(ArrayList<YoFramePoint> footStepLocationsFramePoints,
           ArrayList<YoFramePoint> equivalentConstantCoPsFramePoints, ArrayList<YoFramePoint> consideredFootStepLocationsFramePoints,
           int numberOfConsideredFootstepLocations, ArrayList<Point3d> equivalentConstantCoPsVectors, boolean isFirstStep)
   {
      int footListSize = footStepLocationsFramePoints.size();

      if (!(footListSize > 2))
      {
         finalCounter += 1;
      }

      if (finalCounter >= 2)
      {
         finalStepReached = true;
      }
      else
         finalStepReached = false;

      ArrayList<Integer> footListSelectionIndices = new ArrayList<Integer>();

      footListSelectionIndices.add(0);

      if ((footListSize <= 2) && finalStepReached)
      {
         tempFramePointI.set(footStepLocationsFramePoints.get(0).getFramePointCopy());

         tempFramePointIminus1.set(footStepLocationsFramePoints.get(1).getFramePointCopy());
         tempFramePointI.add(tempFramePointIminus1);
         tempFramePointI.scale(0.5);


         equivalentConstantCoPsFramePoints.get(0).set(tempFramePointI);

         consideredFootStepLocationsFramePoints.get(0).set(footStepLocationsFramePoints.get(0));

      }
      else
      {
//         equivalentConstantCoPsFramePoints.get(0).set(footStepLocationsFramePoints.get(0));
         
         if (isFirstStep)
         {
            tempFramePointI.set(footStepLocationsFramePoints.get(0).getFramePointCopy());  
            
            tempFramePointIminus1.set(footStepLocationsFramePoints.get(1).getFramePointCopy());
            tempFramePointI.add(tempFramePointIminus1);
            tempFramePointI.scale(0.5);
            
            
            equivalentConstantCoPsFramePoints.get(0).set(tempFramePointI);
         }
         else
         {
            equivalentConstantCoPsFramePoints.get(0).set(footStepLocationsFramePoints.get(0));
         }

         consideredFootStepLocationsFramePoints.get(0).set(footStepLocationsFramePoints.get(0));
      }

      boolean endWasReached = false;

      for (int i = 1; i < numberOfConsideredFootstepLocations; i++)
      {
         footListSelectionIndices.add(footListSelectionIndices.get(i - 1) + 1);

         if (endWasReached)
         {
            equivalentConstantCoPsFramePoints.get(i).set(equivalentConstantCoPsFramePoints.get(i - 1));

            consideredFootStepLocationsFramePoints.get(i).set(footStepLocationsFramePoints.get(footListSize - 1));
         }
         else if (i + 1 >= footListSize)
         {
            endWasReached = true;

            tempFramePointI.set(footStepLocationsFramePoints.get(i).getFramePointCopy());

            tempFramePointIminus1.set(footStepLocationsFramePoints.get(i - 1).getFramePointCopy());
            tempFramePointI.add(tempFramePointIminus1);
            tempFramePointI.scale(0.5);

            equivalentConstantCoPsFramePoints.get(i).set(tempFramePointI);

            consideredFootStepLocationsFramePoints.get(i).set(footStepLocationsFramePoints.get(footListSize - 1));
         }
         else
         {
            equivalentConstantCoPsFramePoints.get(i).set(footStepLocationsFramePoints.get(i));

            consideredFootStepLocationsFramePoints.get(i).set(footStepLocationsFramePoints.get(i));
         }
      }

      for (int i = 0; i < numberOfConsideredFootstepLocations; i++)
      {
         Point3d point3d = equivalentConstantCoPsVectors.get(i);
         point3d.set(equivalentConstantCoPsFramePoints.get(i).getFramePointCopy().getPoint());
      }

      if (footListSize > 2)
      {
         footStepLocationsFramePoints.remove(0);
      }
   }
}
