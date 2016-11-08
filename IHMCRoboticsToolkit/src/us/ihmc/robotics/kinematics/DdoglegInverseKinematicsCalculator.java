package us.ihmc.robotics.kinematics;

import georegression.geometry.ConvertRotation3D_F64;
import georegression.metric.UtilAngle;
import georegression.struct.EulerType;
import org.ddogleg.optimization.FactoryOptimization;
import org.ddogleg.optimization.RegionStepType;
import org.ddogleg.optimization.UnconstrainedLeastSquares;
import org.ddogleg.optimization.functions.FunctionNtoM;
import org.ejml.data.DenseMatrix64F;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.GeometricJacobian;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.ScrewTools;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;
import java.util.Random;

/**
 * Solver for inverse kinematics which uses DDogleg and Twan's code for forward kinematics.
 *
 * @author Peter Abeles
 */
public class DdoglegInverseKinematicsCalculator implements InverseKinematicsCalculator
{
   private final ReferenceFrame baseFrame;
   private final ReferenceFrame endEffectorFrame;
   private final OneDoFJoint[] oneDoFJoints;
   private final int maxIterations;
   private final Random random = new Random(1251253L);

   private double errorScalar;

   private final RigidBodyTransform actualTransform = new RigidBodyTransform();
   private final Matrix3d rotationMatrix = new Matrix3d();

   private final Vector3d actualT = new Vector3d();
   private final Vector3d actualR = new Vector3d();

   private final Vector3d desiredT = new Vector3d();
   private final Vector3d desiredR = new Vector3d();
   private final DenseMatrix64F m = new DenseMatrix64F(3, 3);

   private int numberOfIterations;
   private final boolean solveOrientation;

   private final double orientationDiscount;

   // convergence tolerance
   private final double convergeTolerance;

   // initial parameters
   private double originalParam[];

   // tolerances for it returning true or false
   private final double acceptTolLoc;
   private final double acceptTolAngle;
   
   private final double parameterChangePenalty;

   private final double euler[] = new double[3];

   private InverseKinematicsStepListener stepListener = null;
   
   /**
    * @param jacobian
    * @param orientationDiscount How much it discounts orientation by.  0 to 1.0.  Try 0.2
    * @param maxIterations
    * @param solveOrientation
    * @param convergeTolerance Convergence tolerance.  Try 1e-12
    * @param acceptTolLoc  Tolerance for location error.  Try 0.005
    * @param acceptTolAngle Tolerance for angle error in radians.  Try 0.02
    */
   public DdoglegInverseKinematicsCalculator(GeometricJacobian jacobian, double orientationDiscount, int maxIterations, boolean solveOrientation,
           double convergeTolerance, double acceptTolLoc, double acceptTolAngle, double parameterChangePenalty)
   {
      if (jacobian.getJacobianFrame() != jacobian.getEndEffectorFrame())
         throw new RuntimeException("jacobian.getJacobianFrame() != jacobian.getEndEffectorFrame()");
      
      baseFrame = jacobian.getBaseFrame();
      endEffectorFrame = jacobian.getEndEffectorFrame();
      
      this.orientationDiscount = orientationDiscount;
      this.solveOrientation = solveOrientation;
      
      this.oneDoFJoints = ScrewTools.filterJoints(jacobian.getJointsInOrder(), OneDoFJoint.class);
      if (oneDoFJoints.length != jacobian.getJointsInOrder().length)
         throw new RuntimeException("Can currently only handle OneDoFJoints");
      
      this.maxIterations = maxIterations;
      this.acceptTolLoc = acceptTolLoc;
      this.acceptTolAngle = acceptTolAngle;
      this.convergeTolerance = convergeTolerance;
      this.parameterChangePenalty = parameterChangePenalty;
   }

   @Override
   public void attachInverseKinematicsStepListener(InverseKinematicsStepListener stepListener)
   {      
      this.stepListener = stepListener;
   }

   @Override
   public boolean solve(RigidBodyTransform desiredTransform)
   {
      double initParam[] = new double[oneDoFJoints.length];
      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         initParam[i] = oneDoFJoints[i].getQ();
      }
      originalParam = initParam.clone();

      extractTandR(desiredTransform, desiredT, desiredR);
      
      FunctionNtoM func = new FunctionErrors(desiredTransform, parameterChangePenalty);

      UnconstrainedLeastSquares optimizer = FactoryOptimization.leastSquaresTrustRegion(0.1, RegionStepType.DOG_LEG_F, false);

      optimizer.setFunction(func, null);

      double bestParam[] = new double[initParam.length];
      System.arraycopy(initParam, 0, bestParam, 0, bestParam.length);
      double bestScore = Double.MAX_VALUE;

      optimizer.initialize(initParam, 0, convergeTolerance);

      int totalSinceLast = 0;

      for (numberOfIterations = 0; numberOfIterations < maxIterations; numberOfIterations++)
      {
         if (stepListener != null)
         {
            stepListener.didAnInverseKinemticsStep(errorScalar);
         }
         
         boolean done;

         do
         {
            done = optimizer.iterate();
         }
         while (!done &&!optimizer.isUpdated());

         if( optimizer.isUpdated() ) 
         {
            double foundParam[] = optimizer.getParameters();

            if (bestScore > optimizer.getFunctionValue())
            {
               bestScore = optimizer.getFunctionValue();
               System.arraycopy(foundParam, 0, bestParam, 0, bestParam.length);

               if (bestScore <= convergeTolerance)
                  break;
            }
         }

         if (done || (totalSinceLast > 30) || (optimizer.getFunctionValue() <= convergeTolerance))
         {
            totalSinceLast = 0;

            // randomize the parameters a bit
            for (int i = 0; i < initParam.length; i++)
            {
               OneDoFJoint oneDoFJoint = oneDoFJoints[i];
               if (Double.isInfinite(oneDoFJoint.getJointLimitUpper()))
               {
                  initParam[i] = random.nextDouble() * 2 * Math.PI - Math.PI;
               }
               else
               {
                  double delta = oneDoFJoint.getJointLimitUpper() - oneDoFJoint.getJointLimitLower();
                  initParam[i] = random.nextDouble() * delta + oneDoFJoint.getJointLimitLower();
               }
            }

            optimizer.initialize(initParam, 0, convergeTolerance);
         }
         else
         {
            totalSinceLast++;
         }
      }

      numberOfIterations++;
      errorScalar = bestScore;
      updateState(bestParam);

      extractTandR(actualTransform, actualT, actualR);
      
      if (Math.abs(actualT.getX() - desiredT.getX()) > acceptTolLoc)
         return false;
      if (Math.abs(actualT.getY() - desiredT.getY()) > acceptTolLoc)
         return false;
      if (Math.abs(actualT.getZ() - desiredT.getZ()) > acceptTolLoc)
         return false;

      if (UtilAngle.dist(actualR.getX(), desiredR.getX()) > acceptTolAngle)
         return false;
      if (UtilAngle.dist(actualR.getY(), desiredR.getY()) > acceptTolAngle)
         return false;
      if (UtilAngle.dist(actualR.getZ(), desiredR.getZ()) > acceptTolAngle)
         return false;

      return true;
   }

   public int getNumberOfIterations()
   {
      return numberOfIterations;
   }

   public double getErrorScalar()
   {
      return errorScalar;
   }


   private void updateState(double[] parameters)
   {
      for (int i = 0; i < parameters.length; i++)
      {
         OneDoFJoint oneDoFJoint = oneDoFJoints[i];

         // apply constraints to the input parameters
         double newQ = UtilAngle.bound(parameters[i]);
         if (Double.isNaN(newQ))
            continue;
         newQ = parameters[i] = MathTools.clipToMinMax(newQ, oneDoFJoint.getJointLimitLower(), oneDoFJoint.getJointLimitUpper());
         oneDoFJoint.setQ(newQ);
         oneDoFJoint.getFrameAfterJoint().update();
      }
   }



   /**
    * Adds a pentially term for large changes in joint angle.  Designed to minimize the case where the arm makes a huge rotation
    */
   public class FunctionErrors implements FunctionNtoM
   {
      RigidBodyTransform desiredTransform;
      double parameterChangePenalty;


      public FunctionErrors(RigidBodyTransform desiredTransform, double parameterChangePenalty)
      {
         this.parameterChangePenalty=parameterChangePenalty;
         this.desiredTransform = desiredTransform;
      }

      public int getNumOfInputsN()
      {
         return oneDoFJoints.length;
      }

      public int getNumOfOutputsM()
      {
         return originalParam.length + (solveOrientation ? 6 : 3);
      }

      public void process(double[] parameters, double[] functions)
      {
         updateState(parameters);
         int index = 0;
         for( ; index < parameters.length; index++) {
            if(index <3)
               functions[index] = parameterChangePenalty*Math.abs(parameters[index]-originalParam[index]);
            else
               functions[index] = parameterChangePenalty*Math.abs(UtilAngle.minus(parameters[index],originalParam[index]));
         }

         endEffectorFrame.getTransformToDesiredFrame(actualTransform, baseFrame);

         extractTandR(actualTransform, actualT, actualR);

         functions[index+0] = actualT.getX() - desiredT.getX();
         functions[index+1] = actualT.getY() - desiredT.getY();
         functions[index+2] = actualT.getZ() - desiredT.getZ();

         if (solveOrientation)
         {
            functions[index+3] = orientationDiscount * UtilAngle.minus(actualR.getX(), desiredR.getX());
            functions[index+4] = orientationDiscount * UtilAngle.minus(actualR.getY(), desiredR.getY());
            functions[index+5] = orientationDiscount * UtilAngle.minus(actualR.getZ(), desiredR.getZ());
         }
      }
   }


   private void extractTandR(RigidBodyTransform tran, Vector3d T, Vector3d R)
   {
      tran.getTranslation(T);
      tran.getRotation(rotationMatrix);

      MatrixTools.matrix3DToDenseMatrix(rotationMatrix, m, 0, 0);

      ConvertRotation3D_F64.matrixToEuler(m, EulerType.XYZ, euler);
      R.setX(euler[0]);
      R.setY(euler[1]);
      R.setZ(euler[2]);
   }

   @Override
   public void setLimitJointAngles(boolean limitJointAngles)
   {
      // TODO Auto-generated method stub
      
   }


}
