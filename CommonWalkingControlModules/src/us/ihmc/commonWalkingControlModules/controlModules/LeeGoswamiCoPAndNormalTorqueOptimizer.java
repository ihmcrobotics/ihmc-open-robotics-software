package us.ihmc.commonWalkingControlModules.controlModules;

import java.util.Arrays;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.commonWalkingControlModules.controlModules.nativeOptimization.LeeGoswamiCoPAndNormalTorqueOptimizerNative;
import us.ihmc.utilities.exeptions.NoConvergenceException;
import us.ihmc.utilities.math.MathTools;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.ReferenceFrame;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;


public class LeeGoswamiCoPAndNormalTorqueOptimizer
{
   private static final int VECTOR3D_LENGTH = 3;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final DoubleYoVariable epsilonCoP = new DoubleYoVariable("epsilonCoP", registry);    // TODO: better name
   private final DoubleYoVariable epsilonTauN = new DoubleYoVariable("epsilonTauN", registry);
   private final DoubleYoVariable copAndNormalTorqueOptimalValue = new DoubleYoVariable("copAndNormalTorqueOptimalValue", registry);
   private final BooleanYoVariable converged = new BooleanYoVariable("copAndNormalTorqueConverged", registry);

   private final ReferenceFrame centerOfMassFrame;

   private final double[] psik;
   private final double[] kappaK;
   private final double[] etaMin;
   private final double[] etaMax;
   private final double[] etaD;
   private final double[] epsilon;

   private final Matrix3d tempMatrix = new Matrix3d();

   private LeeGoswamiCoPAndNormalTorqueOptimizerNative leeGoswamiCoPAndNormalTorqueOptimizerNative;



   public LeeGoswamiCoPAndNormalTorqueOptimizer(ReferenceFrame centerOfMassFrame, YoVariableRegistry parentRegistry)
   {
      this.centerOfMassFrame = centerOfMassFrame;
      parentRegistry.addChild(registry);

      int nRows = VECTOR3D_LENGTH;
      int nColumns = LeeGoswamiCoPAndNormalTorqueOptimizerNative.MAX_NUMBER_OF_CONTACTS * VECTOR3D_LENGTH;

      psik = new double[nRows * nColumns];
      kappaK = new double[nRows];
      etaMin = new double[nColumns];
      etaMax = new double[nColumns];
      etaD = new double[nColumns];
      epsilon = new double[nColumns];

      leeGoswamiCoPAndNormalTorqueOptimizerNative = new LeeGoswamiCoPAndNormalTorqueOptimizerNative(VECTOR3D_LENGTH);
   }

   public void setEpsilonCoP(double epsilonCoP)
   {
      this.epsilonCoP.set(epsilonCoP);
   }

   public void setEpsilonTauN(double epsilonTauN)
   {
      this.epsilonTauN.set(epsilonTauN);
   }

   // TODO: assumes that ankle is directly above origin of planeFrame for a contact state
   public void solve(LinkedHashMap<PlaneContactState, FramePoint2d> centersOfPressure, LinkedHashMap<PlaneContactState, Double> normalTorques,
                     HashMap<PlaneContactState, Double> rotationalCoefficientsOfFriction, Vector3d desiredNetTorque,
                     HashMap<PlaneContactState, FrameVector> forces)
   {
      // psiK, kappaK
      Arrays.fill(psik, 0.0);
      MatrixTools.tuple3dToArray(desiredNetTorque, kappaK, 0);

      // TODO: garbage
      DenseMatrix64F a = new DenseMatrix64F(VECTOR3D_LENGTH, VECTOR3D_LENGTH);
      DenseMatrix64F skew = new DenseMatrix64F(VECTOR3D_LENGTH, VECTOR3D_LENGTH);
      FramePoint ankle = new FramePoint(ReferenceFrame.getWorldFrame());
      Vector3d tempKappaVector = new Vector3d();

      int startIndex = 0;
      for (PlaneContactState contactState : centersOfPressure.keySet())
      {
         FrameVector force = forces.get(contactState);
         force.changeFrame(centerOfMassFrame);

         MatrixTools.vectorToSkewSymmetricMatrix(skew, force.getVector());

         RigidBodyTransform transform = contactState.getFrameAfterParentJoint().getTransformToDesiredFrame(centerOfMassFrame);
         transform.get(tempMatrix);
         DenseMatrix64F rotationMatrix = new DenseMatrix64F(VECTOR3D_LENGTH, VECTOR3D_LENGTH);
         MatrixTools.setDenseMatrixFromMatrix3d(0, 0, tempMatrix, rotationMatrix);

         CommonOps.mult(-1.0, skew, rotationMatrix, a);

         // update psiK
         startIndex += MatrixTools.denseMatrixToArrayColumnMajor(a, 0, 0, a.getNumRows(), 2, psik, startIndex);    // first two columns of A
         startIndex += MatrixTools.denseMatrixToArrayColumnMajor(rotationMatrix, 0, 2, rotationMatrix.getNumRows(), 1, psik, startIndex);    // last column of R

         // update kappaK
         ankle.setToZero(contactState.getFrameAfterParentJoint());
         ankle.changeFrame(contactState.getPlaneFrame());
         double ankleHeight = ankle.getZ();

         MatrixTools.denseMatrixToVector3d(a, tempKappaVector, 0, 2);
         tempKappaVector.scale(ankleHeight);    // last column of A times h
         desiredNetTorque.add(tempKappaVector);
         kappaK[0] += tempKappaVector.getX();
         kappaK[1] += tempKappaVector.getY();
         kappaK[2] += tempKappaVector.getZ();
      }

      // etaMin, etaMax (assumes rectangular feet)
      int startRow = 0;
      Arrays.fill(etaMin, 0.0);
      Arrays.fill(etaMax, 0.0);
      Arrays.fill(etaD, 0.0);
      for (PlaneContactState contactState : centersOfPressure.keySet())
      {
         FrameVector force = forces.get(contactState);
         force.changeFrame(contactState.getPlaneFrame());
         double normalForce = force.getZ();
         double tauMax = rotationalCoefficientsOfFriction.get(contactState) * normalForce;

         List<FramePoint2d> contactPoints = contactState.getContactFramePoints2dInContactCopy();
         double xMin = Double.POSITIVE_INFINITY;
         double xMax = Double.NEGATIVE_INFINITY;
         double yMin = Double.POSITIVE_INFINITY;
         double yMax = Double.NEGATIVE_INFINITY;

         for (FramePoint2d contactPoint : contactPoints)
         {
            contactPoint.changeFrame(contactState.getPlaneFrame());
            double x = contactPoint.getX();
            double y = contactPoint.getY();
            if (x < xMin)
               xMin = x;
            if (x > xMax)
               xMax = x;
            if (y < yMin)
               yMin = y;
            if (y > yMax)
               yMax = y;
         }

         if (Double.isInfinite(xMax))
            throw new RuntimeException();
         
         etaMin[startRow + 0] = xMin;
         etaMin[startRow + 1] = yMin;
         etaMin[startRow + 2] = -tauMax;

         etaMax[startRow + 0] = xMax;
         etaMax[startRow + 1] = yMax;
         etaMax[startRow + 2] = tauMax;

         startRow += VECTOR3D_LENGTH;
      }

      // etad (average of min and max for now; could change later
      for (int i = 0; i < etaMin.length; i++)
      {
         etaD[i] = (etaMin[i] + etaMax[i]) * 0.5;
      }

      // epsilon
      int epsilonIndex = 0;
      double epsilonCoPSquared = MathTools.square(epsilonCoP.getDoubleValue());
      double epsilonTauNSquared = MathTools.square(epsilonTauN.getDoubleValue());
      for (int i = 0; i < centersOfPressure.size(); i++)
      {
         epsilon[epsilonIndex++] = epsilonCoPSquared;
         epsilon[epsilonIndex++] = epsilonCoPSquared;
         epsilon[epsilonIndex++] = epsilonTauNSquared;
      }

      try
      {
         leeGoswamiCoPAndNormalTorqueOptimizerNative.solve(psik, kappaK, etaMin, etaMax, etaD, epsilon);
         converged.set(true);
      }
      catch (NoConvergenceException e)
      {
         converged.set(false);
      }

      double[] eta = leeGoswamiCoPAndNormalTorqueOptimizerNative.getEta();
      this.copAndNormalTorqueOptimalValue.set(leeGoswamiCoPAndNormalTorqueOptimizerNative.getOptval());

      int etaIndex = 0;
      for (PlaneContactState contactState : centersOfPressure.keySet())
      {
         FramePoint2d cop = centersOfPressure.get(contactState);
         cop.setX(eta[etaIndex++]);
         cop.setY(eta[etaIndex++]);
         normalTorques.put(contactState, eta[etaIndex++]);
      }
   }
}
