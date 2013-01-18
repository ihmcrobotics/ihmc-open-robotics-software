package us.ihmc.commonWalkingControlModules.controlModules;

import java.util.Arrays;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;

import javax.media.j3d.Transform3D;
import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.commonWalkingControlModules.controlModules.nativeOptimization.LeeGoswamiCoPAndNormalTorqueOptimizerNative;
import us.ihmc.utilities.math.MatrixTools;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;

public class LeeGoswamiCoPAndNormalTorqueOptimizer
{
   private static final int VECTOR3D_LENGTH = 3;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final DoubleYoVariable epsilonCoP = new DoubleYoVariable("epsilonCoP", registry);    // TODO: better name
   private final DoubleYoVariable epsilonTauN = new DoubleYoVariable("epsilonTauN", registry);

   private final ReferenceFrame centerOfMassFrame;

   private final double[] psik;
   private final double[] kappaK;
   private final double[] etaMin;
   private final double[] etaMax;
   private final double[] etaD;

   private final Matrix3d tempMatrix = new Matrix3d();

   private LeeGoswamiCoPAndNormalTorqueOptimizerNative leeGoswamiCoPAndNormalTorqueOptimizerNative;

   public LeeGoswamiCoPAndNormalTorqueOptimizer(ReferenceFrame centerOfMassFrame, YoVariableRegistry parentRegistry)
   {
      this.centerOfMassFrame = centerOfMassFrame;
      parentRegistry.addChild(registry);

      int maxNContacts = 2;
      int nRows = VECTOR3D_LENGTH;
      int nColumns = maxNContacts * VECTOR3D_LENGTH;

      psik = new double[nRows * nColumns];
      kappaK = new double[nRows];
      etaMin = new double[nColumns];
      etaMax = new double[nColumns];
      etaD = new double[nColumns];
   }

   // TODO: assumes that ankle is directly above origin of planeFrame for a contact state
   public void solve(LinkedHashMap<PlaneContactState, FramePoint2d> centersOfPressure, LinkedHashMap<PlaneContactState, Double> normalTorques,
                     HashMap<PlaneContactState, Double> rotationalCoefficientsOfFriction, Vector3d desiredNetTorque,
                     HashMap<PlaneContactState, FrameVector> forces)
   {
      // psiK, kappaK
      Arrays.fill(psik, 0.0);
      Arrays.fill(kappaK, 0.0);

      // TODO: garbage
      DenseMatrix64F a = new DenseMatrix64F(VECTOR3D_LENGTH, VECTOR3D_LENGTH);
      DenseMatrix64F skew = new DenseMatrix64F(VECTOR3D_LENGTH, VECTOR3D_LENGTH);
      FramePoint ankle = new FramePoint(ReferenceFrame.getWorldFrame());
      Vector3d tempKappa = new Vector3d();

      int startIndex = 0;
      for (PlaneContactState contactState : centersOfPressure.keySet())
      {
         FrameVector force = forces.get(contactState);
         force.changeFrame(centerOfMassFrame);

         MatrixTools.vectorToSkewSymmetricMatrix(skew, force.getVector());

         Transform3D transform = contactState.getBodyFrame().getTransformToDesiredFrame(centerOfMassFrame);
         transform.get(tempMatrix);
         DenseMatrix64F rotationMatrix = new DenseMatrix64F(VECTOR3D_LENGTH, VECTOR3D_LENGTH);
         MatrixTools.setDenseMatrixFromMatrix3d(0, 0, tempMatrix, rotationMatrix);

         CommonOps.mult(-1.0, skew, rotationMatrix, a);

         // update psiK
         startIndex += MatrixTools.denseMatrixToArrayColumnMajor(a, 0, 0, a.getNumRows(), 2, psik, startIndex);    // first two columns of A
         startIndex += MatrixTools.denseMatrixToArrayColumnMajor(rotationMatrix, 0, 2, rotationMatrix.getNumRows(), 1, psik, startIndex);    // last column of R

         // update kappaK
         ankle.setToZero(contactState.getBodyFrame());
         ankle.changeFrame(contactState.getPlaneFrame());
         double ankleHeight = ankle.getZ();

         MatrixTools.denseMatrixToVector3d(a, tempKappa, 0, 2);
         tempKappa.scale(ankleHeight);    // last column of A times h
         desiredNetTorque.add(tempKappa);
      }

      MatrixTools.tuple3dToArray(desiredNetTorque, kappaK, 0);

      // etaMin, etaMax (assumes rectangular feet)
      int startRow = 0;
      for (PlaneContactState contactState : centersOfPressure.keySet())
      {
         FrameVector force = forces.get(contactState);
         force.changeFrame(contactState.getPlaneFrame());
         double normalForce = force.getZ();
         double tauMax = rotationalCoefficientsOfFriction.get(contactState) * normalForce;

         List<FramePoint2d> contactPoints = contactState.getContactPoints2d();
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

      double epsilonCoP = this.epsilonCoP.getDoubleValue();
      double epsilonTauN = this.epsilonTauN.getDoubleValue();

//      double[] copsAndNormalTorques = leeGoswamiCoPAndNormalTorqueOptimizerNative.solve(psik, kappaK, etaMin, etaMax, epsilonCoP, epsilonTauN);

//      int index = 0;
//      for (PlaneContactState contactState : centersOfPressure.keySet())
//      {
//         FramePoint2d cop = centersOfPressure.get(contactState);
//         cop.setX(copsAndNormalTorques[index++]);
//         cop.setY(copsAndNormalTorques[index++]);
//         normalTorques.put(contactState, copsAndNormalTorques[index++]);
//      }
   }
}
