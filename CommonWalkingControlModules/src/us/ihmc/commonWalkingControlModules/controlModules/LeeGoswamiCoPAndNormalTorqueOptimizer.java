package us.ihmc.commonWalkingControlModules.controlModules;

import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;

import javax.media.j3d.Transform3D;
import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
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
   
   private final DenseMatrix64F psik = new DenseMatrix64F(0);
   private final DenseMatrix64F kappaK = new DenseMatrix64F(0);
   private final DenseMatrix64F etaMin = new DenseMatrix64F(0);
   private final DenseMatrix64F etaMax = new DenseMatrix64F(0);
   private final DenseMatrix64F etaD = new DenseMatrix64F(0);

   private final Matrix3d tempMatrix = new Matrix3d();

   public LeeGoswamiCoPAndNormalTorqueOptimizer(ReferenceFrame centerOfMassFrame, YoVariableRegistry parentRegistry)
   {
      this.centerOfMassFrame = centerOfMassFrame;
      parentRegistry.addChild(registry);
   }
   
   private void reshape(int nContacts)
   {
      psik.reshape(VECTOR3D_LENGTH, nContacts * VECTOR3D_LENGTH);
      kappaK.reshape(VECTOR3D_LENGTH, 1);
      etaMin.reshape(nContacts * VECTOR3D_LENGTH, 1);
      etaMax.reshape(nContacts * VECTOR3D_LENGTH, 1);
      etaD.reshape(nContacts * VECTOR3D_LENGTH, 1);
   }

   // TODO: assumes that ankle is directly above origin of planeFrame for a contact state
   public void solve(LinkedHashMap<PlaneContactState, FramePoint2d> centersOfPressure, HashMap<PlaneContactState, Double> normalTorques,
           HashMap<PlaneContactState, Double> rotationalCoefficientsOfFriction, Vector3d desiredNetTorque, HashMap<PlaneContactState, FrameVector> forces)
   {
      reshape(centersOfPressure.size());
      
      // psiK, kappaK
      MatrixTools.setDenseMatrixFromTuple3d(kappaK, desiredNetTorque, 0, 0);
      int startColumn = 0;

      // TODO: garbage
      DenseMatrix64F a = new DenseMatrix64F(VECTOR3D_LENGTH, VECTOR3D_LENGTH);
      DenseMatrix64F skew = new DenseMatrix64F(VECTOR3D_LENGTH, VECTOR3D_LENGTH);
      FramePoint ankle = new FramePoint(ReferenceFrame.getWorldFrame());

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
         CommonOps.extract(a, 0, a.getNumRows(), 0, 2, psik, 0, startColumn);    // first two columns of A
         CommonOps.extract(rotationMatrix, 0, rotationMatrix.getNumRows(), 2, 3, psik, 0, startColumn + 2);    // last column of R
         startColumn += VECTOR3D_LENGTH;

         // update kappaK
         ankle.setToZero(contactState.getBodyFrame());
         ankle.changeFrame(contactState.getPlaneFrame());
         double ankleHeight = ankle.getZ();
         MatrixTools.addMatrixBlock(kappaK, 0, 0, a, 0, 2, a.getNumRows(), 1, ankleHeight);    // last column of A times h
      }

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

         etaMin.set(startRow + 0, 0, xMin);
         etaMin.set(startRow + 1, 0, yMin);
         etaMin.set(startRow + 2, 0, -tauMax);

         etaMax.set(startRow + 0, 0, xMax);
         etaMax.set(startRow + 1, 0, yMax);
         etaMax.set(startRow + 2, 0, tauMax);

         startRow += VECTOR3D_LENGTH;
      }

      // etad (average of min and max for now; could change later
      CommonOps.add(0.5, etaMin, etaMax, etaD);

      double epsilonCoP = this.epsilonCoP.getDoubleValue();
      double epsilonTauN = this.epsilonTauN.getDoubleValue();

      // TODO: call CoP solver with Psi, kappa, etaMin, etaMax, etad, and epsilonP
   }
}
