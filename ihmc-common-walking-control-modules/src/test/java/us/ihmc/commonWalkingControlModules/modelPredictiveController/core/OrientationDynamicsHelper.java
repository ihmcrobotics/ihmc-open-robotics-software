package us.ihmc.commonWalkingControlModules.modelPredictiveController.core;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling.MPCContactPlane;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling.MPCContactPoint;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.interfaces.FrameOrientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameTestTools;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.MatrixMissingTools;

import java.util.ArrayList;
import java.util.List;

public class OrientationDynamicsHelper
{
   public static final double gravityZ = -9.81;

   public static FrameVector3DReadOnly computeExpectedAngularVelocityErrorRate(double mass,
                                                                               FramePoint3DReadOnly comPosition,
                                                                               FrameVector3DReadOnly angularErrorAtCurrentTick,
                                                                               FrameVector3DReadOnly angularVelocityErrorAtCurrentTick,
                                                                               Matrix3DReadOnly momentOfInertia,
                                                                               FramePoint3DReadOnly desiredCoMPosition,
                                                                               FrameVector3DReadOnly desiredCoMAcceleration,
                                                                               FrameOrientation3DReadOnly desiredBodyOrientation,
                                                                               Vector3DReadOnly desiredBodyAngularVelocity,
                                                                               Vector3DReadOnly desiredNetAngularMomentumRate,
                                                                               Vector3DReadOnly desiredInternalAngularMomentumRate,
                                                                               List<MPCContactPlane> contactPlanes)
   {
      FrameVector3D angularVelocityErrorRate = new FrameVector3D();
      FrameVector3DReadOnly angularVelocityErrorRateFromAngularError = computeExpectedAngularVelocityErrorRateFromAngularError(angularErrorAtCurrentTick,
                                                                                                                               momentOfInertia,
                                                                                                                               desiredBodyOrientation,
                                                                                                                               desiredNetAngularMomentumRate,
                                                                                                                               desiredInternalAngularMomentumRate);
      FrameVector3DReadOnly angularVelocityErrorRateFromAngularVelocityError = computeExpectedAngularVelocityErrorRateFromAngularVelocityError(angularVelocityErrorAtCurrentTick,
                                                                                                                                               momentOfInertia,
                                                                                                                                               desiredBodyAngularVelocity);
      FrameVector3DReadOnly angularVelocityErrorRateFromContact = computeExpectedAngularVelocityErrorRateFromContact(mass,
                                                                                                                     comPosition,
                                                                                                                     momentOfInertia,
                                                                                                                     desiredCoMPosition,
                                                                                                                     desiredCoMAcceleration,
                                                                                                                     desiredBodyOrientation,
                                                                                                                     desiredNetAngularMomentumRate,
                                                                                                                     contactPlanes);

      angularVelocityErrorRate.set(angularVelocityErrorRateFromAngularError);
      angularVelocityErrorRate.add(angularVelocityErrorRateFromAngularVelocityError);
      angularVelocityErrorRate.add(angularVelocityErrorRateFromContact);


      return angularVelocityErrorRate;
   }

   public static FrameVector3DReadOnly computeExpectedAngularVelocityErrorRateFromAngularError(FrameVector3DReadOnly angularErrorAtCurrentTick,
                                                                                               Matrix3DReadOnly momentOfInertia,
                                                                                               FrameOrientation3DReadOnly desiredBodyOrientation,
                                                                                               Vector3DReadOnly desiredNetAngularMomentumRate,
                                                                                               Vector3DReadOnly desiredInternalAngularMomentumRate)
   {
      FrameVector3D angularVelocityErrorRateFromAngularError = new FrameVector3D();

      FrameVector3D desiredBodyAngularMomentumRate = new FrameVector3D();
      desiredBodyAngularMomentumRate.sub(desiredNetAngularMomentumRate, desiredInternalAngularMomentumRate);

      FrameVector3D tempVector = new FrameVector3D(desiredBodyAngularMomentumRate);
      desiredBodyOrientation.inverseTransform(tempVector);
      angularVelocityErrorRateFromAngularError.cross(tempVector, angularErrorAtCurrentTick);
      momentOfInertia.inverseTransform(angularVelocityErrorRateFromAngularError);

      return angularVelocityErrorRateFromAngularError;
   }

   public static FrameVector3DReadOnly computeExpectedAngularVelocityErrorRateFromAngularVelocityError(FrameVector3DReadOnly angularVelocityErrorAtCurrentTick,
                                                                                                       Matrix3DReadOnly momentOfInertia,
                                                                                                       Vector3DReadOnly desiredBodyAngularVelocity)
   {
      FrameVector3D angularVelocityErrorRateFromAngularVelocityError = new FrameVector3D();

      FrameVector3D tempVector = new FrameVector3D();
      FrameVector3D tempVector2 = new FrameVector3D();
      Matrix3D tempMatrix = new Matrix3D();
      momentOfInertia.inverseTransform(desiredBodyAngularVelocity, tempVector);
      MatrixMissingTools.toSkewSymmetricMatrix(tempVector, tempMatrix);

      tempMatrix.transform(angularVelocityErrorAtCurrentTick, angularVelocityErrorRateFromAngularVelocityError);
      momentOfInertia.transform(angularVelocityErrorAtCurrentTick, tempVector);
      tempVector2.cross(desiredBodyAngularVelocity, tempVector);
      angularVelocityErrorRateFromAngularVelocityError.sub(tempVector2);
      momentOfInertia.inverseTransform(angularVelocityErrorRateFromAngularVelocityError);

      return angularVelocityErrorRateFromAngularVelocityError;
   }

   public static FrameVector3DReadOnly computeExpectedAngularErrorRateFromContact()
   {
      return new FrameVector3D();
   }

   public static FrameVector3DReadOnly computeExpectedAngularErrorRate(FrameVector3DReadOnly angularErrorAtCurrentTick,
                                                                       FrameVector3DReadOnly angularVelocityErrorAtCurrentTick,
                                                                       Vector3DReadOnly desiredBodyAngularVelocity)
   {
      FrameVector3D angularErrorRate = new FrameVector3D();
      FrameVector3D angularErrorRateFromAngularError = new FrameVector3D(computeExpectedAngularErrorRateFromAngularError(angularErrorAtCurrentTick, desiredBodyAngularVelocity));
      FrameVector3D angularErrorRateFromAngularVelocityError = new FrameVector3D(computeExpectedAngularErrorRateFromAngularVelocityError(angularVelocityErrorAtCurrentTick));

      angularErrorRate.add(angularErrorRateFromAngularError, angularErrorRateFromAngularVelocityError);

      return angularErrorRate;
   }

   public static FrameVector3DReadOnly computeExpectedAngularErrorRateFromAngularError(FrameVector3DReadOnly angularErrorAtCurrentTick,
                                                                                       Vector3DReadOnly desiredBodyAngularVelocity)
   {
      FrameVector3D angularErrorRateFromAngularError = new FrameVector3D();

      angularErrorRateFromAngularError.cross(desiredBodyAngularVelocity, angularErrorAtCurrentTick);
      angularErrorRateFromAngularError.scale(-1.0);

      return angularErrorRateFromAngularError;
   }

   public static FrameVector3DReadOnly computeExpectedAngularErrorRateFromAngularVelocityError(FrameVector3DReadOnly angularVelocityErrorAtCurrentTick)
   {
      return new FrameVector3D(angularVelocityErrorAtCurrentTick);
   }

   static DMatrixRMaj computeActualRateVector(FrameVector3DReadOnly angularErrorAtCurrentTick,
                                              FrameVector3DReadOnly angularVelocityErrorAtCurrentTick,
                                              DMatrixRMaj trajectoryCoefficients,
                                              OrientationDynamicsCalculator inputCalculator)
   {
      DMatrixRMaj stateVector = new DMatrixRMaj(6, 1);
      DMatrixRMaj rateVector = new DMatrixRMaj(6, 1);

      angularErrorAtCurrentTick.get(stateVector);
      angularVelocityErrorAtCurrentTick.get(3, stateVector);

      CommonOps_DDRM.mult(inputCalculator.getContinuousAMatrix(), stateVector, rateVector);
      CommonOps_DDRM.multAdd(inputCalculator.getContinuousBMatrix(), trajectoryCoefficients, rateVector);
      CommonOps_DDRM.addEquals(rateVector, inputCalculator.getContinuousCMatrix());

      return rateVector;
   }

   static DMatrixRMaj computeActualRateVectorFromAngularError(FrameVector3DReadOnly angularErrorAtCurrentTick, OrientationDynamicsCalculator inputCalculator)
   {
      DMatrixRMaj angleErrorStateVector = new DMatrixRMaj(6, 1);
      DMatrixRMaj rateFromAngleError = new DMatrixRMaj(6, 1);

      angularErrorAtCurrentTick.get(angleErrorStateVector);

      CommonOps_DDRM.mult(inputCalculator.getContinuousAMatrix(), angleErrorStateVector, rateFromAngleError);

      return rateFromAngleError;
   }

   static DMatrixRMaj computeActualRateVectorFromContact(DMatrixRMaj trajectoryCoefficients, OrientationDynamicsCalculator inputCalculator)
   {
      DMatrixRMaj rateFromContact = new DMatrixRMaj(6, 1);

      CommonOps_DDRM.mult(inputCalculator.getContinuousBMatrix(), trajectoryCoefficients, rateFromContact);
      CommonOps_DDRM.addEquals(rateFromContact, inputCalculator.getContinuousCMatrix());

      return rateFromContact;
   }

   static DMatrixRMaj computeActualRateVectorFromAngularVelocityError(FrameVector3DReadOnly angularVelocityErrorAtCurrentTick,
                                                                      OrientationDynamicsCalculator inputCalculator)
   {
      DMatrixRMaj velocityErrorStateVector = new DMatrixRMaj(6, 1);
      DMatrixRMaj rateFromAngularVelocityError = new DMatrixRMaj(6, 1);

      angularVelocityErrorAtCurrentTick.get(3, velocityErrorStateVector);

      CommonOps_DDRM.mult(inputCalculator.getContinuousAMatrix(), velocityErrorStateVector, rateFromAngularVelocityError);

      return rateFromAngularVelocityError;
   }

   static FrameVector3DReadOnly computeActualAngularVelocityErrorRateFromContact(DMatrixRMaj trajectoryCoefficients,
                                                                                 OrientationDynamicsCalculator inputCalculator)
   {
      FrameVector3D actualAngularVelocityErrorRateFromContact = new FrameVector3D();
      actualAngularVelocityErrorRateFromContact.set(3, computeActualRateVectorFromContact(trajectoryCoefficients, inputCalculator));

      return actualAngularVelocityErrorRateFromContact;
   }

   static FrameVector3DReadOnly computeActualAngularErrorRateFromContact(DMatrixRMaj trajectoryCoefficients, OrientationDynamicsCalculator inputCalculator)
   {
      FrameVector3D angularErrorRateFromContact = new FrameVector3D();
      angularErrorRateFromContact.set(computeActualRateVectorFromContact(trajectoryCoefficients, inputCalculator));

      return angularErrorRateFromContact;
   }

   static FrameVector3DReadOnly computeActualAngularVelocityErrorRateFromAngularVelocityError(FrameVector3DReadOnly angularVelocityErrorAtCurrentTick,
                                                                                              OrientationDynamicsCalculator inputCalculator)
   {
      FrameVector3D actualAngularVelocityErrorRateFromAngularError = new FrameVector3D();
      actualAngularVelocityErrorRateFromAngularError.set(3, computeActualRateVectorFromAngularVelocityError(angularVelocityErrorAtCurrentTick, inputCalculator));

      return actualAngularVelocityErrorRateFromAngularError;
   }

   static FrameVector3DReadOnly computeActualAngularErrorRateFromAngularVelocityError(FrameVector3DReadOnly angularVelocityErrorAtCurrentTick,
                                                                                      OrientationDynamicsCalculator inputCalculator)
   {
      FrameVector3D angularErrorRateFromAngularError = new FrameVector3D();
      angularErrorRateFromAngularError.set(computeActualRateVectorFromAngularVelocityError(angularVelocityErrorAtCurrentTick, inputCalculator));

      return angularErrorRateFromAngularError;
   }

   static FrameVector3DReadOnly computeActualAngularVelocityErrorRateFromAngularError(FrameVector3DReadOnly angularErrorAtCurrentTick,
                                                                                      OrientationDynamicsCalculator inputCalculator)
   {
      FrameVector3D actualAngularVelocityErrorRateFromAngularError = new FrameVector3D();
      actualAngularVelocityErrorRateFromAngularError.set(3, computeActualRateVectorFromAngularError(angularErrorAtCurrentTick, inputCalculator));

      return actualAngularVelocityErrorRateFromAngularError;
   }

   static FrameVector3DReadOnly computeActualAngularErrorRateFromAngularError(FrameVector3DReadOnly angularErrorAtCurrentTick,
                                                                              OrientationDynamicsCalculator inputCalculator)
   {
      FrameVector3D angularErrorRateFromAngularError = new FrameVector3D();
      angularErrorRateFromAngularError.set(computeActualRateVectorFromAngularError(angularErrorAtCurrentTick, inputCalculator));

      return angularErrorRateFromAngularError;
   }

   static FrameVector3DReadOnly computeActualAngularVelocityErrorRate(FrameVector3DReadOnly angularErrorAtCurrentTick,
                                                                      FrameVector3DReadOnly angularVelocityErrorAtCurrentTick,
                                                                      DMatrixRMaj trajectoryCoefficients,
                                                                      OrientationDynamicsCalculator inputCalculator)
   {
      FrameVector3D actualAngularVelocityErrorRate = new FrameVector3D();
      actualAngularVelocityErrorRate.set(3, computeActualRateVector(angularErrorAtCurrentTick,
                                                                    angularVelocityErrorAtCurrentTick,
                                                                    trajectoryCoefficients,
                                                                    inputCalculator));

      return actualAngularVelocityErrorRate;
   }

   static FrameVector3DReadOnly computeActualAngularErrorRate(FrameVector3DReadOnly angularErrorAtCurrentTick,
                                                              FrameVector3DReadOnly angularVelocityErrorAtCurrentTick,
                                                              DMatrixRMaj trajectoryCoefficients,
                                                              OrientationDynamicsCalculator inputCalculator)
   {
      FrameVector3D actualAngularErrorRate = new FrameVector3D();
      actualAngularErrorRate.set(computeActualRateVector(angularErrorAtCurrentTick,
                                                         angularVelocityErrorAtCurrentTick,
                                                         trajectoryCoefficients,
                                                         inputCalculator));

      return actualAngularErrorRate;
   }

   static void assertRateFromAngularErrorIsCorrect(FrameVector3DReadOnly angularErrorAtCurrentTick,
                                                   OrientationDynamicsCalculator inputCalculator,
                                                   Matrix3DReadOnly momentOfInertia,
                                                   FrameOrientation3DReadOnly desiredBodyOrientation,
                                                   Vector3DReadOnly desiredBodyAngularVelocity,
                                                   Vector3DReadOnly desiredNetAngularMomentumRate,
                                                   Vector3DReadOnly desiredInternalAngularMomentumRate)
   {
      FrameVector3DReadOnly expectedAngularVelocityErrorRateFromAngularError = computeExpectedAngularVelocityErrorRateFromAngularError(angularErrorAtCurrentTick,
                                                                                                                                       momentOfInertia,
                                                                                                                                       desiredBodyOrientation,
                                                                                                                                       desiredNetAngularMomentumRate,
                                                                                                                                       desiredInternalAngularMomentumRate);
      FrameVector3DReadOnly expectedAngularErrorRateFromAngularError = computeExpectedAngularErrorRateFromAngularError(angularErrorAtCurrentTick, desiredBodyAngularVelocity);
      FrameVector3DReadOnly actualAngularErrorRateFromAngularError = computeActualAngularErrorRateFromAngularError(angularErrorAtCurrentTick, inputCalculator);
      FrameVector3DReadOnly actualAngularVelocityErrorRateFromAngularError = computeActualAngularVelocityErrorRateFromAngularError(angularErrorAtCurrentTick,
                                                                                                                                   inputCalculator);

      EuclidFrameTestTools.assertFrameVector3DGeometricallyEquals("Angular error rate from angular error is incorrect",
                                                                  expectedAngularErrorRateFromAngularError,
                                                                  actualAngularErrorRateFromAngularError,
                                                                  1e-6);
      EuclidFrameTestTools.assertFrameVector3DGeometricallyEquals("Angular velocity error rate from angular error is incorrect",
                                                                  expectedAngularVelocityErrorRateFromAngularError,
                                                                  actualAngularVelocityErrorRateFromAngularError,
                                                                  1e-6);
   }

   public static FrameVector3DReadOnly computeExpectedAngularVelocityErrorRateFromContact(double mass,
                                                                                          FramePoint3DReadOnly comPosition,
                                                                                          Matrix3DReadOnly momentOfInertia,
                                                                                          FramePoint3DReadOnly desiredCoMPosition,
                                                                                          FrameVector3DReadOnly desiredCoMAcceleration,
                                                                                          FrameOrientation3DReadOnly desiredBodyOrientation,
                                                                                          Vector3DReadOnly desiredNetAngularMomentumRate,
                                                                                          List<MPCContactPlane> contactPlanes)
   {
      FrameVector3D angularVelocityErrorRateFromContact = new FrameVector3D();

      List<MPCContactPoint> allContactPoints = new ArrayList<>();

      for (int i = 0; i < contactPlanes.size(); i++)
      {
         for (int j = 0; j < contactPlanes.get(i).getNumberOfContactPoints(); j++)
            allContactPoints.add(contactPlanes.get(i).getContactPointHelper(j));
      }

      FrameVector3D desiredContactPointForce = new FrameVector3D(desiredCoMAcceleration);
      desiredContactPointForce.addZ(Math.abs(gravityZ));
      desiredContactPointForce.scale(mass);

      angularVelocityErrorRateFromContact.sub(desiredNetAngularMomentumRate);

      FrameVector3D desiredTorqueFromContact = new FrameVector3D();
      FrameVector3D comError = new FrameVector3D();

      comError.sub(comPosition, desiredCoMPosition);
      desiredTorqueFromContact.cross(desiredContactPointForce, comError);
      angularVelocityErrorRateFromContact.add(desiredTorqueFromContact);


      for (int contactIdx = 0; contactIdx < allContactPoints.size(); contactIdx++)
      {
         MPCContactPoint contactPoint = allContactPoints.get(contactIdx);

         FrameVector3D desiredMomentArm = new FrameVector3D();
         desiredMomentArm.sub(contactPoint.getBasisVectorOrigin(), desiredCoMPosition);

         FrameVector3D torqueFromContact = new FrameVector3D();
         torqueFromContact.cross(desiredMomentArm, contactPoint.getContactAcceleration());
         torqueFromContact.scale(mass);

         angularVelocityErrorRateFromContact.add(torqueFromContact);
      }

      desiredBodyOrientation.inverseTransform(angularVelocityErrorRateFromContact);
      momentOfInertia.inverseTransform(angularVelocityErrorRateFromContact);

      return angularVelocityErrorRateFromContact;
   }

   static void assertRateFromAngularVelocityErrorIsCorrect(FrameVector3DReadOnly angularVelocityErrorAtCurrentTick,
                                                           OrientationDynamicsCalculator inputCalculator,
                                                           Matrix3DReadOnly momentOfInertia,
                                                           Vector3DReadOnly desiredBodyAngularVelocity)
   {
      FrameVector3DReadOnly angularErrorRateFromAngularVelocityError = computeExpectedAngularErrorRateFromAngularVelocityError(angularVelocityErrorAtCurrentTick);

      FrameVector3DReadOnly expectedAngularVelocityErrorRateFromAngularVelocityError = computeExpectedAngularVelocityErrorRateFromAngularVelocityError(
            angularVelocityErrorAtCurrentTick,
            momentOfInertia,
            desiredBodyAngularVelocity);
      EuclidFrameTestTools.assertFrameVector3DGeometricallyEquals("Angular error rate from angular velocity error is incorrect",
                                                                  angularErrorRateFromAngularVelocityError,
                                                                  computeActualAngularErrorRateFromAngularVelocityError(angularVelocityErrorAtCurrentTick, inputCalculator),
                                                                  1e-6);
      EuclidFrameTestTools.assertFrameVector3DGeometricallyEquals("Angular velocity error rate from angular velocity error is incorrect",
                                                                  expectedAngularVelocityErrorRateFromAngularVelocityError,
                                                                  computeActualAngularVelocityErrorRateFromAngularVelocityError(angularVelocityErrorAtCurrentTick, inputCalculator),
                                                                  1e-6);
   }

   static void assertRateFromContactIsCorrect(double mass,
                                              FramePoint3DReadOnly comPosition,
                                              DMatrixRMaj trajectoryCoefficients,
                                              OrientationDynamicsCalculator inputCalculator,
                                              Matrix3DReadOnly momentOfInertia,
                                              FramePoint3DReadOnly desiredCoMPosition,
                                              FrameVector3DReadOnly desiredCoMAcceleration,
                                              FrameOrientation3DReadOnly desiredBodyOrientation,
                                              Vector3DReadOnly desiredNetAngularMomentumRate,
                                              List<MPCContactPlane> contactPlanes)
   {

      FrameVector3DReadOnly expectedAngularErrorRateFromContact = computeExpectedAngularErrorRateFromContact();
      FrameVector3DReadOnly expectedAngularVelocityErrorRateFromContact = computeExpectedAngularVelocityErrorRateFromContact(mass,
                                                                                                                             comPosition,
                                                                                                                             momentOfInertia,
                                                                                                                             desiredCoMPosition,
                                                                                                                             desiredCoMAcceleration,
                                                                                                                             desiredBodyOrientation,
                                                                                                                             desiredNetAngularMomentumRate,
                                                                                                                             contactPlanes);
      FrameVector3DReadOnly actualAngularErrorRateFromContact = computeActualAngularErrorRateFromContact(trajectoryCoefficients, inputCalculator);
      FrameVector3DReadOnly actualAngularVelocityErrorRateFromContact = computeActualAngularVelocityErrorRateFromContact(trajectoryCoefficients,
                                                                                                                         inputCalculator);

      EuclidFrameTestTools.assertFrameVector3DGeometricallyEquals("Angular error rate from contact is incorrect",
                                                                  expectedAngularErrorRateFromContact,
                                                                  actualAngularErrorRateFromContact,
                                                                  1e-6);
      EuclidFrameTestTools.assertFrameVector3DGeometricallyEquals("Angular velocity error rate from contact is incorrect",
                                                                  expectedAngularVelocityErrorRateFromContact,
                                                                  actualAngularVelocityErrorRateFromContact,
                                                                  1e-6);
   }

   static void assertRateIsCorrect(double mass,
                                   FramePoint3DReadOnly comPosition,
                                   FrameVector3DReadOnly angularErrorAtCurrentTick,
                                   FrameVector3DReadOnly angularVelocityErrorAtCurrentTick,
                                   DMatrixRMaj trajectoryCoefficients,
                                   OrientationDynamicsCalculator inputCalculator,
                                   Matrix3DReadOnly momentOfInertia,
                                   FramePoint3DReadOnly desiredCoMPosition,
                                   FrameVector3DReadOnly desiredCoMAcceleration,
                                   FrameOrientation3DReadOnly desiredBodyOrientation,
                                   Vector3DReadOnly desiredBodyAngularVelocity,
                                   Vector3DReadOnly desiredNetAngularMomentumRate,
                                   Vector3DReadOnly desiredInternalAngularMomentumRate,
                                   List<MPCContactPlane> contactPlanes)
   {
      FrameVector3DReadOnly expectedAngularErrorRate = computeExpectedAngularErrorRate(angularErrorAtCurrentTick,
                                                                                       angularVelocityErrorAtCurrentTick,
                                                                                       desiredBodyAngularVelocity);
      FrameVector3DReadOnly actualAngularErrorRate = computeActualAngularErrorRate(angularErrorAtCurrentTick,
                                                                                   angularVelocityErrorAtCurrentTick,
                                                                                   trajectoryCoefficients,
                                                                                   inputCalculator);
      FrameVector3DReadOnly expectedAngularVelocityErrorRate = computeExpectedAngularVelocityErrorRate(mass,
                                                                                                       comPosition,
                                                                                                       angularErrorAtCurrentTick,
                                                                                                       angularVelocityErrorAtCurrentTick,
                                                                                                       momentOfInertia,
                                                                                                       desiredCoMPosition,
                                                                                                       desiredCoMAcceleration,
                                                                                                       desiredBodyOrientation,
                                                                                                       desiredBodyAngularVelocity,
                                                                                                       desiredNetAngularMomentumRate,
                                                                                                       desiredInternalAngularMomentumRate,
                                                                                                       contactPlanes);
      FrameVector3DReadOnly actualAngularVelocityErrorRate = computeActualAngularVelocityErrorRate(angularErrorAtCurrentTick,
                                                                                                   angularVelocityErrorAtCurrentTick,
                                                                                                   trajectoryCoefficients,
                                                                                                   inputCalculator);

      EuclidFrameTestTools.assertFrameVector3DGeometricallyEquals("Angular error rate is incorrect",
                                                                  expectedAngularErrorRate,
                                                                  actualAngularErrorRate,
                                                                  1e-6);
      EuclidFrameTestTools.assertFrameVector3DGeometricallyEquals("Angular velocity error rate is incorrect",
                                                                  expectedAngularVelocityErrorRate,
                                                                  actualAngularVelocityErrorRate,
                                                                  1e-6);
   }

   public static void assertAllRatesAreCorrect(double mass,
                                               FramePoint3DReadOnly comPosition,
                                               FrameVector3DReadOnly angularErrorAtCurrentTick,
                                               FrameVector3DReadOnly angularVelocityErrorAtCurrentTick,
                                               DMatrixRMaj trajectoryCoefficients,
                                               OrientationDynamicsCalculator inputCalculator,
                                               Matrix3DReadOnly momentOfInertia,
                                               FramePoint3DReadOnly desiredCoMPosition,
                                               FrameVector3DReadOnly desiredCoMAcceleration,
                                               FrameOrientation3DReadOnly desiredBodyOrientation,
                                               Vector3DReadOnly desiredBodyAngularVelocity,
                                               Vector3DReadOnly desiredNetAngularMomentumRate,
                                               Vector3DReadOnly desiredInternalAngularMomentumRate,
                                               List<MPCContactPlane> contactPlanes)
   {
      assertRateFromAngularErrorIsCorrect(angularErrorAtCurrentTick,
                                          inputCalculator,
                                          momentOfInertia,
                                          desiredBodyOrientation,
                                          desiredBodyAngularVelocity,
                                          desiredNetAngularMomentumRate,
                                          desiredInternalAngularMomentumRate);
      assertRateFromAngularVelocityErrorIsCorrect(angularVelocityErrorAtCurrentTick, inputCalculator, momentOfInertia, desiredBodyAngularVelocity);
      assertRateFromContactIsCorrect(mass,
                                     comPosition,
                                     trajectoryCoefficients,
                                     inputCalculator,
                                     momentOfInertia,
                                     desiredCoMPosition,
                                     desiredCoMAcceleration,
                                     desiredBodyOrientation,
                                     desiredNetAngularMomentumRate,
                                     contactPlanes);
      assertRateIsCorrect(mass,
                          comPosition,
                          angularErrorAtCurrentTick,
                          angularVelocityErrorAtCurrentTick,
                          trajectoryCoefficients,
                          inputCalculator,
                          momentOfInertia,
                          desiredCoMPosition,
                          desiredCoMAcceleration,
                          desiredBodyOrientation,
                          desiredBodyAngularVelocity,
                          desiredNetAngularMomentumRate,
                          desiredInternalAngularMomentumRate,
                          contactPlanes);
   }
}
