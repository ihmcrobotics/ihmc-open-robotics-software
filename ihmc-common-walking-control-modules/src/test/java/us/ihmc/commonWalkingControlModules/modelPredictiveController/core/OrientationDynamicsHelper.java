package us.ihmc.commonWalkingControlModules.modelPredictiveController.core;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.commands.DiscreteAngularVelocityOrientationCommand;
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
                                                                               DiscreteAngularVelocityOrientationCommand command)
   {
      FrameVector3D angularVelocityErrorRate = new FrameVector3D();
      FrameVector3DReadOnly angularVelocityErrorRateFromAngularError = computeExpectedAngularVelocityErrorRateFromAngularError(angularErrorAtCurrentTick, command);
      FrameVector3DReadOnly angularVelocityErrorRateFromAngularVelocityError = computeExpectedAngularVelocityErrorRateFromAngularVelocityError(angularVelocityErrorAtCurrentTick,
                                                                                                                                                                                command);
      FrameVector3DReadOnly angularVelocityErrorRateFromContact = computeExpectedAngularVelocityErrorRateFromContact(mass, comPosition, command);

      angularVelocityErrorRate.set(angularVelocityErrorRateFromAngularError);
      angularVelocityErrorRate.add(angularVelocityErrorRateFromAngularVelocityError);
      angularVelocityErrorRate.add(angularVelocityErrorRateFromContact);


      return angularVelocityErrorRate;
   }

   public static FrameVector3DReadOnly computeExpectedAngularVelocityErrorRateFromAngularError(FrameVector3DReadOnly angularErrorAtCurrentTick,
                                                                                                DiscreteAngularVelocityOrientationCommand command)
   {
      FrameOrientation3DReadOnly desiredBodyOrientation = command.getDesiredBodyOrientation();
      FrameVector3D angularVelocityErrorRateFromAngularError = new FrameVector3D();

      FrameVector3D desiredBodyAngularMomentumRate = new FrameVector3D();
      desiredBodyAngularMomentumRate.sub(command.getDesiredNetAngularMomentumRate(), command.getDesiredInternalAngularMomentumRate());
      Matrix3DReadOnly momentOfInertia = command.getMomentOfInertiaInBodyFrame();

      FrameVector3D tempVector = new FrameVector3D(desiredBodyAngularMomentumRate);
      desiredBodyOrientation.inverseTransform(tempVector);
      angularVelocityErrorRateFromAngularError.cross(tempVector, angularErrorAtCurrentTick);
      momentOfInertia.inverseTransform(angularVelocityErrorRateFromAngularError);

      return angularVelocityErrorRateFromAngularError;
   }

   public static FrameVector3DReadOnly computeExpectedAngularVelocityErrorRateFromAngularVelocityError(FrameVector3DReadOnly angularVelocityErrorAtCurrentTick,
                                                                                                        DiscreteAngularVelocityOrientationCommand command)
   {
      Vector3DReadOnly desiredBodyAngularVelocity = command.getDesiredBodyAngularVelocity();
      Matrix3DReadOnly momentOfInertia = command.getMomentOfInertiaInBodyFrame();

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
                                                                        DiscreteAngularVelocityOrientationCommand command)
   {
      FrameVector3D angularErrorRate = new FrameVector3D();
      FrameVector3D angularErrorRateFromAngularError = new FrameVector3D(computeExpectedAngularErrorRateFromAngularError(angularErrorAtCurrentTick, command));
      FrameVector3D angularErrorRateFromAngularVelocityError = new FrameVector3D(computeExpectedAngularErrorRateFromAngularVelocityError(angularVelocityErrorAtCurrentTick));

      angularErrorRate.add(angularErrorRateFromAngularError, angularErrorRateFromAngularVelocityError);

      return angularErrorRate;
   }

   public static FrameVector3DReadOnly computeExpectedAngularErrorRateFromAngularError(FrameVector3DReadOnly angularErrorAtCurrentTick,
                                                                                        DiscreteAngularVelocityOrientationCommand command)
   {
      FrameVector3D angularErrorRateFromAngularError = new FrameVector3D();

      Vector3DReadOnly desiredBodyAngularVelocity = command.getDesiredBodyAngularVelocity();
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
                                                   DiscreteAngularVelocityOrientationCommand command)
   {
      FrameVector3DReadOnly expectedAngularVelocityErrorRateFromAngularError = computeExpectedAngularVelocityErrorRateFromAngularError(
            angularErrorAtCurrentTick,
            command);
      FrameVector3DReadOnly expectedAngularErrorRateFromAngularError = computeExpectedAngularErrorRateFromAngularError(angularErrorAtCurrentTick, command);
      FrameVector3DReadOnly actualAngularErrorRateFromAngularError = computeActualAngularErrorRateFromAngularError(angularErrorAtCurrentTick, inputCalculator);
      FrameVector3DReadOnly actualAngularVelocityErrorRateFromAngularError = computeActualAngularVelocityErrorRateFromAngularError(angularErrorAtCurrentTick,
                                                                                                                                   inputCalculator);

      EuclidFrameTestTools.assertFrameVector3DGeometricallyEquals("Angular error rate from angular error is incorrect at tick " + command.getEndDiscreteTickId(),
                                                                  expectedAngularErrorRateFromAngularError,
                                                                  actualAngularErrorRateFromAngularError,
                                                                  1e-6);
      EuclidFrameTestTools.assertFrameVector3DGeometricallyEquals("Angular velocity error rate from angular error is incorrect at tick " + command.getEndDiscreteTickId(),
                                                                  expectedAngularVelocityErrorRateFromAngularError,
                                                                  actualAngularVelocityErrorRateFromAngularError,
                                                                  1e-6);
   }

   public static FrameVector3DReadOnly computeExpectedAngularVelocityErrorRateFromContact(double mass,
                                                                                           FramePoint3DReadOnly comPosition,
                                                                                           DiscreteAngularVelocityOrientationCommand command)
   {
      FrameVector3D angularVelocityErrorRateFromContact = new FrameVector3D();

      Matrix3DReadOnly momentOfInertia = command.getMomentOfInertiaInBodyFrame();
      FramePoint3DReadOnly desiredCoMPosition = command.getDesiredCoMPosition();
      FrameVector3DReadOnly desiredCoMAcceleration = command.getDesiredCoMAcceleration();
      FrameOrientation3DReadOnly desiredBodyOrientation = command.getDesiredBodyOrientation();

      List<MPCContactPoint> allContactPoints = new ArrayList<>();

      for (int i = 0; i < command.getNumberOfContacts(); i++)
      {
         for (int j = 0; j < command.getContactPlane(i).getNumberOfContactPoints(); j++)
            allContactPoints.add(command.getContactPlane(i).getContactPointHelper(j));
      }

      FrameVector3D desiredContactPointForce = new FrameVector3D(desiredCoMAcceleration);
      desiredContactPointForce.addZ(Math.abs(gravityZ));
      desiredContactPointForce.scale(mass / allContactPoints.size());

      for (int contactIdx = 0; contactIdx < allContactPoints.size(); contactIdx++)
      {
         MPCContactPoint contactPoint = allContactPoints.get(contactIdx);

         FrameVector3D desiredMomentArm = new FrameVector3D();
         desiredMomentArm.sub(contactPoint.getBasisVectorOrigin(), desiredCoMPosition);

         FrameVector3D contactForceError = new FrameVector3D(contactPoint.getContactAcceleration());
         contactForceError.scale(mass);
         contactForceError.sub(desiredContactPointForce);

         FrameVector3D comError = new FrameVector3D();
         comError.sub(comPosition, desiredCoMPosition);

         FrameVector3D torqueFromContact = new FrameVector3D();
         torqueFromContact.cross(desiredMomentArm, contactForceError);

         FrameVector3D coriolisForce = new FrameVector3D();
         coriolisForce.cross(desiredContactPointForce, comError);
         torqueFromContact.add(coriolisForce);

         angularVelocityErrorRateFromContact.add(torqueFromContact);
      }

      desiredBodyOrientation.inverseTransform(angularVelocityErrorRateFromContact);
      momentOfInertia.inverseTransform(angularVelocityErrorRateFromContact);

      return angularVelocityErrorRateFromContact;
   }

   static void assertRateFromAngularVelocityErrorIsCorrect(FrameVector3DReadOnly angularVelocityErrorAtCurrentTick,
                                                           OrientationDynamicsCalculator inputCalculator,
                                                           DiscreteAngularVelocityOrientationCommand command)
   {
      FrameVector3DReadOnly angularErrorRateFromAngularVelocityError = computeExpectedAngularErrorRateFromAngularVelocityError(angularVelocityErrorAtCurrentTick);

      FrameVector3DReadOnly expectedAngularVelocityErrorRateFromAngularVelocityError = computeExpectedAngularVelocityErrorRateFromAngularVelocityError(
            angularVelocityErrorAtCurrentTick,
            command);
      EuclidFrameTestTools.assertFrameVector3DGeometricallyEquals("Angular error rate from angular velocity error is incorrect at tick " + command.getEndDiscreteTickId(),
                                                                  angularErrorRateFromAngularVelocityError,
                                                                  computeActualAngularErrorRateFromAngularVelocityError(angularVelocityErrorAtCurrentTick, inputCalculator),
                                                                  1e-6);
      EuclidFrameTestTools.assertFrameVector3DGeometricallyEquals("Angular velocity error rate from angular velocity error is incorrect at tick " + command.getEndDiscreteTickId(),
                                                                  expectedAngularVelocityErrorRateFromAngularVelocityError,
                                                                  computeActualAngularVelocityErrorRateFromAngularVelocityError(angularVelocityErrorAtCurrentTick, inputCalculator),
                                                                  1e-6);
   }

   static void assertRateFromContactIsCorrect(double mass,
                                              FramePoint3DReadOnly comPosition,
                                              DMatrixRMaj trajectoryCoefficients,
                                              OrientationDynamicsCalculator inputCalculator,
                                              DiscreteAngularVelocityOrientationCommand command)
   {

      FrameVector3DReadOnly expectedAngularErrorRateFromContact = computeExpectedAngularErrorRateFromContact();
      FrameVector3DReadOnly expectedAngularVelocityErrorRateFromContact = computeExpectedAngularVelocityErrorRateFromContact(mass, comPosition, command);
      FrameVector3DReadOnly actualAngularErrorRateFromContact = computeActualAngularErrorRateFromContact(trajectoryCoefficients, inputCalculator);
      FrameVector3DReadOnly actualAngularVelocityErrorRateFromContact = computeActualAngularVelocityErrorRateFromContact(trajectoryCoefficients,
                                                                                                                         inputCalculator);

      EuclidFrameTestTools.assertFrameVector3DGeometricallyEquals("Angular error rate from contact is incorrect at tick " + command.getEndDiscreteTickId(),
                                                                  expectedAngularErrorRateFromContact,
                                                                  actualAngularErrorRateFromContact,
                                                                  1e-6);
      EuclidFrameTestTools.assertFrameVector3DGeometricallyEquals("Angular velocity error rate from contact is incorrect at tick " + command.getEndDiscreteTickId(),
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
                                   DiscreteAngularVelocityOrientationCommand command)
   {
      FrameVector3DReadOnly expectedAngularErrorRate = computeExpectedAngularErrorRate(angularErrorAtCurrentTick,
                                                                                       angularVelocityErrorAtCurrentTick,
                                                                                       command);
      FrameVector3DReadOnly actualAngularErrorRate = computeActualAngularErrorRate(angularErrorAtCurrentTick,
                                                                                   angularVelocityErrorAtCurrentTick,
                                                                                   trajectoryCoefficients,
                                                                                   inputCalculator);
      FrameVector3DReadOnly expectedAngularVelocityErrorRate = computeExpectedAngularVelocityErrorRate(mass,
                                                                                                       comPosition,
                                                                                                       angularErrorAtCurrentTick,
                                                                                                       angularVelocityErrorAtCurrentTick,
                                                                                                       command);
      FrameVector3DReadOnly actualAngularVelocityErrorRate = computeActualAngularVelocityErrorRate(angularErrorAtCurrentTick,
                                                                                                   angularVelocityErrorAtCurrentTick,
                                                                                                   trajectoryCoefficients,
                                                                                                   inputCalculator);

      EuclidFrameTestTools.assertFrameVector3DGeometricallyEquals("Angular error rate is incorrect at tick " + command.getEndDiscreteTickId(),
                                                                  expectedAngularErrorRate,
                                                                  actualAngularErrorRate,
                                                                  1e-6);
      EuclidFrameTestTools.assertFrameVector3DGeometricallyEquals("Angular velocity error rate is incorrect at tick " + command.getEndDiscreteTickId(),
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
                                               DiscreteAngularVelocityOrientationCommand command)
   {
      assertRateFromAngularErrorIsCorrect(angularErrorAtCurrentTick, inputCalculator, command);
      assertRateFromAngularVelocityErrorIsCorrect(angularVelocityErrorAtCurrentTick, inputCalculator, command);
      assertRateFromContactIsCorrect(mass, comPosition, trajectoryCoefficients, inputCalculator, command);
      assertRateIsCorrect(mass, comPosition, angularErrorAtCurrentTick, angularVelocityErrorAtCurrentTick, trajectoryCoefficients, inputCalculator, command);
   }
}
