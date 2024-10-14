package us.ihmc.commonWalkingControlModules.wrenchDistribution;

import java.util.List;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoContactPoint;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.PlaneContactStateCommand;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.SpatialForce;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.spatial.interfaces.WrenchReadOnly;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint2D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class PlaneContactStateToWrenchMatrixHelper
{
   /**
    * This is used when determining whether two contact points are at the same location. If that is the
    * case one of them will rotate it's friction cone approximation to get better coverage of the cone
    * through the basis vectors.
    */
   private static final double distanceThresholdBetweenTwoContactPoint = 0.005;

   /**
    * If the size of the foothold is below this threshold CoP objectives for this plane will be
    * ignored.
    */
   private static final double minFootholdSizeForCoPObjectives = 1.0e-3;

   public static final boolean useOldCoPObjectiveFormulation = true;

   private final int maxNumberOfContactPoints;
   private final int numberOfBasisVectorsPerContactPoint;
   private final double basisVectorAngleIncrement;

   private final int rhoSize;

   private final DMatrixRMaj rhoMatrix;
   private final DMatrixRMaj wrenchJacobianInCoMFrame;
   private final DMatrixRMaj wrenchJacobianInPlaneFrame;

   private final DMatrixRMaj copRegularizationJacobian;
   private final DMatrixRMaj copRegularizationObjective;
   private final DMatrixRMaj copRateRegularizationJacobian;
   private final DMatrixRMaj copRateRegularizationObjective;

   private final DMatrixRMaj rhoMaxMatrix;
   private final DMatrixRMaj rhoWeightMatrix;
   private final DMatrixRMaj rhoRateWeightMatrix;
   private final DMatrixRMaj copRegularizationWeightMatrix = new DMatrixRMaj(2, 2);
   private final DMatrixRMaj copRateRegularizationWeightMatrix = new DMatrixRMaj(2, 2);

   private final DMatrixRMaj activeRhoMatrix;

   private final YoPlaneContactState yoPlaneContactState;

   private final YoBoolean hasReset;
   private final YoBoolean resetRequested;

   private final FrameVector3D contactNormalVector = new FrameVector3D();

   private final ReferenceFrame centerOfMassFrame;
   private final PoseReferenceFrame planeFrame;

   private final YoFramePoint2D desiredCoP;
   private final YoFramePoint2D previousCoP;

   private final YoBoolean deactivateRhoWhenNotInContact;

   private final YoBoolean[] rhoEnabled;
   private final FramePoint3D[] basisVectorsOrigin;
   private final FrameVector3D[] basisVectors;
   private final YoDouble[] maxContactForces;
   private final YoDouble[] rhoWeights;

   private final RotationMatrix normalContactVectorRotationMatrix = new RotationMatrix();

   private final FramePoint2D contactPoint2d = new FramePoint2D();
   private final FrictionConeRotationCalculator coneRotationCalculator;
   private final CoPObjectiveCalculator copObjectiveCalculator = new CoPObjectiveCalculator();

   public PlaneContactStateToWrenchMatrixHelper(ContactablePlaneBody contactablePlaneBody, ReferenceFrame centerOfMassFrame, int maxNumberOfContactPoints,
                                                int numberOfBasisVectorsPerContactPoint, FrictionConeRotationCalculator coneRotationCalculator,
                                                YoRegistry parentRegistry)
   {
      List<FramePoint2D> contactPoints2d = contactablePlaneBody.getContactPoints2d();

      if (contactPoints2d.size() > maxNumberOfContactPoints)
         throw new RuntimeException("Unexpected number of contact points: " + contactPoints2d.size());

      this.centerOfMassFrame = centerOfMassFrame;
      this.maxNumberOfContactPoints = maxNumberOfContactPoints;
      this.numberOfBasisVectorsPerContactPoint = numberOfBasisVectorsPerContactPoint;
      this.coneRotationCalculator = coneRotationCalculator;

      rhoSize = maxNumberOfContactPoints * numberOfBasisVectorsPerContactPoint;
      basisVectorAngleIncrement = 2.0 * Math.PI / numberOfBasisVectorsPerContactPoint;

      rhoMatrix = new DMatrixRMaj(rhoSize, 1);
      wrenchJacobianInCoMFrame = new DMatrixRMaj(SpatialForce.SIZE, rhoSize);
      copRegularizationJacobian = new DMatrixRMaj(2, rhoSize);
      copRegularizationObjective = new DMatrixRMaj(2, 1);
      copRegularizationObjective.zero();
      copRateRegularizationJacobian = new DMatrixRMaj(2, rhoSize);
      copRateRegularizationObjective = new DMatrixRMaj(2, 1);
      copRateRegularizationObjective.zero();
      wrenchJacobianInPlaneFrame = new DMatrixRMaj(Wrench.SIZE, rhoSize);

      rhoMaxMatrix = new DMatrixRMaj(rhoSize, 1);
      rhoWeightMatrix = new DMatrixRMaj(rhoSize, rhoSize);
      rhoRateWeightMatrix = new DMatrixRMaj(rhoSize, rhoSize);

      activeRhoMatrix = new DMatrixRMaj(rhoSize, 1);
      CommonOps_DDRM.fill(activeRhoMatrix, 1.0);

      CommonOps_DDRM.fill(rhoMaxMatrix, Double.POSITIVE_INFINITY);

      String bodyName = contactablePlaneBody.getName();
      String namePrefix = bodyName + "WrenchMatrixHelper";
      YoRegistry registry = new YoRegistry(namePrefix);

      RigidBodyBasics rigidBody = contactablePlaneBody.getRigidBody();
      planeFrame = new PoseReferenceFrame(namePrefix + "ContactFrame", rigidBody.getBodyFixedFrame());
      planeFrame.setPoseAndUpdate(contactablePlaneBody.getSoleFrame().getTransformToDesiredFrame(rigidBody.getBodyFixedFrame()));
      yoPlaneContactState = new YoPlaneContactState(namePrefix, rigidBody, planeFrame, contactPoints2d, 0.0, registry);
      yoPlaneContactState.clear();
      yoPlaneContactState.computeSupportPolygon();

      hasReset = new YoBoolean(namePrefix + "HasReset", registry);
      resetRequested = new YoBoolean(namePrefix + "ResetRequested", registry);
      deactivateRhoWhenNotInContact = new YoBoolean(namePrefix + "DeactivateRhoWhenNotInContact", registry);

      rhoWeights = new YoDouble[contactPoints2d.size()];
      maxContactForces = new YoDouble[contactPoints2d.size()];

      for (int i = 0; i < contactPoints2d.size(); i++)
      {
         YoDouble rhoWeight = new YoDouble(namePrefix + "RhoWeight" + i, registry);
         YoDouble maxContactForce = new YoDouble(namePrefix + "MaxContactForce" + i, registry);
         maxContactForce.set(Double.POSITIVE_INFINITY);

         rhoWeights[i] = rhoWeight;
         maxContactForces[i] = maxContactForce;
      }

      rhoEnabled = new YoBoolean[rhoSize];
      basisVectors = new FrameVector3D[rhoSize];
      basisVectorsOrigin = new FramePoint3D[rhoSize];

      for (int i = 0; i < rhoSize; i++)
      {
         rhoEnabled[i] = new YoBoolean("Rho" + i + "Enabled", registry);
         basisVectors[i] = new FrameVector3D(centerOfMassFrame);
         basisVectorsOrigin[i] = new FramePoint3D(centerOfMassFrame);
      }

      previousCoP = new YoFramePoint2D(namePrefix + "PreviousCoP", planeFrame, registry);
      desiredCoP = new YoFramePoint2D(namePrefix + "DesiredCoP", planeFrame, registry);

      parentRegistry.addChild(registry);
   }

   public void setDeactivateRhoWhenNotInContact(boolean deactivateRhoWhenNotInContact)
   {
      this.deactivateRhoWhenNotInContact.set(deactivateRhoWhenNotInContact);
   }

   public void setPlaneContactStateCommand(PlaneContactStateCommand command)
   {
      RigidBodyTransform contactFramePose = command.getContactFramePoseInBodyFixedFrame();
      if (!contactFramePose.containsNaN())
         planeFrame.setPoseAndUpdate(contactFramePose);

      int previousNumberOfContacts = yoPlaneContactState.getNumberOfContactPointsInContact();

      yoPlaneContactState.updateFromPlaneContactStateCommand(command);
      yoPlaneContactState.computeSupportPolygon();

      int newNumberOfContacts = yoPlaneContactState.getNumberOfContactPointsInContact();
      boolean touchedDown = previousNumberOfContacts == 0 && newNumberOfContacts > 0;
      boolean liftedOff = previousNumberOfContacts > 0 && newNumberOfContacts == 0;

      if (yoPlaneContactState.pollContactHasChangedNotification())
      {
         resetRequested.set(touchedDown || liftedOff);
      }

      for (int i = 0; i < command.getNumberOfContactPoints(); i++)
      {
         rhoWeights[i].set(command.getRhoWeight(i));

         if (command.hasMaxContactPointNormalForce())
         {
            maxContactForces[i].set(command.getMaxContactPointNormalForce(i));
         }
      }
   }

   public void computeMatrices(double defaultRhoWeight, double rhoRateWeight, Vector2DReadOnly copRegularizationWeight,
                               Vector2DReadOnly copRateRegularizationWeight)
   {
      int numberOfContactPointsInContact = yoPlaneContactState.getNumberOfContactPointsInContact();
      if (numberOfContactPointsInContact > maxNumberOfContactPoints)
         throw new RuntimeException("Unhandled number of contact points: " + numberOfContactPointsInContact);

      // Compute the orientation of the normal contact vector and the corresponding transformation matrix
      computeNormalContactVectorRotation(normalContactVectorRotationMatrix);

      List<YoContactPoint> contactPoints = yoPlaneContactState.getContactPoints();

      int rhoIndex = 0;

      for (int contactPointIndex = 0; contactPointIndex < yoPlaneContactState.getTotalNumberOfContactPoints(); contactPointIndex++)
      {
         YoContactPoint contactPoint = contactPoints.get(contactPointIndex);
         boolean inContact = contactPoint.isInContact();

         // rotate each friction cone approximation to point one vector towards the center of the foot
         double angleOffset = coneRotationCalculator.computeConeRotation(yoPlaneContactState, contactPointIndex);

         // If this contact point is in contact check if there is more points in contact close to it.
         if (inContact)
         {
            int matches = 0;
            for (int j = contactPointIndex + 1; j < contactPoints.size(); j++)
            {
               YoContactPoint candidateForMatch = contactPoints.get(j);
               contactPoint2d.setIncludingFrame(candidateForMatch);
               if (candidateForMatch.isInContact() && contactPoint.epsilonEquals(contactPoint2d, distanceThresholdBetweenTwoContactPoint))
               {
                  matches++;
               }
            }
            if (matches > 1)
            {
               // If there is at least two more contact points in this location disable this one.
               inContact = false;
            }
            else if (matches == 1)
            {
               // If there is one more contact point for this location coming up then rotate this one to get a better friction cone representation.
               angleOffset += basisVectorAngleIncrement / 2.0;
            }
         }

         for (int basisVectorIndex = 0; basisVectorIndex < numberOfBasisVectorsPerContactPoint; basisVectorIndex++)
         {
            FramePoint3D basisVectorOrigin = basisVectorsOrigin[rhoIndex];
            FrameVector3D basisVector = basisVectors[rhoIndex];
            rhoEnabled[rhoIndex].set(inContact);

            if (inContact)
            {
               basisVectorOrigin.setIncludingFrame(contactPoint);
               computeBasisVector(basisVectorIndex, angleOffset, normalContactVectorRotationMatrix, basisVector);

               double rhoWeight = rhoWeights[contactPointIndex].getDoubleValue();
               if (Double.isNaN(rhoWeight))
               {
                  rhoWeight = defaultRhoWeight;
               }

               rhoWeightMatrix.set(rhoIndex, rhoIndex, rhoWeight * maxNumberOfContactPoints / numberOfContactPointsInContact);

               if (resetRequested.getBooleanValue())
                  rhoRateWeightMatrix.set(rhoIndex, rhoIndex, 0.0);
               else
                  rhoRateWeightMatrix.set(rhoIndex, rhoIndex, rhoRateWeight);

               activeRhoMatrix.set(rhoIndex, 0, 1.0);
            }
            else
            {
               clear(rhoIndex);

               if (deactivateRhoWhenNotInContact.getBooleanValue())
                  activeRhoMatrix.set(rhoIndex, 0, 0.0);
            }

            //// TODO: 6/5/17 scale this by the vertical magnitude
            rhoMaxMatrix.set(rhoIndex, 0, maxContactForces[contactPointIndex].getDoubleValue() / numberOfBasisVectorsPerContactPoint);

            rhoIndex++;
         }
      }

      computeWrenchJacobianInFrame(centerOfMassFrame, wrenchJacobianInCoMFrame);
      computeWrenchJacobianInFrame(planeFrame, wrenchJacobianInPlaneFrame);
      double weightScalar = computeCopObjectiveJacobian(copRegularizationJacobian, copRegularizationObjective, desiredCoP);
      computeCopObjectiveJacobian(copRateRegularizationJacobian, copRateRegularizationObjective, previousCoP);

      if (yoPlaneContactState.inContact() && !resetRequested.getBooleanValue() && canHandleCoPCommand())
      {
         copRegularizationWeightMatrix.set(0, 0, weightScalar * copRegularizationWeight.getX());
         copRegularizationWeightMatrix.set(1, 1, weightScalar * copRegularizationWeight.getY());
         copRateRegularizationWeightMatrix.set(0, 0, weightScalar * copRateRegularizationWeight.getX());
         copRateRegularizationWeightMatrix.set(1, 1, weightScalar * copRateRegularizationWeight.getY());
      }
      else
      {
         copRegularizationWeightMatrix.zero();
         copRateRegularizationWeightMatrix.zero();
      }

      hasReset.set(resetRequested.getBooleanValue()); // So it is visible from SCS when the reset has been processed.
      resetRequested.set(false);

      // Should not get there as long as the number of contact points of the contactable body is less or equal to maxNumberOfContactPoints.
      for (; rhoIndex < rhoSize; rhoIndex++)
         clear(rhoIndex);
   }

   private final FrameVector3D forceFromRho = new FrameVector3D();

   public double computeCopObjectiveJacobian(DMatrixRMaj jacobianToPack, DMatrixRMaj objectiveToPack, FramePoint2DReadOnly desiredCoP)
   {
      if (desiredCoP.containsNaN())
      {
         jacobianToPack.reshape(2, rhoSize);
         jacobianToPack.zero();
         objectiveToPack.zero();
         return 0.0;
      }

      if (wrenchFromRho.getLinearPart().normSquared() < 1.0e-1)
      {
         jacobianToPack.reshape(2, rhoSize);
         jacobianToPack.zero();
         objectiveToPack.zero();
         return 0.0;
      }

      forceFromRho.setIncludingFrame(wrenchFromRho.getLinearPart());
      forceFromRho.changeFrame(planeFrame);

      if (forceFromRho.getZ() < 1.0e-1)
      {
         jacobianToPack.reshape(2, rhoSize);
         jacobianToPack.zero();
         objectiveToPack.zero();
         return 0.0;
      }

      if (useOldCoPObjectiveFormulation)
      {
         double fzInverse = 1.0 / forceFromRho.getZ();

         // [ -J_ty / F_z_previous ] * rho == x_cop
         int tauYIndex = 1;
         MatrixTools.setMatrixBlock(jacobianToPack, 0, 0, wrenchJacobianInPlaneFrame, tauYIndex, 0, 1, rhoSize, -fzInverse);

         // [  J_tx / F_z_previous] * rho == y_cop
         int tauXIndex = 0;
         MatrixTools.setMatrixBlock(jacobianToPack, 1, 0, wrenchJacobianInPlaneFrame, tauXIndex, 0, 1, rhoSize, fzInverse);

         // Get the objective value
         desiredCoP.get(objectiveToPack);

         return 1.0;
      }
      else
      {
         desiredCoP.checkReferenceFrameMatch(planeFrame);

         copObjectiveCalculator.computeTask(wrenchJacobianInPlaneFrame,
                                            desiredCoP,
                                            rhoSize,
                                            jacobianToPack,
                                            objectiveToPack);

         return 1.0 / MathTools.square(forceFromRho.getZ());
      }

   }

   private void clear(int rhoIndex)
   {
      FramePoint3D basisVectorOrigin = basisVectorsOrigin[rhoIndex];
      FrameVector3D basisVector = basisVectors[rhoIndex];

      basisVectorOrigin.setToZero(centerOfMassFrame);
      basisVector.setToZero(centerOfMassFrame);

      rhoMatrix.set(rhoIndex, 0, 0.0);
      rhoMaxMatrix.set(rhoIndex, 0, Double.POSITIVE_INFINITY);
      rhoWeightMatrix.set(rhoIndex, rhoIndex, 1.0); // FIXME why is this setting to 1.0????
      rhoRateWeightMatrix.set(rhoIndex, rhoIndex, 0.0);
   }

   private final Wrench wrenchFromRho = new Wrench();
   private final DMatrixRMaj totalWrenchMatrix = new DMatrixRMaj(SpatialForce.SIZE, 1);

   public void computeWrenchFromRho(int startIndex, DMatrixRMaj allRobotRho)
   {
      CommonOps_DDRM.extract(allRobotRho, startIndex, startIndex + rhoSize, 0, 1, rhoMatrix, 0, 0);

      ReferenceFrame bodyFixedFrame = getRigidBody().getBodyFixedFrame();
      if (yoPlaneContactState.inContact())
      {
         CommonOps_DDRM.mult(wrenchJacobianInPlaneFrame, rhoMatrix, totalWrenchMatrix);
         wrenchFromRho.setIncludingFrame(bodyFixedFrame, planeFrame, totalWrenchMatrix);

         previousCoP.setX(-wrenchFromRho.getAngularPartY() / wrenchFromRho.getLinearPartZ());
         previousCoP.setY(wrenchFromRho.getAngularPartX() / wrenchFromRho.getLinearPartZ());
      }
      else
      {
         wrenchFromRho.setToZero(bodyFixedFrame, planeFrame);
         previousCoP.setToZero();
      }
   }

   public WrenchReadOnly getWrench()
   {
      return wrenchFromRho;
   }

   private final SpatialForce unitSpatialForceVector = new SpatialForce();

   public void computeWrenchJacobianInFrame(ReferenceFrame frame, DMatrixRMaj matrixToPack)
   {
      matrixToPack.reshape(Wrench.SIZE, rhoSize);
      for (int rhoIndex = 0; rhoIndex < rhoSize; rhoIndex++)
      {
         if (rhoEnabled[rhoIndex].getValue())
         {
            FramePoint3D basisVectorOrigin = basisVectorsOrigin[rhoIndex];
            FrameVector3D basisVector = basisVectors[rhoIndex];
            basisVectorOrigin.changeFrame(frame);
            basisVector.changeFrame(frame);
            unitSpatialForceVector.setIncludingFrame(null, basisVector, basisVectorOrigin);
            unitSpatialForceVector.get(0, rhoIndex, matrixToPack);
         }
         else
         {
            MatrixTools.zeroColumn(rhoIndex, matrixToPack);
         }
      }
   }

   private void computeNormalContactVectorRotation(RotationMatrix normalContactVectorRotationMatrixToPack)
   {
      yoPlaneContactState.getContactNormalFrameVector(contactNormalVector);
      contactNormalVector.changeFrame(planeFrame);
      EuclidGeometryTools.orientation3DFromZUpToVector3D(contactNormalVector, normalContactVectorRotationMatrixToPack);
   }

   private void computeBasisVector(int basisVectorIndex, double rotationOffset, RotationMatrix normalContactVectorRotationMatrix,
                                   FrameVector3D basisVectorToPack)
   {
      double angle = rotationOffset + basisVectorIndex * basisVectorAngleIncrement;
      double mu = yoPlaneContactState.getCoefficientOfFriction();

      // Compute the linear part considering a normal contact vector pointing z-up
      basisVectorToPack.setIncludingFrame(planeFrame, Math.cos(angle) * mu, Math.sin(angle) * mu, 1.0);

      // Transforming the result to consider the actual normal contact vector
      normalContactVectorRotationMatrix.transform(basisVectorToPack);
      basisVectorToPack.normalize();
   }

   public RigidBodyBasics getRigidBody()
   {
      return yoPlaneContactState.getRigidBody();
   }

   public int getRhoSize()
   {
      return rhoSize;
   }

   public DMatrixRMaj getLastRho()
   {
      return rhoMatrix;
   }

   public DMatrixRMaj getRhoJacobian()
   {
      return wrenchJacobianInCoMFrame;
   }

   public DMatrixRMaj getActiveRhoMatrix()
   {
      return activeRhoMatrix;
   }

   public DMatrixRMaj getRhoMax()
   {
      return rhoMaxMatrix;
   }

   public DMatrixRMaj getRhoWeight()
   {
      return rhoWeightMatrix;
   }

   public DMatrixRMaj getRhoRateWeight()
   {
      return rhoRateWeightMatrix;
   }

   public Wrench getWrenchFromRho()
   {
      return wrenchFromRho;
   }

   public DMatrixRMaj getCoPRegularizationJacobian()
   {
      return copRegularizationJacobian;
   }

   public DMatrixRMaj getCoPRegularizationObjective()
   {
      return copRegularizationObjective;
   }

   public DMatrixRMaj getCoPRateRegularizationJacobian()
   {
      return copRateRegularizationJacobian;
   }

   public DMatrixRMaj getCoPRateRegularizationObjective()
   {
      return copRateRegularizationObjective;
   }

   public DMatrixRMaj getCoPRegularizationWeight()
   {
      return copRegularizationWeightMatrix;
   }

   public DMatrixRMaj getCoPRateRegularizationWeight()
   {
      return copRateRegularizationWeightMatrix;
   }

   public FramePoint2DReadOnly getDesiredCoP()
   {
      return desiredCoP;
   }

   public FramePoint3D[] getBasisVectorsOrigin()
   {
      return basisVectorsOrigin;
   }

   public FrameVector3D[] getBasisVectors()
   {
      return basisVectors;
   }

   public boolean hasReset()
   {
      return hasReset.getBooleanValue();
   }

   public DMatrixRMaj getWrenchJacobianMatrix()
   {
      return wrenchJacobianInPlaneFrame;
   }

   public ReferenceFrame getPlaneFrame()
   {
      return planeFrame;
   }

   public boolean canHandleCoPCommand()
   {
      return yoPlaneContactState.getFootholdArea() > minFootholdSizeForCoPObjectives;
   }
}
