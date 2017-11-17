package us.ihmc.commonWalkingControlModules.wrenchDistribution;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoContactPoint;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.CenterOfPressureCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.PlaneContactStateCommand;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.math.frames.YoMatrix;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SpatialForceVector;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class PlaneContactStateToWrenchMatrixHelper
{
   /**
    * This is used when determining whether two contact points are at the same location. If that is the case
    * one of them will rotate it's friction cone approximation to get better coverage of the cone through the
    * basis vectors.
    */
   private static final double distanceThresholdBetweenTwoContactPoint = 0.01;

   private final int maxNumberOfContactPoints;
   private final int numberOfBasisVectorsPerContactPoint;
   private final double basisVectorAngleIncrement;

   private final int rhoSize;

   private final DenseMatrix64F rhoMatrix;
   private final DenseMatrix64F rhoJacobianMatrix;
   private final DenseMatrix64F copJacobianMatrix;

   private final DenseMatrix64F desiredCoPMatrix = new DenseMatrix64F(2, 1);
   private final DenseMatrix64F previousCoPMatrix = new DenseMatrix64F(2, 1);

   private final DenseMatrix64F rhoMaxMatrix;
   private final DenseMatrix64F rhoWeightMatrix;
   private final DenseMatrix64F rhoRateWeightMatrix;
   private final DenseMatrix64F desiredCoPWeightMatrix = new DenseMatrix64F(2, 2);
   private final DenseMatrix64F copRateWeightMatrix = new DenseMatrix64F(2, 2);

   private final DenseMatrix64F activeRhoMatrix;

   private final YoPlaneContactState yoPlaneContactState;

   private final YoBoolean hasReset;
   private final YoBoolean resetRequested;

   private final YoMatrix yoRho;

   private final FrameVector3D contactNormalVector = new FrameVector3D();
   private final AxisAngle normalContactVectorRotation = new AxisAngle();

   private final ReferenceFrame centerOfMassFrame;
   private final ReferenceFrame planeFrame;

   private final YoFramePoint desiredCoP;
   private final YoFramePoint previousCoP;

   private final YoBoolean hasReceivedCenterOfPressureCommand;
   private final YoBoolean isFootholdAreaLargeEnough;
   private final YoBoolean deactivateRhoWhenNotInContact;
   private final YoFramePoint2d desiredCoPCommandInSoleFrame;
   private final Vector2D desiredCoPCommandWeightInSoleFrame = new Vector2D();

   private final List<FramePoint3D> basisVectorsOrigin = new ArrayList<>();
   private final List<FrameVector3D> basisVectors = new ArrayList<>();
   private final HashMap<YoContactPoint, YoDouble> maxContactForces = new HashMap<>();
   private final HashMap<YoContactPoint, YoDouble> rhoWeights = new HashMap<>();

   private final RotationMatrix normalContactVectorRotationMatrix = new RotationMatrix();

   private final FramePoint2D contactPoint2d = new FramePoint2D();
   private final FrictionConeRotationCalculator coneRotationCalculator;

   public PlaneContactStateToWrenchMatrixHelper(ContactablePlaneBody contactablePlaneBody, ReferenceFrame centerOfMassFrame, int maxNumberOfContactPoints,
                                                int numberOfBasisVectorsPerContactPoint, FrictionConeRotationCalculator coneRotationCalculator,
                                                YoVariableRegistry parentRegistry)
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

      rhoMatrix = new DenseMatrix64F(rhoSize, 1);
      rhoJacobianMatrix = new DenseMatrix64F(SpatialForceVector.SIZE, rhoSize);
      copJacobianMatrix = new DenseMatrix64F(2, rhoSize);

      rhoMaxMatrix = new DenseMatrix64F(rhoSize, 1);
      rhoWeightMatrix = new DenseMatrix64F(rhoSize, rhoSize);
      rhoRateWeightMatrix = new DenseMatrix64F(rhoSize, rhoSize);

      activeRhoMatrix = new DenseMatrix64F(rhoSize, 1);
      CommonOps.fill(activeRhoMatrix, 1.0);

      CommonOps.fill(rhoMaxMatrix, Double.POSITIVE_INFINITY);

      String bodyName = contactablePlaneBody.getName();
      String namePrefix = bodyName + "WrenchMatrixHelper";
      YoVariableRegistry registry = new YoVariableRegistry(namePrefix);

      RigidBody rigidBody = contactablePlaneBody.getRigidBody();
      planeFrame = contactablePlaneBody.getSoleFrame();
      yoPlaneContactState = new YoPlaneContactState(namePrefix, rigidBody, planeFrame, contactPoints2d, 0.0, registry);
      yoPlaneContactState.clear();
      yoPlaneContactState.computeSupportPolygon();

      hasReset = new YoBoolean(namePrefix + "HasReset", registry);
      resetRequested = new YoBoolean(namePrefix + "ResetRequested", registry);
      deactivateRhoWhenNotInContact = new YoBoolean(namePrefix + "DeactivateRhoWhenNotInContact", registry);

      for (int i = 0; i < contactPoints2d.size(); i++)
      {
         YoDouble rhoWeight = new YoDouble(namePrefix + "RhoWeight" + i, registry);
         YoDouble maxContactForce = new YoDouble(namePrefix + "MaxContactForce" + i, registry);
         maxContactForce.set(Double.POSITIVE_INFINITY);

         rhoWeights.put(yoPlaneContactState.getContactPoints().get(i), rhoWeight);
         maxContactForces.put(yoPlaneContactState.getContactPoints().get(i), maxContactForce);
      }

      hasReceivedCenterOfPressureCommand = new YoBoolean(namePrefix + "HasReceivedCoPCommand", registry);
      isFootholdAreaLargeEnough = new YoBoolean(namePrefix + "isFootholdAreaLargeEnough", registry);
      desiredCoPCommandInSoleFrame = new YoFramePoint2d(namePrefix + "DesiredCoPCommand", planeFrame, registry);

      yoRho = new YoMatrix(namePrefix + "Rho", rhoSize, 1, registry);

      for (int i = 0; i < rhoSize; i++)
      {
         basisVectors.add(new FrameVector3D(centerOfMassFrame));
         basisVectorsOrigin.add(new FramePoint3D(centerOfMassFrame));
      }

      desiredCoP = new YoFramePoint(namePrefix + "DesiredCoP", planeFrame, registry);
      previousCoP = new YoFramePoint(namePrefix + "PreviousCoP", planeFrame, registry);
      ReferenceFrame bodyFixedFrame = rigidBody.getBodyFixedFrame();
      wrenchFromRho.setToZero(bodyFixedFrame, centerOfMassFrame);

      parentRegistry.addChild(registry);
   }

   public void setDeactivateRhoWhenNotInContact(boolean deactivateRhoWhenNotInContact)
   {
      this.deactivateRhoWhenNotInContact.set(deactivateRhoWhenNotInContact);
   }

   public void setPlaneContactStateCommand(PlaneContactStateCommand command)
   {
      yoPlaneContactState.updateFromPlaneContactStateCommand(command);
      yoPlaneContactState.computeSupportPolygon();

      if (yoPlaneContactState.pollContactHasChangedNotification())
      {
         resetRequested.set(true);
      }

      for (int i = 0; i < command.getNumberOfContactPoints(); i++)
      {
         rhoWeights.get(yoPlaneContactState.getContactPoints().get(i)).set(command.getRhoWeight(i));
         if (command.hasMaxContactPointNormalForce())
         {
            maxContactForces.get(yoPlaneContactState.getContactPoints().get(i)).set(command.getMaxContactPointNormalForce(i));
         }
      }
   }

   public void setCenterOfPressureCommand(CenterOfPressureCommand command)
   {
      desiredCoPCommandInSoleFrame.set(command.getDesiredCoPInSoleFrame());
      desiredCoPCommandWeightInSoleFrame.set(command.getWeightInSoleFrame());
      hasReceivedCenterOfPressureCommand.set(true);
   }

   public void computeMatrices(double defaultRhoWeight, double rhoRateWeight, Vector2D desiredCoPWeight, Vector2D copRateWeight)
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

         // in case the contact point is close to another point rotate it
         if (inContact)
         {
            int matches = 0;
            for (int j = contactPointIndex + 1; j < contactPoints.size(); j++)
            {
               YoContactPoint candidateForMatch = contactPoints.get(j);
               candidateForMatch.getPosition2d(contactPoint2d);
               if (candidateForMatch.isInContact() && contactPoint.epsilonEquals(contactPoint2d, distanceThresholdBetweenTwoContactPoint))
               {
                  matches++;
               }
            }
            // TODO: If there are more then two contacts in the same spot we should probably disable them.
            if (matches > 0)
            {
               angleOffset += basisVectorAngleIncrement / 2.0;
            }
         }

         for (int basisVectorIndex = 0; basisVectorIndex < numberOfBasisVectorsPerContactPoint; basisVectorIndex++)
         {
            FramePoint3D basisVectorOrigin = basisVectorsOrigin.get(rhoIndex);
            FrameVector3D basisVector = basisVectors.get(rhoIndex);

            if (inContact)
            {
               contactPoint.getPosition(basisVectorOrigin);
               computeBasisVector(basisVectorIndex, angleOffset, normalContactVectorRotationMatrix, basisVector);

               DenseMatrix64F singleRhoJacobian = computeSingleRhoJacobian(basisVectorOrigin, basisVector);
               CommonOps.insert(singleRhoJacobian, rhoJacobianMatrix, 0, rhoIndex);

               DenseMatrix64F singleRhoCoPJacobian = computeSingleRhoCoPJacobian(basisVectorOrigin, basisVector);
               CommonOps.insert(singleRhoCoPJacobian, copJacobianMatrix, 0, rhoIndex);

               double rhoWeight = rhoWeights.get(yoPlaneContactState.getContactPoints().get(contactPointIndex)).getDoubleValue();
               if(Double.isNaN(rhoWeight))
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
            rhoMaxMatrix.set(rhoIndex, 0, maxContactForces.get(yoPlaneContactState.getContactPoints().get(contactPointIndex)).getDoubleValue() / numberOfBasisVectorsPerContactPoint);

            rhoIndex++;
         }

      }

      isFootholdAreaLargeEnough.set(yoPlaneContactState.getFootholdArea() > 1.0e-3);
      if (yoPlaneContactState.inContact() && !resetRequested.getBooleanValue() && isFootholdAreaLargeEnough.getBooleanValue())
      {
         if (hasReceivedCenterOfPressureCommand.getBooleanValue())
         {
            desiredCoPMatrix.set(0, 0, desiredCoPCommandInSoleFrame.getX());
            desiredCoPMatrix.set(1, 0, desiredCoPCommandInSoleFrame.getY());
            desiredCoPWeightMatrix.set(0, 0, desiredCoPCommandWeightInSoleFrame.getX());
            desiredCoPWeightMatrix.set(1, 1, desiredCoPCommandWeightInSoleFrame.getY());

            hasReceivedCenterOfPressureCommand.set(false);
         }
         else
         {
            // // FIXME: 6/5/17 Is this ever even used now?
            desiredCoPMatrix.set(0, 0, desiredCoP.getX());
            desiredCoPMatrix.set(1, 0, desiredCoP.getY());
            desiredCoPWeightMatrix.set(0, 0, desiredCoPWeight.getX());
            desiredCoPWeightMatrix.set(1, 1, desiredCoPWeight.getY());
         }
         copRateWeightMatrix.set(0, 0, copRateWeight.getX());
         copRateWeightMatrix.set(1, 1, copRateWeight.getY());
      }
      else
      {
         desiredCoPMatrix.zero();
         desiredCoPWeightMatrix.zero();
         copRateWeightMatrix.zero();
      }

      hasReset.set(resetRequested.getBooleanValue()); // So it is visible from SCS when the reset has been processed.
      resetRequested.set(false);

      // Should not get there as long as the number of contact points of the contactable body is less or equal to maxNumberOfContactPoints.
      for (; rhoIndex < rhoSize; rhoIndex++)
         clear(rhoIndex);
   }

   private void clear(int rhoIndex)
   {
      FramePoint3D basisVectorOrigin = basisVectorsOrigin.get(rhoIndex);
      FrameVector3D basisVector = basisVectors.get(rhoIndex);

      basisVectorOrigin.setToZero(centerOfMassFrame);
      basisVector.setToZero(centerOfMassFrame);

      for (int row = 0; row < Wrench.SIZE; row++)
         rhoJacobianMatrix.set(row, rhoIndex, 0.0);

      for (int row = 0; row < 2; row++)
         copJacobianMatrix.set(row, rhoIndex, 0.0);

      rhoMaxMatrix.set(rhoIndex, 0, Double.POSITIVE_INFINITY);
      rhoWeightMatrix.set(rhoIndex, rhoIndex, 1.0);
      rhoRateWeightMatrix.set(rhoIndex, rhoIndex, 0.0);
   }

   private final Wrench wrenchFromRho = new Wrench();
   private final DenseMatrix64F totalWrenchMatrix = new DenseMatrix64F(SpatialForceVector.SIZE, 1);
   private final DenseMatrix64F singleRhoWrenchMatrix = new DenseMatrix64F(SpatialForceVector.SIZE, 1);

   public void computeWrenchFromRho(int startIndex, DenseMatrix64F allRobotRho)
   {
      CommonOps.extract(allRobotRho, startIndex, startIndex + rhoSize, 0, 1, rhoMatrix, 0, 0);
      yoRho.set(rhoMatrix);

      if (yoPlaneContactState.inContact())
      {
         totalWrenchMatrix.zero();

         for (int rhoIndex = 0; rhoIndex < rhoSize; rhoIndex++)
         {
            double rho = rhoMatrix.get(rhoIndex, 0);
            CommonOps.extract(rhoJacobianMatrix, 0, SpatialForceVector.SIZE, rhoIndex, rhoIndex + 1, singleRhoWrenchMatrix, 0, 0);
            MatrixTools.addMatrixBlock(totalWrenchMatrix, 0, 0, singleRhoWrenchMatrix, 0, 0, SpatialForceVector.SIZE, 1, rho);
         }

         RigidBody rigidBody = yoPlaneContactState.getRigidBody();
         ReferenceFrame bodyFixedFrame = rigidBody.getBodyFixedFrame();

         wrenchFromRho.set(bodyFixedFrame, centerOfMassFrame, totalWrenchMatrix);

         CommonOps.mult(copJacobianMatrix, rhoMatrix, previousCoPMatrix);
         previousCoP.setX(previousCoPMatrix.get(0, 0));
         previousCoP.setY(previousCoPMatrix.get(1, 0));
      }
      else
      {
         wrenchFromRho.setToZero();
      }
   }

   private void computeNormalContactVectorRotation(RotationMatrix normalContactVectorRotationMatrixToPack)
   {
      yoPlaneContactState.getContactNormalFrameVector(contactNormalVector);
      contactNormalVector.changeFrame(planeFrame);
      contactNormalVector.normalize();
      EuclidGeometryTools.axisAngleFromZUpToVector3D(contactNormalVector.getVector(), normalContactVectorRotation);
      normalContactVectorRotationMatrixToPack.set(normalContactVectorRotation);
   }

   private void computeBasisVector(int basisVectorIndex, double rotationOffset, RotationMatrix normalContactVectorRotationMatrix, FrameVector3D basisVectorToPack)
   {
      double angle = rotationOffset + basisVectorIndex * basisVectorAngleIncrement;
      double mu = yoPlaneContactState.getCoefficientOfFriction();

      // Compute the linear part considering a normal contact vector pointing z-up
      basisVectorToPack.setIncludingFrame(planeFrame, Math.cos(angle) * mu, Math.sin(angle) * mu, 1.0);

      // Transforming the result to consider the actual normal contact vector
      normalContactVectorRotationMatrix.transform(basisVectorToPack.getVector());
      basisVectorToPack.normalize();
   }

   private final SpatialForceVector unitSpatialForceVector = new SpatialForceVector();
   private final DenseMatrix64F singleRhoJacobian = new DenseMatrix64F(SpatialForceVector.SIZE, 1);

   private DenseMatrix64F computeSingleRhoJacobian(FramePoint3D basisVectorOrigin, FrameVector3D basisVector)
   {
      basisVectorOrigin.changeFrame(centerOfMassFrame);
      basisVector.changeFrame(centerOfMassFrame);

      // Compute the unit wrench corresponding to the basis vector
      unitSpatialForceVector.setIncludingFrame(basisVector, basisVectorOrigin);
      unitSpatialForceVector.getMatrix(singleRhoJacobian);
      return singleRhoJacobian;
   }

   private final FrameVector3D forceFromRho = new FrameVector3D();
   private final DenseMatrix64F singleRhoCoPJacobian = new DenseMatrix64F(2, 1);

   private DenseMatrix64F computeSingleRhoCoPJacobian(FramePoint3D basisVectorOrigin, FrameVector3D basisVector)
   {
      wrenchFromRho.getLinearPartIncludingFrame(forceFromRho);
      forceFromRho.changeFrame(planeFrame);

      if (forceFromRho.getZ() > 1.0e-1)
      {
         basisVectorOrigin.changeFrame(planeFrame);
         basisVector.changeFrame(planeFrame);

         unitSpatialForceVector.setIncludingFrame(basisVector, basisVectorOrigin);

         singleRhoCoPJacobian.set(0, 0, -unitSpatialForceVector.getAngularPartY() / forceFromRho.getZ());
         singleRhoCoPJacobian.set(1, 0, unitSpatialForceVector.getAngularPartX() / forceFromRho.getZ());
      }
      else
      {
         singleRhoCoPJacobian.zero();
      }

      return singleRhoCoPJacobian;
   }

   public RigidBody getRigidBody()
   {
      return yoPlaneContactState.getRigidBody();
   }

   public int getRhoSize()
   {
      return rhoSize;
   }

   public DenseMatrix64F getLastRho()
   {
      return rhoMatrix;
   }

   public DenseMatrix64F getRhoJacobian()
   {
      return rhoJacobianMatrix;
   }

   public DenseMatrix64F getActiveRhoMatrix()
   {
      return activeRhoMatrix;
   }

   public DenseMatrix64F getRhoMax()
   {
      return rhoMaxMatrix;
   }

   public DenseMatrix64F getRhoWeight()
   {
      return rhoWeightMatrix;
   }

   public DenseMatrix64F getRhoRateWeight()
   {
      return rhoRateWeightMatrix;
   }

   public Wrench getWrenchFromRho()
   {
      return wrenchFromRho;
   }

   public DenseMatrix64F getCopJacobianMatrix()
   {
      return copJacobianMatrix;
   }

   public DenseMatrix64F getDesiredCoPMatrix()
   {
      return desiredCoPMatrix;
   }

   public DenseMatrix64F getPreviousCoPMatrix()
   {
      return previousCoPMatrix;
   }

   public DenseMatrix64F getDesiredCoPWeightMatrix()
   {
      return desiredCoPWeightMatrix;
   }

   public DenseMatrix64F getCoPRateWeightMatrix()
   {
      return copRateWeightMatrix;
   }

   public List<FramePoint3D> getBasisVectorsOrigin()
   {
      return basisVectorsOrigin;
   }

   public List<FrameVector3D> getBasisVectors()
   {
      return basisVectors;
   }

   public boolean hasReset()
   {
      return hasReset.getBooleanValue();
   }
}
