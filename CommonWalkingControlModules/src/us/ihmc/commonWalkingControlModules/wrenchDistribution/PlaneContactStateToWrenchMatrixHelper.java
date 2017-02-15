package us.ihmc.commonWalkingControlModules.wrenchDistribution;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Matrix3d;
import javax.vecmath.Vector2d;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoContactPoint;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.CenterOfPressureCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.PlaneContactStateCommand;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.LongYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.GeometryTools;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.math.frames.YoMatrix;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SpatialForceVector;
import us.ihmc.robotics.screwTheory.Wrench;

public class PlaneContactStateToWrenchMatrixHelper
{
   private final int maxNumberOfContactPoints;
   private final int numberOfBasisVectorsPerContactPoint;
   private final double basisVectorAngleIncrement;

   private final int rhoSize;

   private final DenseMatrix64F rhoMatrix;
   private final DenseMatrix64F rhoJacobianMatrix;
   private final DenseMatrix64F copJacobianMatrix;

   private final DenseMatrix64F desiredCoPMatrix = new DenseMatrix64F(2, 1);
   private final DenseMatrix64F previousCoPMatrix = new DenseMatrix64F(2, 1);

   private final DenseMatrix64F rhoWeightMatrix;
   private final DenseMatrix64F rhoRateWeightMatrix;
   private final DenseMatrix64F desiredCoPWeightMatrix = new DenseMatrix64F(2, 2);
   private final DenseMatrix64F copRateWeightMatrix = new DenseMatrix64F(2, 2);

   private final YoPlaneContactState yoPlaneContactState;

   private final LongYoVariable lastCommandId;
   private final BooleanYoVariable hasReset;
   private final BooleanYoVariable resetRequested;

   private final YoMatrix yoRho;

   private final FrameVector contactNormalVector = new FrameVector();
   private final AxisAngle4d normalContactVectorRotation = new AxisAngle4d();

   private final ReferenceFrame centerOfMassFrame;
   private final ReferenceFrame planeFrame;

   private final YoFramePoint desiredCoP;
   private final YoFramePoint previousCoP;

   private final BooleanYoVariable hasReceivedCenterOfPressureCommand;
   private final YoFramePoint2d desiredCoPCommandInSoleFrame;
   private final Vector2d desiredCoPCommandWeightInSoleFrame = new Vector2d();

   private final List<FramePoint> basisVectorsOrigin = new ArrayList<>();
   private final List<FrameVector> basisVectors = new ArrayList<>();

   private final Matrix3d normalContactVectorRotationMatrix = new Matrix3d();

   public PlaneContactStateToWrenchMatrixHelper(ContactablePlaneBody contactablePlaneBody, ReferenceFrame centerOfMassFrame, int maxNumberOfContactPoints,
         int numberOfBasisVectorsPerContactPoint, YoVariableRegistry parentRegistry)
   {
      List<FramePoint2d> contactPoints2d = contactablePlaneBody.getContactPoints2d();

      if (contactPoints2d.size() > maxNumberOfContactPoints)
         throw new RuntimeException("Unexpected number of contact points: " + contactPoints2d.size());

      this.centerOfMassFrame = centerOfMassFrame;
      this.maxNumberOfContactPoints = maxNumberOfContactPoints;
      this.numberOfBasisVectorsPerContactPoint = numberOfBasisVectorsPerContactPoint;

      rhoSize = maxNumberOfContactPoints * numberOfBasisVectorsPerContactPoint;
      basisVectorAngleIncrement = 2.0 * Math.PI / numberOfBasisVectorsPerContactPoint;

      rhoMatrix = new DenseMatrix64F(rhoSize, 1);
      rhoJacobianMatrix = new DenseMatrix64F(SpatialForceVector.SIZE, rhoSize);
      copJacobianMatrix = new DenseMatrix64F(2, rhoSize);

      rhoWeightMatrix = new DenseMatrix64F(rhoSize, rhoSize);
      rhoRateWeightMatrix = new DenseMatrix64F(rhoSize, rhoSize);

      String bodyName = contactablePlaneBody.getName();
      String namePrefix = bodyName + "WrenchMatrixHelper";
      YoVariableRegistry registry = new YoVariableRegistry(namePrefix);

      RigidBody rigidBody = contactablePlaneBody.getRigidBody();
      planeFrame = contactablePlaneBody.getSoleFrame();
      yoPlaneContactState = new YoPlaneContactState(namePrefix, rigidBody, planeFrame, contactPoints2d, 0.0, registry);

      hasReset = new BooleanYoVariable(namePrefix + "HasReset", registry);
      resetRequested = new BooleanYoVariable(namePrefix + "ResetRequested", registry);
      lastCommandId = new LongYoVariable(namePrefix + "LastCommandId", registry);

      hasReceivedCenterOfPressureCommand = new BooleanYoVariable(namePrefix + "HasReceivedCoPCommand", registry);
      desiredCoPCommandInSoleFrame = new YoFramePoint2d(namePrefix + "DesiredCoPCommand", planeFrame, registry);
      
      yoRho = new YoMatrix(namePrefix + "Rho", rhoSize, 1, registry);

      for (int i = 0; i < rhoSize; i++)
      {
         basisVectors.add(new FrameVector(centerOfMassFrame));
         basisVectorsOrigin.add(new FramePoint(centerOfMassFrame));
      }

      desiredCoP = new YoFramePoint(namePrefix + "DesiredCoP", planeFrame, registry);
      previousCoP = new YoFramePoint(namePrefix + "PreviousCoP", planeFrame, registry);
      ReferenceFrame bodyFixedFrame = rigidBody.getBodyFixedFrame();
      wrenchFromRho.setToZero(bodyFixedFrame, centerOfMassFrame);

      parentRegistry.addChild(registry);
   }

   public void setPlaneContactStateCommand(PlaneContactStateCommand command)
   {
      yoPlaneContactState.updateFromPlaneContactStateCommand(command);
      yoPlaneContactState.computeSupportPolygon();

      if (lastCommandId.getLongValue() != command.getId())
      {
         resetRequested.set(true);
         lastCommandId.set(command.getId());
      }
   }

   public void setCenterOfPressureCommand(CenterOfPressureCommand command)
   {
      desiredCoPCommandInSoleFrame.set(command.getDesiredCoPInSoleFrame());
      desiredCoPCommandWeightInSoleFrame.set(command.getWeightInSoleFrame());
      hasReceivedCenterOfPressureCommand.set(true);
   }

   public void computeMatrices(double rhoWeight, double rhoRateWeight, Vector2d desiredCoPWeight, Vector2d copRateWeight)
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

         for (int basisVectorIndex = 0; basisVectorIndex < numberOfBasisVectorsPerContactPoint; basisVectorIndex++)
         {
            if (inContact)
            {
               FramePoint basisVectorOrigin = basisVectorsOrigin.get(rhoIndex);
               FrameVector basisVector = basisVectors.get(rhoIndex);

               contactPoint.getPosition(basisVectorOrigin);
               computeBasisVector(basisVectorIndex, normalContactVectorRotationMatrix, basisVector);

               DenseMatrix64F singleRhoJacobian = computeSingleRhoJacobian(basisVectorOrigin, basisVector);
               CommonOps.insert(singleRhoJacobian, rhoJacobianMatrix, 0, rhoIndex);

               DenseMatrix64F singleRhoCoPJacobian = computeSingleRhoCoPJacobian(basisVectorOrigin, basisVector);
               CommonOps.insert(singleRhoCoPJacobian, copJacobianMatrix, 0, rhoIndex);

               rhoWeightMatrix.set(rhoIndex, rhoIndex, rhoWeight * (double) maxNumberOfContactPoints / (double) numberOfContactPointsInContact);

               if (resetRequested.getBooleanValue())
                  rhoRateWeightMatrix.set(rhoIndex, rhoIndex, 0.0);
               else
                  rhoRateWeightMatrix.set(rhoIndex, rhoIndex, rhoRateWeight);
            }
            else
            {
               clear(rhoIndex);
            }

            rhoIndex++;
         }
      }

      boolean isFootholdAreaLargeEnough = yoPlaneContactState.getFootholdArea() > 1.0e-3;

      if (yoPlaneContactState.inContact() && !resetRequested.getBooleanValue() && isFootholdAreaLargeEnough)
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
      FramePoint basisVectorOrigin = basisVectorsOrigin.get(rhoIndex);
      FrameVector basisVector = basisVectors.get(rhoIndex);

      basisVectorOrigin.setToZero(centerOfMassFrame);
      basisVector.setToZero(centerOfMassFrame);

      for (int row = 0; row < Wrench.SIZE; row++)
         rhoJacobianMatrix.set(row, rhoIndex, 0.0);

      for (int row = 0; row < 2; row++)
         copJacobianMatrix.set(row, rhoIndex, 0.0);

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

   private void computeNormalContactVectorRotation(Matrix3d normalContactVectorRotationMatrixToPack)
   {
      yoPlaneContactState.getContactNormalFrameVector(contactNormalVector);
      contactNormalVector.changeFrame(planeFrame);
      contactNormalVector.normalize();
      GeometryTools.getAxisAngleFromZUpToVector(contactNormalVector.getVector(), normalContactVectorRotation);
      normalContactVectorRotationMatrixToPack.set(normalContactVectorRotation);
   }

   private void computeBasisVector(int basisVectorIndex, Matrix3d normalContactVectorRotationMatrix, FrameVector basisVectorToPack)
   {
      double angle = basisVectorIndex * basisVectorAngleIncrement;
      double mu = yoPlaneContactState.getCoefficientOfFriction();

      // Compute the linear part considering a normal contact vector pointing z-up
      basisVectorToPack.setIncludingFrame(planeFrame, Math.cos(angle) * mu, Math.sin(angle) * mu, 1.0);

      // Transforming the result to consider the actual normal contact vector
      normalContactVectorRotationMatrix.transform(basisVectorToPack.getVector());
      basisVectorToPack.normalize();
   }

   private final SpatialForceVector unitSpatialForceVector = new SpatialForceVector();
   private final DenseMatrix64F singleRhoJacobian = new DenseMatrix64F(SpatialForceVector.SIZE, 1);

   private DenseMatrix64F computeSingleRhoJacobian(FramePoint basisVectorOrigin, FrameVector basisVector)
   {
      basisVectorOrigin.changeFrame(centerOfMassFrame);
      basisVector.changeFrame(centerOfMassFrame);

      // Compute the unit wrench corresponding to the basis vector
      unitSpatialForceVector.setIncludingFrame(basisVector, basisVectorOrigin);
      unitSpatialForceVector.getMatrix(singleRhoJacobian);
      return singleRhoJacobian;
   }

   private final FrameVector forceFromRho = new FrameVector();
   private final DenseMatrix64F singleRhoCoPJacobian = new DenseMatrix64F(2, 1);

   private DenseMatrix64F computeSingleRhoCoPJacobian(FramePoint basisVectorOrigin, FrameVector basisVector)
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

   public List<FramePoint> getBasisVectorsOrigin()
   {
      return basisVectorsOrigin;
   }

   public List<FrameVector> getBasisVectors()
   {
      return basisVectors;
   }
}
