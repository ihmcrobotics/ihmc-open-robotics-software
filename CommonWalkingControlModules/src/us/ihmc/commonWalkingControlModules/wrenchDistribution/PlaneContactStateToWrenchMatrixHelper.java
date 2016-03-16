package us.ihmc.commonWalkingControlModules.wrenchDistribution;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Matrix3d;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoContactPoint;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
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

   private final DenseMatrix64F rhoMinMatrix;
   private final DenseMatrix64F rhoWeightMatrix;
   private final DenseMatrix64F rhoRateWeightMatrix;

   private final YoPlaneContactState yoPlaneContactState;

   private final LongYoVariable lastCommandId;
   private final BooleanYoVariable resetRhoRateRegularization;

   private final YoMatrix yoRho;

   private final FrameVector contactNormalVector = new FrameVector();
   private final AxisAngle4d normalContactVectorRotation = new AxisAngle4d();

   private final ReferenceFrame centerOfMassFrame;
   private final ReferenceFrame planeFrame;

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

      rhoMinMatrix = new DenseMatrix64F(rhoSize, 1);
      rhoWeightMatrix = new DenseMatrix64F(rhoSize, rhoSize);
      rhoRateWeightMatrix = new DenseMatrix64F(rhoSize, rhoSize);

      String bodyName = contactablePlaneBody.getName();
      String namePrefix = bodyName + "WrenchMatrixHelper";
      YoVariableRegistry registry = new YoVariableRegistry(namePrefix);

      RigidBody rigidBody = contactablePlaneBody.getRigidBody();
      planeFrame = contactablePlaneBody.getSoleFrame();
      yoPlaneContactState = new YoPlaneContactState(namePrefix, rigidBody, planeFrame, contactPoints2d, 0.0, registry);

      resetRhoRateRegularization = new BooleanYoVariable(namePrefix + "ResetRhoRateRegularization", registry);
      lastCommandId = new LongYoVariable(namePrefix + "LastCommandId", registry);

      yoRho = new YoMatrix(namePrefix + "Rho", rhoSize, 1, registry);

      for (int i = 0; i < rhoSize; i++)
      {
         basisVectors.add(new FrameVector(centerOfMassFrame));
         basisVectorsOrigin.add(new FramePoint(centerOfMassFrame));
      }

      parentRegistry.addChild(registry);
   }

   public void setPlaneContactStateCommand(PlaneContactStateCommand command)
   {
      yoPlaneContactState.updateFromPlaneContactStateCommand(command);

      if (lastCommandId.getLongValue() != command.getId())
      {
         resetRhoRateRegularization.set(true);
         lastCommandId.set(command.getId());
      }
   }

   public void computeMatrices(double rhoMin, double rhoWeight, double rhoRateWeight)
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
            FramePoint basisVectorOrigin = basisVectorsOrigin.get(rhoIndex);
            FrameVector basisVector = basisVectors.get(rhoIndex);

            contactPoint.getPosition(basisVectorOrigin);

            if (inContact)
            {
               DenseMatrix64F basisVectorMatrix = computeBasisVector(basisVectorIndex, normalContactVectorRotationMatrix, basisVectorOrigin, basisVector);
               CommonOps.insert(basisVectorMatrix, rhoJacobianMatrix, 0, rhoIndex);
               rhoMinMatrix.set(rhoIndex, 0, rhoMin);
               rhoWeightMatrix.set(rhoIndex, rhoIndex, rhoWeight * (double) maxNumberOfContactPoints / (double) numberOfContactPointsInContact);

               if (resetRhoRateRegularization.getBooleanValue())
                  rhoRateWeightMatrix.set(rhoIndex, rhoIndex, 0.0);
               else
                  rhoRateWeightMatrix.set(rhoIndex, rhoIndex, rhoRateWeight);
            }
            else
            {
               basisVector.setToZero(centerOfMassFrame);

               for (int row = 0; row < Wrench.SIZE; row++)
                  rhoJacobianMatrix.set(row, rhoIndex, 0.0); // Set the basis vectors of the points not in contact to zero

               rhoMinMatrix.set(rhoIndex, 0, Double.NEGATIVE_INFINITY);
               rhoWeightMatrix.set(rhoIndex, rhoIndex, 0.0);
               rhoRateWeightMatrix.set(rhoIndex, rhoIndex, 0.0);
            }

            rhoIndex++;
         }
      }
      resetRhoRateRegularization.set(false);

      // Should not get there as long as the number of contact poins of the contactable body is less or equal to maxNumberOfContactPoints.
      for (; rhoIndex < rhoSize; rhoIndex++)
      {
         FramePoint basisVectorOrigin = basisVectorsOrigin.get(rhoIndex);
         FrameVector basisVector = basisVectors.get(rhoIndex);

         basisVectorOrigin.setToZero(centerOfMassFrame);
         basisVector.setToZero(centerOfMassFrame);

         for (int row = 0; row < Wrench.SIZE; row++)
            rhoJacobianMatrix.set(row, rhoIndex, 0.0);

         rhoMinMatrix.set(rhoIndex, rhoIndex, Double.NEGATIVE_INFINITY);
         rhoWeightMatrix.set(rhoIndex, rhoIndex, 0.0);
         rhoRateWeightMatrix.set(rhoIndex, rhoIndex, 0.0);
      }
   }

   private final Wrench wrenchFromRho = new Wrench();
   private final DenseMatrix64F totalWrenchMatrix = new DenseMatrix64F(SpatialForceVector.SIZE, 1);
   private final DenseMatrix64F singleRhoWrenchMatrix = new DenseMatrix64F(SpatialForceVector.SIZE, 1);

   public Wrench computeWrenchFromRho(int startIndex, DenseMatrix64F allRobotRho)
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
      }
      else
      {
         wrenchFromRho.setToZero();
      }

      return wrenchFromRho;
   }

   private void computeNormalContactVectorRotation(Matrix3d normalContactVectorRotationMatrixToPack)
   {
      yoPlaneContactState.getContactNormalFrameVector(contactNormalVector);
      contactNormalVector.changeFrame(planeFrame);
      contactNormalVector.normalize();
      GeometryTools.getRotationBasedOnNormal(normalContactVectorRotation, contactNormalVector.getVector());
      normalContactVectorRotationMatrixToPack.set(normalContactVectorRotation);
   }

   private final SpatialForceVector currentBasisSpatialVector = new SpatialForceVector();
   private final DenseMatrix64F currentBasisVectorMatrix = new DenseMatrix64F(SpatialForceVector.SIZE, 1);

   private DenseMatrix64F computeBasisVector(int basisVectorIndex, Matrix3d normalContactVectorRotationMatrix, FramePoint basisVectorOrigin,
         FrameVector basisVectorToPack)
   {
      double angle = basisVectorIndex * basisVectorAngleIncrement;
      double mu = yoPlaneContactState.getCoefficientOfFriction();

      basisVectorOrigin.changeFrame(centerOfMassFrame);

      // Compute the linear part considering a normal contact vector pointing z-up
      basisVectorToPack.setIncludingFrame(planeFrame, Math.cos(angle) * mu, Math.sin(angle) * mu, 1.0);

      // Transforming the result to consider the actual normal contact vector
      normalContactVectorRotationMatrix.transform(basisVectorToPack.getVector());
      basisVectorToPack.normalize();
      basisVectorToPack.changeFrame(centerOfMassFrame);

      // Compute the unit wrench corresponding to the basis vector
      currentBasisSpatialVector.setIncludingFrame(basisVectorToPack, basisVectorOrigin);
      currentBasisSpatialVector.getMatrix(currentBasisVectorMatrix);
      return currentBasisVectorMatrix;
   }

   public RigidBody getRigidBody()
   {
      return yoPlaneContactState.getRigidBody();
   }

   public int getRhoSize()
   {
      return rhoSize;
   }

   public DenseMatrix64F getRhoMin()
   {
      return rhoMinMatrix;
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

   public List<FramePoint> getBasisVectorsOrigin()
   {
      return basisVectorsOrigin;
   }

   public List<FrameVector> getBasisVectors()
   {
      return basisVectors;
   }
}
