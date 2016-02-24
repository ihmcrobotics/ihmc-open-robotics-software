package us.ihmc.commonWalkingControlModules.wrenchDistribution;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import org.apache.commons.lang3.mutable.MutableLong;
import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactPointInterface;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.solver.PlaneContactStateCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.solver.PlaneContactStateCommandPool;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
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

public class PlaneContactWrenchMatrixCalculator
{
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);

   private final IntegerYoVariable numberOfLoadedEndEffectors = new IntegerYoVariable("numberOfLoadedEndEffectors", registry);
   private final DoubleYoVariable wRho = new DoubleYoVariable("wRho", registry);
   private final DoubleYoVariable wRhoSmoother = new DoubleYoVariable("wRhoSmoother", registry);
   private final DoubleYoVariable wRhoPenalizer = new DoubleYoVariable("wRhoPenalizer", registry);
   private final DoubleYoVariable rhoMinScalar = new DoubleYoVariable("rhoMinScalarInAdapter", registry);

   private final ReferenceFrame centerOfMassFrame;
   private final Map<RigidBody, Wrench> wrenches = new LinkedHashMap<RigidBody, Wrench>();

   private final List<YoPlaneContactState> planeContactStates;
   private final Map<RigidBody, YoPlaneContactState> rigidBodyToContactStateMap = new HashMap<>();
   private final Map<YoPlaneContactState, MutableLong> previousCommandIdMap = new HashMap<>();
   private final BooleanYoVariable resetCoPPenalizer = new BooleanYoVariable("resetCoPPenalizer", registry);
   private final Map<YoPlaneContactState, BooleanYoVariable> resetRhoSmootherMap = new HashMap<>();

   private final DenseMatrix64F rhoMin;
   private final DenseMatrix64F qRho;
   private final DenseMatrix64F wRhoMatrix;
   private final DenseMatrix64F wRhoSmootherMatrix;

   private final DenseMatrix64F wRhoPenalizerMatrix;

   private final DoubleYoVariable rhoTotal;

   private final DenseMatrix64F rhoMeanTemp;
   private final YoMatrix rhoMeanYoMatrix;

   private final DenseMatrix64F rhoMeanLoadedEndEffectors;
   private final YoMatrix rhoMeanLoadedEndEffectorsYoMatrix;

   private final int totalRhoSize;
   private final int nSupportVectors;
   private final int nPointsPerPlane;
   private final double supportVectorAngleIncrement;

   // Temporary variables
   private final SpatialForceVector currentBasisVector = new SpatialForceVector();
   private final FrameVector tempContactNormalVector = new FrameVector();
   private final AxisAngle4d normalContactVectorRotation = new AxisAngle4d();
   private final Matrix3d tempNormalContactVectorRotationMatrix = new Matrix3d();
   private final Vector3d tempLinearPart = new Vector3d();
   private final Vector3d tempArm = new Vector3d();
   private final FramePoint tempFramePoint = new FramePoint();

   private final DenseMatrix64F tempSum = new DenseMatrix64F(SpatialForceVector.SIZE, 1);
   private final DenseMatrix64F tempVector = new DenseMatrix64F(SpatialForceVector.SIZE, 1);

   private final Map<Integer, FrameVector> basisVectors = new LinkedHashMap<Integer, FrameVector>();
   private final Map<Integer, FramePoint> contactPoints = new LinkedHashMap<Integer, FramePoint>();
   private FrameVector basisVector;
   private FramePoint contactPoint;

   private final Wrench tempWrench = new Wrench();

   public PlaneContactWrenchMatrixCalculator(ReferenceFrame centerOfMassFrame, int rhoSize, int maxNPointsPerPlane, int maxNSupportVectors, double wRho,
         double wRhoSmoother, double wRhoPenalizer, List<? extends ContactablePlaneBody> contactablePlaneBodies, YoVariableRegistry parentRegistry)
   {
      this.centerOfMassFrame = centerOfMassFrame;

      this.totalRhoSize = rhoSize;
      this.nSupportVectors = maxNSupportVectors;
      this.nPointsPerPlane = maxNPointsPerPlane;
      this.supportVectorAngleIncrement = 2.0 * Math.PI / maxNSupportVectors;

      this.wRho.set(wRho);
      this.wRhoSmoother.set(wRhoSmoother);
      this.wRhoPenalizer.set(wRhoPenalizer);

      rhoMeanTemp = new DenseMatrix64F(rhoSize, 1);
      rhoMeanYoMatrix = new YoMatrix(name + "RhoMean", rhoSize, 1, registry);

      rhoMeanLoadedEndEffectors = new DenseMatrix64F(contactablePlaneBodies.size(), 1);
      rhoMeanLoadedEndEffectorsYoMatrix = new YoMatrix(name + "RhoMeanLoadedEndEffectorsYoMatrix", contactablePlaneBodies.size(), 1, registry);

      rhoTotal = new DoubleYoVariable(name + "RhoTotal", registry);

      qRho = new DenseMatrix64F(Wrench.SIZE, rhoSize);
      rhoMin = new DenseMatrix64F(rhoSize, 1);
      wRhoMatrix = new DenseMatrix64F(rhoSize, rhoSize);
      CommonOps.setIdentity(wRhoMatrix);
      CommonOps.scale(wRho, wRhoMatrix);

      wRhoSmootherMatrix = new DenseMatrix64F(rhoSize, rhoSize);
      CommonOps.setIdentity(wRhoSmootherMatrix);
      CommonOps.scale(wRhoSmoother, wRhoSmootherMatrix);

      wRhoPenalizerMatrix = new DenseMatrix64F(rhoSize, rhoSize);
      CommonOps.setIdentity(wRhoPenalizerMatrix);
      CommonOps.scale(wRhoPenalizer, wRhoPenalizerMatrix);

      for (int i = 0; i < rhoSize; i++)
      {
         basisVectors.put(i, new FrameVector(centerOfMassFrame));
         contactPoints.put(i, new FramePoint(centerOfMassFrame));
      }

      this.planeContactStates = new ArrayList<>(contactablePlaneBodies.size());

      for (int i = 0; i < contactablePlaneBodies.size(); i++)
      {
         ContactablePlaneBody contactablePlaneBody = contactablePlaneBodies.get(i);
         RigidBody rigidBody = contactablePlaneBody.getRigidBody();
         ReferenceFrame soleFrame = contactablePlaneBody.getSoleFrame();
         List<FramePoint2d> contactPoints2d = contactablePlaneBody.getContactPoints2d();
         String namePrefix = soleFrame.getName() + "WrenchCalculator";
         YoPlaneContactState yoPlaneContactState = new YoPlaneContactState(namePrefix, rigidBody, soleFrame, contactPoints2d, 0.0, registry);
         planeContactStates.add(yoPlaneContactState);
         rigidBodyToContactStateMap.put(rigidBody, yoPlaneContactState);
         previousCommandIdMap.put(yoPlaneContactState, new MutableLong(-1L));
         BooleanYoVariable resetRhoSmoother = new BooleanYoVariable(namePrefix + "ResetRhoSmoother", registry);
         resetRhoSmootherMap.put(yoPlaneContactState, resetRhoSmoother);
      }

      for (int i = 0; i < this.planeContactStates.size(); i++)
      {
         PlaneContactState planeContactState = this.planeContactStates.get(i);
         RigidBody rigidBody = planeContactState.getRigidBody();
         ReferenceFrame bodyFixedFrame = rigidBody.getBodyFixedFrame();

         Wrench wrench = new Wrench(bodyFixedFrame, bodyFixedFrame);
         wrenches.put(rigidBody, wrench);
      }

      parentRegistry.addChild(registry);
   }

   public void setPlaneContactStateCommand(PlaneContactStateCommand contactStateCommand)
   {
      YoPlaneContactState planeContactState = rigidBodyToContactStateMap.get(contactStateCommand.getContactingRigidBody());
      planeContactState.updateFromPlaneContactStateCommand(contactStateCommand);
      wRhoSmoother.set(contactStateCommand.getWRhoSmoother());
      MutableLong previousCommandId = previousCommandIdMap.get(planeContactState);
      if (previousCommandId.longValue() != contactStateCommand.getId())
      {
         resetCoPPenalizer.set(true);
         resetRhoSmootherMap.get(planeContactState).set(true);
         previousCommandId.setValue(contactStateCommand.getId());
      }
   }

   public void setPlaneContactStateCommandPool(PlaneContactStateCommandPool contactStateCommandPool)
   {
      for (int i = 0; i < contactStateCommandPool.getNumberOfCommands(); i++)
         setPlaneContactStateCommand(contactStateCommandPool.getCommand(i));
   }

   /**
    * Computes and fills the QRho matrix that contains the unit wrenches and will be used by the solver.
    * Each unit wrench represents one basis vector of a contact point.
    * Also computes the weighting matrices wRho and wRhoSmoother and the rho lower boundary rhoMin.
    */
   public void computeMatrices()
   {
      int iRho = 0;
      numberOfLoadedEndEffectors.set(0);

      for (int i = 0; i < planeContactStates.size(); i++)
      {
         YoPlaneContactState planeContactState = planeContactStates.get(i);

         if (planeContactState.getNumberOfContactPointsInContact() > nPointsPerPlane)
            throw new RuntimeException("Unhandled number of contact points: " + planeContactState.getNumberOfContactPointsInContact());

         if (planeContactState.inContact())
            numberOfLoadedEndEffectors.increment();

         // Compute the orientation of the normal contact vector and the corresponding transformation matrix
         computeNormalContactVectorTransform(planeContactState);

         for (int j = 0; j < planeContactState.getTotalNumberOfContactPoints(); j++)
         {
            ContactPointInterface contactPoint = planeContactState.getContactPoints().get(j);
            boolean isInContact = contactPoint.isInContact();

            for (int k = 0; k < nSupportVectors; k++)
            {
             this.contactPoint = contactPoints.get(iRho);
             contactPoint.getPosition(this.contactPoint);

             basisVector = basisVectors.get(iRho);

               if (isInContact)
               {
                  computeBasisVector(planeContactState, contactPoint, k);

                basisVector.set(currentBasisVector.getLinearPart());

                  currentBasisVector.getMatrixColumn(qRho, iRho);
                  rhoMin.set(iRho, 0, rhoMinScalar.getDoubleValue());
                  wRhoMatrix.set(iRho, iRho, wRho.getDoubleValue());
                  if (resetRhoSmootherMap.get(planeContactState).getBooleanValue())
                     wRhoSmootherMatrix.set(iRho, iRho, 0.0);
                  else
                     wRhoSmootherMatrix.set(iRho, iRho, wRhoSmoother.getDoubleValue());
               }
               else
               {
                  basisVector.setToZero(centerOfMassFrame);

                  rhoMin.set(iRho, 0, 0.0);
                  wRhoMatrix.set(iRho, iRho, 0.0);
                  wRhoSmootherMatrix.set(iRho, iRho, 0.0);

                  for (int m = 0; m < Wrench.SIZE; m++)
                     qRho.set(m, iRho, 0.0); // Set the basis vectors of the points not in contact to zero
               }

               iRho++;
            }
         }
         resetRhoSmootherMap.get(planeContactState).set(false);
      }

      for (int i = iRho; i < totalRhoSize; i++)
         for (int j = 0; j < Wrench.SIZE; j++)
            qRho.set(j, i, 0.0); // Set the basis vectors of the points not in contact to zero   
   }

   private void computeNormalContactVectorTransform(PlaneContactState planeContactState)
   {
      planeContactState.getContactNormalFrameVector(tempContactNormalVector);
      tempContactNormalVector.changeFrame(planeContactState.getPlaneFrame());
      tempContactNormalVector.normalize();
      GeometryTools.getRotationBasedOnNormal(normalContactVectorRotation, tempContactNormalVector.getVector());
      tempNormalContactVectorRotationMatrix.set(normalContactVectorRotation);
   }

   private void computeBasisVector(PlaneContactState planeContactState, ContactPointInterface contactPoint, int k)
   {
      double angle = k * supportVectorAngleIncrement;
      double mu = planeContactState.getCoefficientOfFriction();

      contactPoint.getPosition(tempFramePoint);

      // Compute the linear part considering a normal contact vector pointing up
      tempLinearPart.set(Math.cos(angle) * mu, Math.sin(angle) * mu, 1);

      // Transforming the result to consider the actual normal contact vector
      tempNormalContactVectorRotationMatrix.transform(tempLinearPart);
      tempLinearPart.normalize();

      // Compute the unit wrench corresponding to the basis vector
      tempArm.set(tempFramePoint.getX(), tempFramePoint.getY(), tempFramePoint.getZ());
      currentBasisVector.setUsingArm(planeContactState.getPlaneFrame(), tempLinearPart, tempArm);

      currentBasisVector.changeFrame(centerOfMassFrame);
   }

   private final DenseMatrix64F rhosSingleEndEffector = new DenseMatrix64F(1, 1);

   /**
    * Computes from rho the corresponding wrenches to be applied on the contactable bodies.
    * Also computes the weighting matrix wRhoPenalizer and saves the average of rho for each end effector.
    * @param rho contains the force magnitude to be applied for every basis vector provided. It is computed by the solver.
    * @return Map of the wrenches to be applied on each contactable body.
    */
   public Map<RigidBody, Wrench> computeWrenches(DenseMatrix64F rho)
   {
      int iRho = 0;

      rhoMeanTemp.zero();
      rhoMeanYoMatrix.set(rhoMeanTemp);

      rhoTotal.set(0.0);

      rhoMeanLoadedEndEffectors.zero();
      // Reinintialize wrenches
      for (int i = 0; i < planeContactStates.size(); i++)
      {
         RigidBody rigidBody = planeContactStates.get(i).getRigidBody();
         wrenches.get(rigidBody).setToZero();
      }

      // Compute wrenches
      for (int i = 0; i < planeContactStates.size(); i++)
      {
         PlaneContactState planeContactState = planeContactStates.get(i);

         if (planeContactState.inContact())
         {
            int rhoSizeInContact = planeContactState.getNumberOfContactPointsInContact() * nSupportVectors;
            int rhoSizeTotalForOneEndEffector = planeContactState.getTotalNumberOfContactPoints() * nSupportVectors;
            int iRhoStart = iRho;
            int iRhoFinal = iRhoStart + rhoSizeTotalForOneEndEffector;

            rhosSingleEndEffector.reshape(rhoSizeTotalForOneEndEffector, 1);
            CommonOps.extract(rho, iRhoStart, iRhoFinal, 0, 1, rhosSingleEndEffector, 0, 0);
            double rhosAverage = CommonOps.elementSum(rhosSingleEndEffector) / rhoSizeInContact;
            rhoTotal.add(rhosAverage);
            rhoMeanLoadedEndEffectors.set(i, rhosAverage);

            tempSum.zero();
            for (; iRho < iRhoFinal; iRho++)
            {
               rhoMeanTemp.set(iRho, rhosAverage);
               rhoMeanYoMatrix.set(rhoMeanTemp);

               CommonOps.extract(qRho, 0, SpatialForceVector.SIZE, iRho, iRho + 1, tempVector, 0, 0);
               MatrixTools.addMatrixBlock(tempSum, 0, 0, tempVector, 0, 0, SpatialForceVector.SIZE, 1, rho.get(iRho));
            }

            RigidBody rigidBody = planeContactState.getRigidBody();
            ReferenceFrame bodyFixedFrame = rigidBody.getBodyFixedFrame();

            tempWrench.set(bodyFixedFrame, centerOfMassFrame, tempSum);
            tempWrench.changeFrame(bodyFixedFrame);

            wrenches.get(rigidBody).add(tempWrench);
         }
         else
         {
            iRho = iRho + planeContactState.getTotalNumberOfContactPoints() * nSupportVectors;
         }
      }

      rhoMeanLoadedEndEffectorsYoMatrix.set(rhoMeanLoadedEndEffectors);
      return wrenches;
   }

   private void computeWRhoPenalizerMatrix()
   {
      rhoMeanLoadedEndEffectorsYoMatrix.get(rhoMeanLoadedEndEffectors);

      int iRho = 0;
      for (int i = 0; i < planeContactStates.size(); i++)
      {
         PlaneContactState planeContactState = planeContactStates.get(i);

         if (planeContactState.inContact() && !resetCoPPenalizer.getBooleanValue())
         {
            double penaltyScaling = 0.0;

            if (rhoTotal.getDoubleValue() > 1e-3)
               penaltyScaling = Math.max(0.0, 1.0 - rhoMeanLoadedEndEffectors.get(i) / rhoTotal.getDoubleValue());

            int rhoSizeTotalForOneEndEffector = planeContactState.getTotalNumberOfContactPoints() * nSupportVectors;
            int iRhoFinal = iRho + rhoSizeTotalForOneEndEffector;
            for (; iRho < iRhoFinal; iRho++)
               wRhoPenalizerMatrix.set(iRho, iRho, wRhoPenalizer.getDoubleValue() * penaltyScaling);
         }
         else
         {
            int rhoSizeTotalForOneEndEffector = planeContactState.getTotalNumberOfContactPoints() * nSupportVectors;

            int iRhoFinal = iRho + rhoSizeTotalForOneEndEffector;
            for (; iRho < iRhoFinal; iRho++)
               wRhoPenalizerMatrix.set(iRho, iRho, 0.0);
         }
      }

      resetCoPPenalizer.set(false);
   }

   public void setRhoMinScalar(double rhoMinScalar)
   {
      this.rhoMinScalar.set(rhoMinScalar);
   }

   public DenseMatrix64F getWRho()
   {
      return wRhoMatrix;
   }

   public DenseMatrix64F getWRhoSmoother()
   {
      return wRhoSmootherMatrix;
   }

   public DenseMatrix64F getWRhoPenalizer()
   {
      computeWRhoPenalizerMatrix();
      return wRhoPenalizerMatrix;
   }

   public DenseMatrix64F getRhoMin()
   {
      return rhoMin;
   }

   public DenseMatrix64F getRhoPreviousAverage()
   {
      rhoMeanYoMatrix.get(rhoMeanTemp);
      return rhoMeanTemp;
   }

   public DenseMatrix64F getQRho()
   {
      return qRho;
   }

   public Map<Integer, FrameVector> getBasisVectors()
   {
      return basisVectors;
   }

   public Map<Integer, FramePoint> getContactPoints()
   {
      return contactPoints;
   }
}
