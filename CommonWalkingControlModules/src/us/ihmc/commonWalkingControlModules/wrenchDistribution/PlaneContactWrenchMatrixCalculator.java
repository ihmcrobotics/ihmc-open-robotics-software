package us.ihmc.commonWalkingControlModules.wrenchDistribution;

import java.util.ArrayList;
import java.util.Collection;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactPointInterface;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.utilities.math.linearAlgebra.MatrixTools;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.GeometryTools;
import us.ihmc.robotics.geometry.ReferenceFrame;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.SpatialForceVector;
import us.ihmc.utilities.screwTheory.Wrench;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.IntegerYoVariable;
import us.ihmc.yoUtilities.math.frames.YoMatrix;


public class PlaneContactWrenchMatrixCalculator
{
   public static final int footCoPXYComponents = 2;
   
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);
   
   private final IntegerYoVariable numberOfLoadedEndEffectors = new IntegerYoVariable("numberOfLoadedEndEffectors", registry);
   private final DoubleYoVariable wRho = new DoubleYoVariable("wRho", registry);
   private final DoubleYoVariable wRhoSmoother = new DoubleYoVariable("wRhoSmoother", registry);
   private final DoubleYoVariable wRhoPenalizer = new DoubleYoVariable("wRhoPenalizer", registry);
   private final DoubleYoVariable rhoMinScalar = new DoubleYoVariable("rhoMinScalarInAdapter", registry);

   private final ReferenceFrame centerOfMassFrame;
   private final Map<RigidBody, Wrench> wrenches = new LinkedHashMap<RigidBody, Wrench>();

   private final List<? extends PlaneContactState> planeContactStates;

   private final DenseMatrix64F rhoMin;
   private final DenseMatrix64F qRho;
   private final DenseMatrix64F wRhoMatrix;
   private final DenseMatrix64F wRhoSmootherMatrix;
   
   private final DenseMatrix64F wRhoPenalizerMatrix;
   
   private final DenseMatrix64F qFeetCoP;

   private final DoubleYoVariable rhoTotal;
   
   private final DenseMatrix64F rhoMeanTemp;
   private final YoMatrix rhoMeanYoMatrix;
   
   private final DenseMatrix64F rhoMeanLoadedEndEffectors;
   private final YoMatrix rhoMeanLoadedEndEffectorsYoMatrix;
   
   private RobotSide footUnderCoPControl;
   private ReferenceFrame footCoPReferenceFrame; 

   private final int totalRhoSize;
   private final int nSupportVectors;
   private final int nPointsPerPlane;
   private final double supportVectorAngleIncrement;

   // Temporary variables
   private final SpatialForceVector currentBasisVector = new SpatialForceVector();
   private final SpatialForceVector currentBasisVectorCoP = new SpatialForceVector();
   private final FrameVector tempContactNormalVector = new FrameVector();
   private final AxisAngle4d normalContactVectorRotation = new AxisAngle4d();
   private final Matrix3d tempNormalContactVectorRotationMatrix = new Matrix3d();
   private final Vector3d tempLinearPart = new Vector3d();
   private final Vector3d tempLinearPartCoP = new Vector3d();
   private final Vector3d tempArm = new Vector3d();
   private final Vector3d tempArmCoP = new Vector3d();
   private final FramePoint tempFramePoint = new FramePoint();
   private final FramePoint tempFramePointCoP = new FramePoint();
   
   private final DenseMatrix64F tempSum = new DenseMatrix64F(SpatialForceVector.SIZE, 1);
   private final DenseMatrix64F tempVector = new DenseMatrix64F(SpatialForceVector.SIZE, 1);

   private final Wrench tempWrench = new Wrench();

   public PlaneContactWrenchMatrixCalculator(ReferenceFrame centerOfMassFrame, int rhoSize, int maxNPointsPerPlane, int maxNSupportVectors, double wRho,
         double wRhoSmoother, double wRhoPenalizer, Collection<? extends PlaneContactState> planeContactStates, YoVariableRegistry parentRegistry)
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
      
      rhoMeanLoadedEndEffectors = new DenseMatrix64F(planeContactStates.size(), 1);
      rhoMeanLoadedEndEffectorsYoMatrix = new YoMatrix(name + "RhoMeanLoadedEndEffectorsYoMatrix", planeContactStates.size(), 1, registry);
      
      rhoTotal = new DoubleYoVariable(name + "RhoTotal", registry);

      qRho = new DenseMatrix64F(Wrench.SIZE, rhoSize);
      rhoMin = new DenseMatrix64F(rhoSize, 1);
      wRhoMatrix = new DenseMatrix64F(rhoSize, rhoSize);
      CommonOps.setIdentity(wRhoMatrix);
      CommonOps.scale(wRho, wRhoMatrix);
      
      qFeetCoP = new DenseMatrix64F(footCoPXYComponents,rhoSize);
      
      wRhoSmootherMatrix = new DenseMatrix64F(rhoSize, rhoSize);
      CommonOps.setIdentity(wRhoSmootherMatrix);
      CommonOps.scale(wRhoSmoother, wRhoSmootherMatrix);
      
      wRhoPenalizerMatrix = new DenseMatrix64F(rhoSize, rhoSize);
      CommonOps.setIdentity(wRhoPenalizerMatrix);
      CommonOps.scale(wRhoPenalizer, wRhoPenalizerMatrix);
      
      this.planeContactStates = new ArrayList<PlaneContactState>(planeContactStates);
      
      for (int i = 0; i < this.planeContactStates.size(); i++)
      {
         RigidBody rigidBody = this.planeContactStates.get(i).getRigidBody();
         ReferenceFrame bodyFixedFrame = rigidBody.getBodyFixedFrame();
         
         Wrench wrench = new Wrench(bodyFixedFrame, bodyFixedFrame);
         wrenches.put(rigidBody, wrench);
      }

      parentRegistry.addChild(registry);
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
         PlaneContactState planeContactState = planeContactStates.get(i);
         
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
        		 if (isInContact)
        		 {
        			 computeBasisVector(planeContactState, contactPoint, k);

        			 currentBasisVector.packMatrixColumn(qRho, iRho);
        			 rhoMin.set(iRho, 0, rhoMinScalar.getDoubleValue());
        			 wRhoMatrix.set(iRho, iRho, wRho.getDoubleValue());
        			 wRhoSmootherMatrix.set(iRho, iRho, wRhoSmoother.getDoubleValue());

        			 if ((footCoPReferenceFrame != null) && (planeContactState.getPlaneFrame() == footCoPReferenceFrame.getParent()))
        			 {
        				 computeCoPBasisVector(planeContactState, contactPoint, k);
        				 qFeetCoP.set(0, iRho, currentBasisVectorCoP.getAngularPartX());
        				 qFeetCoP.set(1, iRho, currentBasisVectorCoP.getAngularPartY());

        			 }
        			 else
        			 {
        				 qFeetCoP.set(0, iRho, 0.0);
        				 qFeetCoP.set(1, iRho, 0.0);
        			 }
        		 }
        		 else
        		 {
        			 rhoMin.set(iRho, 0, 0.0);
        			 wRhoMatrix.set(iRho, iRho, 0.0);
        			 wRhoSmootherMatrix.set(iRho, iRho, 0.0);

        			 qFeetCoP.set(0, iRho, 0.0);
    				 qFeetCoP.set(1, iRho, 0.0); 
    				 
    				 for (int m = 0; m < Wrench.SIZE; m++)
    					 qRho.set(m, iRho, 0.0); // Set the basis vectors of the points not in contact to zero
        		 }

        		 iRho++;
        	 }
         }
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
   
   private void computeCoPBasisVector(PlaneContactState planeContactState, ContactPointInterface contactPoint, int k)
   {
      double angle = k * supportVectorAngleIncrement;
      double mu = planeContactState.getCoefficientOfFriction();

      contactPoint.getPosition(tempFramePointCoP);
      tempFramePointCoP.changeFrame(footCoPReferenceFrame);

      // Compute the linear part considering a normal contact vector pointing up
      tempLinearPartCoP.set(Math.cos(angle) * mu, Math.sin(angle) * mu, 1);
      tempLinearPartCoP.normalize();

      // Compute the unit wrench corresponding to the basis vector
      tempArmCoP.set(tempFramePointCoP.getX(), tempFramePointCoP.getY(), 0.0);
      currentBasisVectorCoP.setUsingArm(footCoPReferenceFrame, tempLinearPartCoP, tempArmCoP);
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

         if (planeContactState.inContact())
         {
            double penaltyScaling = 0.0;

            if (rhoTotal.getDoubleValue() > 1e-3)
               penaltyScaling = Math.max(0.0, 1.0 - rhoMeanLoadedEndEffectors.get(i) / rhoTotal.getDoubleValue());

            int rhoSizeTotalForOneEndEffector = planeContactState.getTotalNumberOfContactPoints() * nSupportVectors;
            int iRhoFinal = iRho + rhoSizeTotalForOneEndEffector;
            for (;iRho < iRhoFinal; iRho++)
               wRhoPenalizerMatrix.set(iRho, iRho, wRhoPenalizer.getDoubleValue() * penaltyScaling);
         }
         else
         {
            int rhoSizeTotalForOneEndEffector = planeContactState.getTotalNumberOfContactPoints() * nSupportVectors;

            int iRhoFinal = iRho + rhoSizeTotalForOneEndEffector;
            for (;iRho < iRhoFinal; iRho++)
               wRhoPenalizerMatrix.set(iRho, iRho, 0.0);
         }
      }
   }
   
   public void setRhoMinScalar(double rhoMinScalar)
   {
      this.rhoMinScalar.set(rhoMinScalar);
   }
   
   public void setFootCoPControlData(RobotSide side, ReferenceFrame frame)
   {
      this.footCoPReferenceFrame = frame;
      this.footUnderCoPControl = side;
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
   
   public DenseMatrix64F getQFeetCoP()
   {
      return qFeetCoP;
   }

   public void setWRhoSmoother(double wRhoSmoother)
   {
      this.wRhoSmoother.set(wRhoSmoother);
   }
}
