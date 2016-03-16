package us.ihmc.commonWalkingControlModules.wrenchDistribution;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.PlaneContactStateCommand;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SpatialForceVector;
import us.ihmc.robotics.screwTheory.Wrench;

public class WrenchMatrixCalculator
{
   private static final int nContactableBodies = WholeBodyControlCoreToolbox.nContactableBodies;
   private static final int maxNumberOfContactPoints = WholeBodyControlCoreToolbox.nContactPointsPerContactableBody;
   private static final int numberOfBasisVectorsPerContactPoint = WholeBodyControlCoreToolbox.nBasisVectorsPerContactPoint;
   private static final int rhoSize = WholeBodyControlCoreToolbox.rhoSize;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final DoubleYoVariable rhoMin = new DoubleYoVariable("rhoMin", registry);
   private final DoubleYoVariable rhoWeight = new DoubleYoVariable("rhoWeight", registry);
   private final DoubleYoVariable rhoRateWeight = new DoubleYoVariable("rhoRateWeight", registry);

   private final DenseMatrix64F rhoJacobianMatrix = new DenseMatrix64F(SpatialForceVector.SIZE, rhoSize);
   private final DenseMatrix64F rhoPreviousMatrix = new DenseMatrix64F(rhoSize, 1);

   private final DenseMatrix64F rhoMinMatrix = new DenseMatrix64F(rhoSize, 1);
   private final DenseMatrix64F rhoWeightMatrix = new DenseMatrix64F(rhoSize, rhoSize);
   private final DenseMatrix64F rhoRateWeightMatrix = new DenseMatrix64F(rhoSize, rhoSize);

   private final ReferenceFrame centerOfMassFrame;
   private final Map<RigidBody, Wrench> wrenchesFromRho = new HashMap<>();

   private final List<RigidBody> rigidBodies = new ArrayList<>();
   private final Map<RigidBody, PlaneContactStateToWrenchMatrixHelper> planeContactStateToWrenchMatrixHelpers = new HashMap<>();

   private final List<FramePoint> basisVectorsOrigin = new ArrayList<>();
   private final List<FrameVector> basisVectors = new ArrayList<>();

   public WrenchMatrixCalculator(WholeBodyControlCoreToolbox toolbox, YoVariableRegistry parentRegistry)
   {
      List<? extends ContactablePlaneBody> contactablePlaneBodies = toolbox.getContactablePlaneBodies();

      if (contactablePlaneBodies.size() > nContactableBodies)
         throw new RuntimeException("Unexpected number of contactable plane bodies: " + contactablePlaneBodies.size());

      centerOfMassFrame = toolbox.getCenterOfMassFrame();

      for (int i = 0; i < contactablePlaneBodies.size(); i++)
      {
         ContactablePlaneBody contactablePlaneBody = contactablePlaneBodies.get(i);
         RigidBody rigidBody = contactablePlaneBody.getRigidBody();

         rigidBodies.add(rigidBody);

         PlaneContactStateToWrenchMatrixHelper helper = new PlaneContactStateToWrenchMatrixHelper(contactablePlaneBody, centerOfMassFrame,
               maxNumberOfContactPoints, numberOfBasisVectorsPerContactPoint, registry);
         planeContactStateToWrenchMatrixHelpers.put(rigidBody, helper);

         basisVectorsOrigin.addAll(helper.getBasisVectorsOrigin());
         basisVectors.addAll(helper.getBasisVectors());

         ReferenceFrame bodyFixedFrame = rigidBody.getBodyFixedFrame();

         Wrench wrench = new Wrench(bodyFixedFrame, bodyFixedFrame);
         wrenchesFromRho.put(rigidBody, wrench);
      }

      parentRegistry.addChild(registry);
   }

   public void submitPlaneContactStateCommand(PlaneContactStateCommand command)
   {
      PlaneContactStateToWrenchMatrixHelper helper = planeContactStateToWrenchMatrixHelpers.get(command.getContactingRigidBody());
      helper.setPlaneContactStateCommand(command);
      rhoRateWeight.set(command.getWRhoSmoother());
   }

   public void computeMatrices()
   {
      int rhoStartIndex = 0;

      for (int i = 0; i < rigidBodies.size(); i++)
      {
         RigidBody rigidBody = rigidBodies.get(i);
         PlaneContactStateToWrenchMatrixHelper helper = planeContactStateToWrenchMatrixHelpers.get(rigidBody);

         helper.computeMatrices(rhoMin.getDoubleValue(), rhoWeight.getDoubleValue(), rhoRateWeight.getDoubleValue());

         CommonOps.insert(helper.getRhoMin(), rhoMinMatrix, rhoStartIndex, 0);
         CommonOps.insert(helper.getLastRho(), rhoPreviousMatrix, rhoStartIndex, 0);
         CommonOps.insert(helper.getRhoJacobian(), rhoJacobianMatrix, 0, rhoStartIndex);
         CommonOps.insert(helper.getRhoWeight(), rhoWeightMatrix, rhoStartIndex, rhoStartIndex);
         CommonOps.insert(helper.getRhoRateWeight(), rhoRateWeightMatrix, rhoStartIndex, rhoStartIndex);

         rhoStartIndex += helper.getRhoSize();
      }
   }

   public Map<RigidBody, Wrench> computeWrenchesFromRho(DenseMatrix64F rho)
   {
      // Reinintialize wrenches
      for (int i = 0; i < rigidBodies.size(); i++)
      {
         RigidBody rigidBody = rigidBodies.get(i);
         ReferenceFrame bodyFixedFrame = rigidBody.getBodyFixedFrame();
         wrenchesFromRho.get(rigidBody).setToZero(bodyFixedFrame, bodyFixedFrame);
      }

      int rhoStartIndex = 0;

      for (int i = 0; i < rigidBodies.size(); i++)
      {
         RigidBody rigidBody = rigidBodies.get(i);
         ReferenceFrame bodyFixedFrame = rigidBody.getBodyFixedFrame();
         PlaneContactStateToWrenchMatrixHelper helper = planeContactStateToWrenchMatrixHelpers.get(rigidBody);

         Wrench wrenchFromRho = helper.computeWrenchFromRho(rhoStartIndex, rho);
         wrenchFromRho.changeFrame(bodyFixedFrame);
         wrenchesFromRho.get(rigidBody).add(wrenchFromRho);

         rhoStartIndex += helper.getRhoSize();
      }

      return wrenchesFromRho;
   }

   public void setRhoMin(double rhoMin)
   {
      this.rhoMin.set(rhoMin);
   }

   public void setRhoWeight(double rhoWeight)
   {
      this.rhoWeight.set(rhoWeight);
   }

   public void setRhoRateWeight(double rhoRateWeight)
   {
      this.rhoRateWeight.set(rhoRateWeight);
   }

   public DenseMatrix64F getRhoMin()
   {
      return rhoMinMatrix;
   }

   public DenseMatrix64F getRhoPrevious()
   {
      return rhoPreviousMatrix;
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
