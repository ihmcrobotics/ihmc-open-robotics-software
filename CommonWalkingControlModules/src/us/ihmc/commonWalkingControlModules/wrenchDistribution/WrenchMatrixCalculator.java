package us.ihmc.commonWalkingControlModules.wrenchDistribution;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import javax.vecmath.Vector2d;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.PlaneContactStateCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFrameVector2d;
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
   private static final int copTaskSize = 2 * nContactableBodies;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final BooleanYoVariable useForceRateHighWeight = new BooleanYoVariable("useForceRateHighWeight", registry);

   private final DoubleYoVariable rhoWeight = new DoubleYoVariable("rhoWeight", registry);
   private final DoubleYoVariable rhoRateDefaultWeight = new DoubleYoVariable("rhoRateDefaultWeight", registry);
   private final DoubleYoVariable rhoRateHighWeight = new DoubleYoVariable("rhoRateHighWeight", registry);
   private final YoFrameVector2d desiredCoPWeight = new YoFrameVector2d("desiredCoPWeight", null, registry);

   private final YoFrameVector2d copRateDefaultWeight = new YoFrameVector2d("copRateDefaultWeight", null, registry);
   private final YoFrameVector2d copRateHighWeight = new YoFrameVector2d("copRateHighWeight", null, registry);

   private final DenseMatrix64F rhoJacobianMatrix = new DenseMatrix64F(SpatialForceVector.SIZE, rhoSize);
   private final DenseMatrix64F copJacobianMatrix = new DenseMatrix64F(copTaskSize, rhoSize);
   private final DenseMatrix64F rhoPreviousMatrix = new DenseMatrix64F(rhoSize, 1);

   private final DenseMatrix64F desiredCoPMatrix = new DenseMatrix64F(copTaskSize, 1);
   private final DenseMatrix64F previousCoPMatrix = new DenseMatrix64F(copTaskSize, 1);
   
   private final DenseMatrix64F rhoWeightMatrix = new DenseMatrix64F(rhoSize, rhoSize);
   private final DenseMatrix64F rhoRateWeightMatrix = new DenseMatrix64F(rhoSize, rhoSize);
   private final DenseMatrix64F desiredCoPWeightMatrix = new DenseMatrix64F(copTaskSize, copTaskSize);
   private final DenseMatrix64F copRateWeightMatrix = new DenseMatrix64F(copTaskSize, copTaskSize);

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

      MomentumOptimizationSettings momentumOptimizationSettings = toolbox.getMomentumOptimizationSettings();
      rhoWeight.set(momentumOptimizationSettings.getRhoWeight());
      rhoRateDefaultWeight.set(momentumOptimizationSettings.getRhoRateDefaultWeight());
      rhoRateHighWeight.set(momentumOptimizationSettings.getRhoRateHighWeight());
      desiredCoPWeight.set(momentumOptimizationSettings.getCoPWeight());
      copRateDefaultWeight.set(momentumOptimizationSettings.getCoPRateDefaultWeight());
      copRateHighWeight.set(momentumOptimizationSettings.getCoPRateHighWeight());

      parentRegistry.addChild(registry);
   }

   public void submitPlaneContactStateCommand(PlaneContactStateCommand command)
   {
      PlaneContactStateToWrenchMatrixHelper helper = planeContactStateToWrenchMatrixHelpers.get(command.getContactingRigidBody());
      helper.setPlaneContactStateCommand(command);
      useForceRateHighWeight.set(command.isUseHighCoPDamping());
   }

   private final Vector2d tempDeisredCoPWeight = new Vector2d();
   private final Vector2d tempCoPRateWeight = new Vector2d();

   public void computeMatrices()
   {
      double rhoWeight = this.rhoWeight.getDoubleValue();
      double rhoRateWeight;
      desiredCoPWeight.get(tempDeisredCoPWeight);
      if (useForceRateHighWeight.getBooleanValue())
      {
         rhoRateWeight = rhoRateHighWeight.getDoubleValue();
         copRateHighWeight.get(tempCoPRateWeight);
      }
      else
      {
         rhoRateWeight = rhoRateDefaultWeight.getDoubleValue();
         copRateDefaultWeight.get(tempCoPRateWeight);
      }

      int rhoStartIndex = 0;
      int copStartIndex = 0;

      for (int i = 0; i < rigidBodies.size(); i++)
      {
         RigidBody rigidBody = rigidBodies.get(i);
         PlaneContactStateToWrenchMatrixHelper helper = planeContactStateToWrenchMatrixHelpers.get(rigidBody);


         helper.computeMatrices(rhoWeight, rhoRateWeight, tempDeisredCoPWeight, tempCoPRateWeight);

         CommonOps.insert(helper.getLastRho(), rhoPreviousMatrix, rhoStartIndex, 0);
         CommonOps.insert(helper.getDesiredCoPMatrix(), desiredCoPMatrix, copStartIndex, 0);
         CommonOps.insert(helper.getPreviousCoPMatrix(), previousCoPMatrix, copStartIndex, 0);

         CommonOps.insert(helper.getRhoJacobian(), rhoJacobianMatrix, 0, rhoStartIndex);
         CommonOps.insert(helper.getCopJacobianMatrix(), copJacobianMatrix, copStartIndex, rhoStartIndex);

         CommonOps.insert(helper.getRhoWeight(), rhoWeightMatrix, rhoStartIndex, rhoStartIndex);
         CommonOps.insert(helper.getRhoRateWeight(), rhoRateWeightMatrix, rhoStartIndex, rhoStartIndex);
         CommonOps.insert(helper.getDesiredCoPWeightMatrix(), desiredCoPWeightMatrix, copStartIndex, copStartIndex);
         CommonOps.insert(helper.getCoPRateWeightMatrix(), copRateWeightMatrix, copStartIndex, copStartIndex);

         rhoStartIndex += helper.getRhoSize();
         copStartIndex += 2;
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

         helper.computeWrenchFromRho(rhoStartIndex, rho);
         Wrench wrenchFromRho = helper.getWrenchFromRho();
         wrenchFromRho.changeFrame(bodyFixedFrame);
         wrenchesFromRho.get(rigidBody).add(wrenchFromRho);

         rhoStartIndex += helper.getRhoSize();
      }

      return wrenchesFromRho;
   }

   public DenseMatrix64F getRhoJacobianMatrix()
   {
      return rhoJacobianMatrix;
   }

   public DenseMatrix64F getCopJacobianMatrix()
   {
      return copJacobianMatrix;
   }

   public DenseMatrix64F getRhoPreviousMatrix()
   {
      return rhoPreviousMatrix;
   }

   public DenseMatrix64F getDesiredCoPMatrix()
   {
      return desiredCoPMatrix;
   }

   public DenseMatrix64F getPreviousCoPMatrix()
   {
      return previousCoPMatrix;
   }

   public DenseMatrix64F getRhoWeightMatrix()
   {
      return rhoWeightMatrix;
   }

   public DenseMatrix64F getRhoRateWeightMatrix()
   {
      return rhoRateWeightMatrix;
   }

   public DenseMatrix64F getDesiredCoPWeightMatrix()
   {
      return desiredCoPWeightMatrix;
   }

   public DenseMatrix64F getCopRateWeightMatrix()
   {
      return copRateWeightMatrix;
   }

   public Map<RigidBody, Wrench> getWrenchesFromRho()
   {
      return wrenchesFromRho;
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
