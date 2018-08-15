package us.ihmc.commonWalkingControlModules.wrenchDistribution;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import gnu.trove.map.TObjectIntMap;
import gnu.trove.map.hash.TObjectIntHashMap;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ConstraintType;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.CenterOfPressureCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.ContactWrenchCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.PlaneContactStateCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.ControllerCoreOptimizationSettings;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.QPInput;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.SelectionCalculator;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.robotics.screwTheory.SpatialForceVector;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.robotics.weightMatrices.WeightMatrix6D;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFrameVector2D;

public class WrenchMatrixCalculator
{
   private final int nContactableBodies;
   private final int maxNumberOfContactPoints;
   private final int numberOfBasisVectorsPerContactPoint;
   private final int rhoSize;
   private final int copTaskSize;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final YoBoolean useForceRateHighWeight = new YoBoolean("useForceRateHighWeight", registry);

   private final YoDouble rhoWeight = new YoDouble("rhoWeight", registry);
   private final YoDouble rhoRateDefaultWeight = new YoDouble("rhoRateDefaultWeight", registry);
   private final YoDouble rhoRateHighWeight = new YoDouble("rhoRateHighWeight", registry);
   private final YoFrameVector2D desiredCoPWeight = new YoFrameVector2D("desiredCoPWeight", null, registry);

   private final YoFrameVector2D copRateDefaultWeight = new YoFrameVector2D("copRateDefaultWeight", null, registry);
   private final YoFrameVector2D copRateHighWeight = new YoFrameVector2D("copRateHighWeight", null, registry);

   private final DenseMatrix64F rhoJacobianMatrix;
   private final DenseMatrix64F copJacobianMatrix;
   private final DenseMatrix64F rhoPreviousMatrix;

   private final DenseMatrix64F desiredCoPMatrix;
   private final DenseMatrix64F previousCoPMatrix;
   private final DenseMatrix64F activeRhoMatrix;

   private final DenseMatrix64F rhoMaxMatrix;
   private final DenseMatrix64F rhoWeightMatrix;
   private final DenseMatrix64F rhoRateWeightMatrix;
   private final DenseMatrix64F desiredCoPWeightMatrix;
   private final DenseMatrix64F copRateWeightMatrix;

   private final ReferenceFrame centerOfMassFrame;
   private final Map<RigidBody, Wrench> wrenchesFromRho = new HashMap<>();

   private final List<RigidBody> rigidBodies = new ArrayList<>();
   private final Map<RigidBody, PlaneContactStateToWrenchMatrixHelper> planeContactStateToWrenchMatrixHelpers = new HashMap<>();
   private final TObjectIntMap<RigidBody> bodyRhoOffsets = new TObjectIntHashMap<RigidBody>();

   private final List<FramePoint3D> basisVectorsOrigin = new ArrayList<>();
   private final List<FrameVector3D> basisVectors = new ArrayList<>();

   private final double dtSquaredInv;

   public WrenchMatrixCalculator(WholeBodyControlCoreToolbox toolbox, YoVariableRegistry parentRegistry)
   {
      this(toolbox, toolbox.getCenterOfMassFrame(), parentRegistry);
   }

   public WrenchMatrixCalculator(WholeBodyControlCoreToolbox toolbox, ReferenceFrame centerOfMassFrame, YoVariableRegistry parentRegistry)
   {
      this.centerOfMassFrame = centerOfMassFrame;
      List<? extends ContactablePlaneBody> contactablePlaneBodies = toolbox.getContactablePlaneBodies();

      double dt = toolbox.getControlDT();
      this.dtSquaredInv = 1.0 / (dt * dt);

      nContactableBodies = toolbox.getNumberOfContactableBodies();
      maxNumberOfContactPoints = toolbox.getNumberOfContactPointsPerContactableBody();
      numberOfBasisVectorsPerContactPoint = toolbox.getNumberOfBasisVectorsPerContactPoint();
      rhoSize = toolbox.getRhoSize();
      copTaskSize = 2 * nContactableBodies;

      rhoJacobianMatrix = new DenseMatrix64F(SpatialForceVector.SIZE, rhoSize);
      copJacobianMatrix = new DenseMatrix64F(copTaskSize, rhoSize);
      rhoPreviousMatrix = new DenseMatrix64F(rhoSize, 1);

      desiredCoPMatrix = new DenseMatrix64F(copTaskSize, 1);
      previousCoPMatrix = new DenseMatrix64F(copTaskSize, 1);
      activeRhoMatrix = new DenseMatrix64F(rhoSize, 1);

      rhoMaxMatrix = new DenseMatrix64F(rhoSize, 1);
      rhoWeightMatrix = new DenseMatrix64F(rhoSize, rhoSize);
      rhoRateWeightMatrix = new DenseMatrix64F(rhoSize, rhoSize);
      desiredCoPWeightMatrix = new DenseMatrix64F(copTaskSize, copTaskSize);
      copRateWeightMatrix = new DenseMatrix64F(copTaskSize, copTaskSize);


      if (contactablePlaneBodies.size() > nContactableBodies)
         throw new RuntimeException("Unexpected number of contactable plane bodies: " + contactablePlaneBodies.size());

      int rhoOffset = 0;
      for (int i = 0; i < contactablePlaneBodies.size(); i++)
      {
         ContactablePlaneBody contactablePlaneBody = contactablePlaneBodies.get(i);
         RigidBody rigidBody = contactablePlaneBody.getRigidBody();

         rigidBodies.add(rigidBody);

         FrictionConeRotationCalculator frictionConeRotation = toolbox.getOptimizationSettings().getFrictionConeRotation();
         PlaneContactStateToWrenchMatrixHelper helper = new PlaneContactStateToWrenchMatrixHelper(contactablePlaneBody, centerOfMassFrame,
               maxNumberOfContactPoints, numberOfBasisVectorsPerContactPoint, frictionConeRotation, registry);
         helper.setDeactivateRhoWhenNotInContact(toolbox.getDeactiveRhoWhenNotInContact());
         planeContactStateToWrenchMatrixHelpers.put(rigidBody, helper);
         bodyRhoOffsets.put(rigidBody, rhoOffset);
         rhoOffset += helper.getRhoSize();

         basisVectorsOrigin.addAll(helper.getBasisVectorsOrigin());
         basisVectors.addAll(helper.getBasisVectors());

         ReferenceFrame bodyFixedFrame = rigidBody.getBodyFixedFrame();

         Wrench wrench = new Wrench(bodyFixedFrame, bodyFixedFrame);
         wrenchesFromRho.put(rigidBody, wrench);
      }

      ControllerCoreOptimizationSettings optimizationSettings = toolbox.getOptimizationSettings();
      rhoWeight.set(optimizationSettings.getRhoWeight());
      rhoRateDefaultWeight.set(optimizationSettings.getRhoRateDefaultWeight());
      rhoRateHighWeight.set(optimizationSettings.getRhoRateHighWeight());
      desiredCoPWeight.set(optimizationSettings.getCoPWeight());
      copRateDefaultWeight.set(optimizationSettings.getCoPRateDefaultWeight());
      copRateHighWeight.set(optimizationSettings.getCoPRateHighWeight());

      parentRegistry.addChild(registry);
   }

   public void submitPlaneContactStateCommand(PlaneContactStateCommand command)
   {
      PlaneContactStateToWrenchMatrixHelper helper = planeContactStateToWrenchMatrixHelpers.get(command.getContactingRigidBody());
      helper.setPlaneContactStateCommand(command);
      useForceRateHighWeight.set(command.isUseHighCoPDamping());
   }

   private final RecyclingArrayList<CenterOfPressureCommand> centerOfPressureCommands = new RecyclingArrayList<>(CenterOfPressureCommand.class);

   public void submitCenterOfPressureCommand(CenterOfPressureCommand command)
   {
      centerOfPressureCommands.add().set(command);
   }

   private final DenseMatrix64F bodyWrenchJacobian = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F fullWrenchJacobian = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F fzRow = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F singleCopRow = new DenseMatrix64F(0, 0);
   private final FrameVector2D weight = new FrameVector2D();

   public boolean getCenterOfPressureInput(QPInput inputToPack)
   {
      int commands = centerOfPressureCommands.size();
      if (commands <= 0)
      {
         return false;
      }

      fullWrenchJacobian.reshape(Wrench.SIZE, rhoSize);
      fzRow.reshape(1, rhoSize);
      singleCopRow.reshape(1, rhoSize);

      CenterOfPressureCommand command = centerOfPressureCommands.get(commands - 1);
      centerOfPressureCommands.remove(commands - 1);

      RigidBody rigidBody = command.getContactingRigidBody();
      FramePoint2DReadOnly desiredCoP = command.getDesiredCoP();

      // Compute the wrench jacobian for the command plane frame:
      // wrench = J * rho
      ReferenceFrame planeFrame = desiredCoP.getReferenceFrame();
      fullWrenchJacobian.zero();
      int rhoStartIndex = 0;

      for (int i = 0; i < rigidBodies.size(); i++)
      {
         PlaneContactStateToWrenchMatrixHelper helper = planeContactStateToWrenchMatrixHelpers.get(rigidBodies.get(i));

         // If the body is null assume the command is for the full robot.
         if (rigidBody == null || (rigidBodies.get(i) == rigidBody && helper.canHandleCoPCommand()))
         {
            helper.computeWrenchJacobianInFrame(planeFrame, bodyWrenchJacobian);
            CommonOps.insert(bodyWrenchJacobian, fullWrenchJacobian, 0, rhoStartIndex);
         }

         rhoStartIndex += helper.getRhoSize();
      }

      inputToPack.reshape(2);
      inputToPack.setConstraintType(command.getConstraintType());

      int fzIndex = 5;
      CommonOps.extractRow(fullWrenchJacobian, fzIndex, fzRow);

      // [x_cop * J_fz + J_ty] * rho == 0
      int tauYIndex = 1;
      CommonOps.extractRow(fullWrenchJacobian, tauYIndex, singleCopRow);
      CommonOps.add(desiredCoP.getX(), fzRow, 1.0, singleCopRow, singleCopRow);
      CommonOps.insert(singleCopRow, inputToPack.getTaskJacobian(), 0, 0);
      inputToPack.getTaskObjective().set(0, 0.0);

      // [y_cop * J_fz - J_tx] * rho == 0
      int tauXIndex = 0;
      CommonOps.extractRow(fullWrenchJacobian, tauXIndex, singleCopRow);
      CommonOps.add(desiredCoP.getY(), fzRow, -1.0, singleCopRow, singleCopRow);
      CommonOps.insert(singleCopRow, inputToPack.getTaskJacobian(), 1, 0);
      inputToPack.getTaskObjective().set(1, 0.0);

      inputToPack.getTaskWeightMatrix().zero();
      if (command.getConstraintType() == ConstraintType.OBJECTIVE)
      {
         weight.setIncludingFrame(command.getWeight());
         weight.changeFrame(planeFrame);
         inputToPack.getTaskWeightMatrix().set(0, 0, command.getWeight().getX());
         inputToPack.getTaskWeightMatrix().set(1, 1, command.getWeight().getY());
      }
      else if (command.getConstraintType() != ConstraintType.EQUALITY)
      {
         throw new RuntimeException("Inequalities are not supported by this command.");
      }

      return true;
   }

   private final RecyclingArrayList<ContactWrenchCommand> contactWrenchCommands = new RecyclingArrayList<>(ContactWrenchCommand.class);

   public void submitContactWrenchCommand(ContactWrenchCommand command)
   {
      // At this point we can not yet compute the matrices since the order of the inverse dynamics commands is not guaranteed. This means
      // the contact state might change. So we need to hold on to the command and wait with computing the task matrices until all inverse
      // dynamics commands are handled.
      contactWrenchCommands.add().set(command);
   }

   private final DenseMatrix64F rigidBodyRhoTaskJacobian = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F tempTaskJacobian = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F tempTaskObjective = new DenseMatrix64F(Wrench.SIZE, 1);
   private final SelectionCalculator selectionCalculator = new SelectionCalculator();

   public boolean getContactWrenchInput(QPInput inputToPack)
   {
      int commands = contactWrenchCommands.size();
      if (commands <= 0)
      {
         return false;
      }

      ContactWrenchCommand command = contactWrenchCommands.get(commands - 1);
      contactWrenchCommands.remove(commands - 1);
      int taskSize = command.getSelectionMatrix().getNumberOfSelectedAxes();
      inputToPack.setConstraintType(command.getConstraintType());
      inputToPack.reshape(taskSize);

      computeCommandMatrices(command, rigidBodyRhoTaskJacobian, inputToPack.getTaskObjective(), inputToPack.getTaskWeightMatrix());

      // Add the Jacobian for the body at the right place in the big Jacobian for all bodies:
      int rhoOffset = bodyRhoOffsets.get(command.getRigidBody());
      DenseMatrix64F fullJacobian = inputToPack.getTaskJacobian();
      fullJacobian.zero();
      CommonOps.insert(rigidBodyRhoTaskJacobian, fullJacobian, 0, rhoOffset);

      return true;
   }

   private void computeCommandMatrices(ContactWrenchCommand command, DenseMatrix64F taskJacobian, DenseMatrix64F taskObjective, DenseMatrix64F taskWeight)
   {
      PlaneContactStateToWrenchMatrixHelper helper = planeContactStateToWrenchMatrixHelpers.get(command.getRigidBody());
      ReferenceFrame planeFrame = helper.getPlaneFrame();
      Wrench wrench = command.getWrench();
      wrench.getBodyFrame().checkReferenceFrameMatch(helper.getRigidBody().getBodyFixedFrame());

      // Get the matrices without considering the selection:
      wrench.changeFrame(planeFrame);
      wrench.getMatrix(tempTaskObjective);
      tempTaskJacobian.set(helper.getWrenchJacobianMatrix());

      SelectionMatrix6D selectionMatrix = command.getSelectionMatrix();
      WeightMatrix6D weightMatrix = command.getWeightMatrix();
      if (command.getConstraintType() != ConstraintType.OBJECTIVE)
      {
         weightMatrix.setAngularWeights(0.0, 0.0, 0.0);
         weightMatrix.setLinearWeights(0.0, 0.0, 0.0);
      }
      selectionCalculator.applySelectionToTask(selectionMatrix, weightMatrix, planeFrame, tempTaskJacobian, tempTaskObjective, taskJacobian, taskObjective,
                                               taskWeight);
   }

   public boolean hasContactStateChanged()
   {
      boolean hasContactStateChanged = false;
      for (int i = 0; i < rigidBodies.size(); i++)
      {
         RigidBody rigidBody = rigidBodies.get(i);
         PlaneContactStateToWrenchMatrixHelper helper = planeContactStateToWrenchMatrixHelpers.get(rigidBody);

         if (helper.hasReset())
            hasContactStateChanged = true;
      }

      return hasContactStateChanged;
   }

   private final Vector2D tempDeisredCoPWeight = new Vector2D();
   private final Vector2D tempCoPRateWeight = new Vector2D();

   public void computeMatrices()
   {
      double rhoWeight = this.rhoWeight.getDoubleValue();
      double rhoRateWeight;
      tempDeisredCoPWeight.set(desiredCoPWeight);
      if (useForceRateHighWeight.getBooleanValue())
      {
         rhoRateWeight = rhoRateHighWeight.getDoubleValue();
         tempCoPRateWeight.set(copRateHighWeight);
      }
      else
      {
         rhoRateWeight = rhoRateDefaultWeight.getDoubleValue();
         tempCoPRateWeight.set(copRateDefaultWeight);
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

         CommonOps.insert(helper.getActiveRhoMatrix(), activeRhoMatrix, rhoStartIndex, 0);

         CommonOps.insert(helper.getRhoJacobian(), rhoJacobianMatrix, 0, rhoStartIndex);
         CommonOps.insert(helper.getCopJacobianMatrix(), copJacobianMatrix, copStartIndex, rhoStartIndex);

         CommonOps.insert(helper.getRhoMax(), rhoMaxMatrix, rhoStartIndex, 0);
         CommonOps.insert(helper.getRhoWeight(), rhoWeightMatrix, rhoStartIndex, rhoStartIndex);
         CommonOps.insert(helper.getRhoRateWeight(), rhoRateWeightMatrix, rhoStartIndex, rhoStartIndex);
         CommonOps.insert(helper.getDesiredCoPWeightMatrix(), desiredCoPWeightMatrix, copStartIndex, copStartIndex);
         CommonOps.insert(helper.getCoPRateWeightMatrix(), copRateWeightMatrix, copStartIndex, copStartIndex);

         rhoStartIndex += helper.getRhoSize();
         copStartIndex += 2;
      }

      CommonOps.scale(dtSquaredInv, rhoRateWeightMatrix);
      CommonOps.scale(dtSquaredInv, copRateWeightMatrix);
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

   public void getRhoJacobianMatrix(DenseMatrix64F rhoJacobianMatrix)
   {
      rhoJacobianMatrix.set(this.rhoJacobianMatrix);
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

   public DenseMatrix64F getActiveRhoMatrix()
   {
      return activeRhoMatrix;
   }

   public DenseMatrix64F getRhoMaxMatrix()
   {
      return rhoMaxMatrix;
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

   public List<FramePoint3D> getBasisVectorsOrigin()
   {
      return basisVectorsOrigin;
   }

   public DenseMatrix64F getRhoJacobianMatrix(RigidBody rigidBody)
   {
      return planeContactStateToWrenchMatrixHelpers.get(rigidBody).getRhoJacobian();
   }

   public ReferenceFrame getJacobianFrame()
   {
      return centerOfMassFrame;
   }

   public List<FrameVector3D> getBasisVectors()
   {
      return basisVectors;
   }

   public int getRhoSize()
   {
      return rhoSize;
   }
}
