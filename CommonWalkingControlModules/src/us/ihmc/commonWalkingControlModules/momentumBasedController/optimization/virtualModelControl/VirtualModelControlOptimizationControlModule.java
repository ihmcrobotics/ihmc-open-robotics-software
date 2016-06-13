package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.virtualModelControl;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.commonWalkingControlModules.virtualModelControl.VirtualModelControlSolution;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.*;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.*;
import us.ihmc.commonWalkingControlModules.visualizer.BasisVectorVisualizer;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.WrenchMatrixCalculator;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.frames.YoWrench;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.*;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.tools.exceptions.NoConvergenceException;
import us.ihmc.tools.io.printing.PrintTools;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class VirtualModelControlOptimizationControlModule
{
   private static final boolean DEBUG = false;
   private static final boolean VISUALIZE_RHO_BASIS_VECTORS = false;
   private static final boolean SETUP_RHO_TASKS = true;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final ReferenceFrame centerOfMassFrame;

   private final WrenchMatrixCalculator wrenchMatrixCalculator;
   private final ExternalWrenchHandler externalWrenchHandler;
   private final SpatialForceVector centroidalMomentumRateSolution = new SpatialForceVector();

   private final MomentumRateCommand momentumRateCommand = new MomentumRateCommand();
   private final DenseMatrix64F momentumSelectionMatrix = new DenseMatrix64F(0, 0);

   private final InverseDynamicsJoint[] jointsToOptimizeFor;
   private final JointIndexHandler jointIndexHandler;
   private final DoubleYoVariable rhoMin = new DoubleYoVariable("rhoMinVMC", registry);

   private YoFrameVector desiredLinearMomentumRate = null;
   private YoFrameVector desiredAngularMomentumRate = null;
   private YoFrameVector achievedLinearMomentumRate = null;
   private YoFrameVector achievedAngularMomentumRate = null;
   private final Map<RigidBody, YoWrench> contactWrenchSolutions = new HashMap<>();
   private final BasisVectorVisualizer basisVectorVisualizer;

   private final VirtualModelControlQPSolver qpSolver;
   private final BooleanYoVariable hasNotConvergedInPast = new BooleanYoVariable("hasNotConvergedInPast", registry);
   private final IntegerYoVariable hasNotConvergedCounts = new IntegerYoVariable("hasNotConvergedCounts", registry);

   private final List<? extends ContactablePlaneBody> contactablePlaneBodies;
   private final List<RigidBody> bodiesInContact = new ArrayList<>();
   private final List<DenseMatrix64F> selectionMatrices = new ArrayList<>();

   private final boolean useMomentumQP;

   public VirtualModelControlOptimizationControlModule(WholeBodyControlCoreToolbox toolbox, InverseDynamicsJoint rootJoint, boolean useMomentumQP,
         YoVariableRegistry parentRegistry)
   {
      this.useMomentumQP = useMomentumQP;
      jointIndexHandler = toolbox.getJointIndexHandler();
      jointsToOptimizeFor = jointIndexHandler.getIndexedJoints();
      centerOfMassFrame = toolbox.getCenterOfMassFrame();
      contactablePlaneBodies = toolbox.getContactablePlaneBodies();

      int rhoSize = WholeBodyControlCoreToolbox.rhoSize;

      double gravityZ = toolbox.getGravityZ();

      if (DEBUG)
      {
         desiredLinearMomentumRate = new YoFrameVector("desiredLinearMomentumRateToQP", null, registry);
         desiredAngularMomentumRate = new YoFrameVector("desiredAngularMomentumRateToQP", null, registry);
         achievedLinearMomentumRate = new YoFrameVector("achievedLinearMomentumRateFromQP", null, registry);
         achievedAngularMomentumRate = new YoFrameVector("achievedAngularMomentumRateFromQP", null, registry);

         for (ContactablePlaneBody contactablePlaneBody : contactablePlaneBodies)
         {
            RigidBody rigidBody = contactablePlaneBody.getRigidBody();
            contactWrenchSolutions.put(rigidBody, new YoWrench(rigidBody.getName() + "_wrenchSolution", rigidBody.getBodyFixedFrame(),
                  rigidBody.getBodyFixedFrame(), registry));
         }
      }


      MomentumOptimizationSettings momentumOptimizationSettings = toolbox.getMomentumOptimizationSettings();

      wrenchMatrixCalculator = new WrenchMatrixCalculator(toolbox, registry);

      YoGraphicsListRegistry yoGraphicsListRegistry = toolbox.getYoGraphicsListRegistry();
      if (VISUALIZE_RHO_BASIS_VECTORS)
         basisVectorVisualizer = new BasisVectorVisualizer("ContactBasisVectors", rhoSize, 1.0, yoGraphicsListRegistry, registry);
      else
         basisVectorVisualizer = null;

      externalWrenchHandler = new ExternalWrenchHandler(gravityZ, toolbox.getCenterOfMassFrame(), rootJoint, contactablePlaneBodies);

      rhoMin.set(momentumOptimizationSettings.getRhoMin());
      qpSolver = new VirtualModelControlQPSolver(rhoSize, registry);
      qpSolver.setMinRho(momentumOptimizationSettings.getRhoMin());

      parentRegistry.addChild(registry);
   }

   public void initialize()
   {
      qpSolver.reset();
      externalWrenchHandler.reset();
      bodiesInContact.clear();
      selectionMatrices.clear();
   }

   private final DenseMatrix64F contactWrench = new DenseMatrix64F(Wrench.SIZE, 1);
   private final DenseMatrix64F totalWrench = new DenseMatrix64F(Wrench.SIZE, 1);
   private final DenseMatrix64F tmpWrench = new DenseMatrix64F(Wrench.SIZE, 1);
   private final Map<RigidBody, Wrench> groundReactionWrenches = new HashMap<>();

   public void compute(VirtualModelControlSolution virtualModelControlSolutionToPack) throws VirtualModelControlModuleException
   {
      groundReactionWrenches.clear();
      wrenchMatrixCalculator.computeMatrices();

      qpSolver.setRhoRegularizationWeight(wrenchMatrixCalculator.getRhoWeightMatrix());
      qpSolver.addRegularization();

      if (VISUALIZE_RHO_BASIS_VECTORS)
         basisVectorVisualizer.visualize(wrenchMatrixCalculator.getBasisVectors(), wrenchMatrixCalculator.getBasisVectorsOrigin());

      if (SETUP_RHO_TASKS)
         setupRhoTasks();

      qpSolver.setMinRho(rhoMin.getDoubleValue());

      processMomentumRateCommand();

      NoConvergenceException noConvergenceException = null;

      // use the force optimization algorithm
      if (useMomentumQP)
      {
         try
         {
            qpSolver.solve();
         }
         catch (NoConvergenceException e)
         {

            if (!hasNotConvergedInPast.getBooleanValue())
            {
               e.printStackTrace();
               PrintTools.warn(this, "Only showing the stack trace of the first " + e.getClass().getSimpleName() + ". This may be happening more than once.");
            }

            hasNotConvergedInPast.set(true);
            hasNotConvergedCounts.increment();

            noConvergenceException = e;
         }

         DenseMatrix64F rhoSolution = qpSolver.getRhos();

         groundReactionWrenches.putAll(wrenchMatrixCalculator.computeWrenchesFromRho(rhoSolution));
      }
      // divide the load evenly among all contacting bodies
      else
      {
         double loadFraction = 1.0 / (double) bodiesInContact.size();
         CommonOps.scale(loadFraction, tempTaskObjective);

         for (int i = 0; i < contactablePlaneBodies.size(); i++)
         {
            RigidBody controlledBody = contactablePlaneBodies.get(i).getRigidBody();

            Wrench wrench;
            if (bodiesInContact.contains(controlledBody))
               wrench = new Wrench(centerOfMassFrame, centerOfMassFrame, tempTaskObjective);
            else
               wrench = new Wrench(centerOfMassFrame, centerOfMassFrame, new DenseMatrix64F(Wrench.SIZE, 1));

            wrench.changeFrame(controlledBody.getBodyFixedFrame());
            wrench.changeBodyFrameAttachedToSameBody(controlledBody.getBodyFixedFrame());
            groundReactionWrenches.put(controlledBody, wrench);
         }
      }
      externalWrenchHandler.computeExternalWrenches(groundReactionWrenches);

      // get all forces for all contactable bodies
      Map<RigidBody, Wrench> externalWrenchSolution = externalWrenchHandler.getExternalWrenchMap();
      List<RigidBody> rigidBodiesWithExternalWrench = externalWrenchHandler.getRigidBodiesWithExternalWrench();

      // record the momentum rate solution coming from the contact forces, minus gravity fixme we don't currently include external forces
      DenseMatrix64F gravityWrench = externalWrenchHandler.getGravitationalWrench();
      contactWrench.zero();
      for (RigidBody rigidBody : rigidBodiesWithExternalWrench)
      {
         externalWrenchSolution.get(rigidBody).changeFrame(centerOfMassFrame);
         externalWrenchSolution.get(rigidBody).getMatrix(tmpWrench);
         CommonOps.add(contactWrench, tmpWrench, contactWrench);
         externalWrenchSolution.get(rigidBody).changeFrame(rigidBody.getBodyFixedFrame());
      }
      CommonOps.add(contactWrench, gravityWrench, totalWrench);
      centroidalMomentumRateSolution.set(centerOfMassFrame, totalWrench);

      // we don't nominally want this stuff
      if (DEBUG)
      {
         achievedLinearMomentumRate.set(totalWrench.get(3), totalWrench.get(4), totalWrench.get(5));
         achievedAngularMomentumRate.set(totalWrench.get(0), totalWrench.get(1), totalWrench.get(2));

         for (RigidBody rigidBody : rigidBodiesWithExternalWrench)
         {
            if (contactWrenchSolutions.containsKey(rigidBody))
               contactWrenchSolutions.get(rigidBody).set(externalWrenchSolution.get(rigidBody));
         }
      }

      virtualModelControlSolutionToPack.setJointsToCompute(jointsToOptimizeFor);
      virtualModelControlSolutionToPack.setExternalWrenchSolution(rigidBodiesWithExternalWrench, externalWrenchSolution);
      virtualModelControlSolutionToPack.setBodiesInContact(bodiesInContact);
      virtualModelControlSolutionToPack.setCentroidalMomentumRateSolution(centroidalMomentumRateSolution);
      virtualModelControlSolutionToPack.setCentroidalMomentumSelectionMatrix(momentumSelectionMatrix);

      if (noConvergenceException != null)
      {
         throw new VirtualModelControlModuleException(noConvergenceException, virtualModelControlSolutionToPack);
      }
   }

   private void setupRhoTasks()
   {
      DenseMatrix64F rhoPrevious = wrenchMatrixCalculator.getRhoPreviousMatrix();
      DenseMatrix64F rhoRateWeight = wrenchMatrixCalculator.getRhoRateWeightMatrix();
      qpSolver.addRhoTask(rhoPrevious, rhoRateWeight);

      DenseMatrix64F copJacobian = wrenchMatrixCalculator.getCopJacobianMatrix();

      DenseMatrix64F previousCoP = wrenchMatrixCalculator.getPreviousCoPMatrix();
      DenseMatrix64F copRateWeight = wrenchMatrixCalculator.getCopRateWeightMatrix();
      qpSolver.addRhoTask(copJacobian, previousCoP, copRateWeight);

      DenseMatrix64F desiredCoP = wrenchMatrixCalculator.getDesiredCoPMatrix();
      DenseMatrix64F desiredCoPWeight = wrenchMatrixCalculator.getDesiredCoPWeightMatrix();
      qpSolver.addRhoTask(copJacobian, desiredCoP, desiredCoPWeight);
   }

   private final DenseMatrix64F tempTaskWeight = new DenseMatrix64F(SpatialAccelerationVector.SIZE, SpatialAccelerationVector.SIZE);
   private final DenseMatrix64F tempTaskWeightSubspace = new DenseMatrix64F(SpatialAccelerationVector.SIZE, SpatialAccelerationVector.SIZE);
   private final DenseMatrix64F tempTaskObjective = new DenseMatrix64F(SpatialAccelerationVector.SIZE, 1);
   private final DenseMatrix64F taskWeightMatrix = new DenseMatrix64F(SpatialAccelerationVector.SIZE, SpatialAccelerationVector.SIZE);
   private final DenseMatrix64F taskJacobian = new DenseMatrix64F(SpatialAccelerationVector.SIZE, SpatialAccelerationVector.SIZE);
   private final DenseMatrix64F taskObjective = new DenseMatrix64F(SpatialAccelerationVector.SIZE, 1);

   public void submitMomentumRateCommand(MomentumRateCommand command)
   {
      momentumRateCommand.set(command);
      momentumRateCommand.setWeights(command.getWeightVector());
      momentumSelectionMatrix.set(command.getSelectionMatrix());
   }

   private final DenseMatrix64F currentColSum = new DenseMatrix64F(1, SpatialForceVector.SIZE);
   private final DenseMatrix64F inputColSum = new DenseMatrix64F(1, SpatialForceVector.SIZE);
   private final DenseMatrix64F tmpSelectionMatrix = new DenseMatrix64F(1, 1);

   public void addSelection(DenseMatrix64F selectionMatrix)
   {
      selectionMatrices.add(selectionMatrix);
   }

   // creates the selection matrix for the full problem, compiling all virtual, external, and balancing wrenches
   private void processSelectionMatrices()
   {
      momentumSelectionMatrix.set(momentumRateCommand.getSelectionMatrix());

      for (int input = 0; input < selectionMatrices.size(); input++)
      {
         DenseMatrix64F selectionMatrix = selectionMatrices.get(input);

         CommonOps.sumCols(momentumSelectionMatrix, currentColSum);
         CommonOps.sumCols(selectionMatrix, inputColSum);

         int currentRow = 0;
         int inputRow = 0;
         for (int col = 0; col < SpatialForceVector.SIZE; col++)
         {
            // if this column has value, we want to insert the next row below it
            if (currentColSum.get(col) == 1)
               currentRow++;

            if (inputColSum.get(col) == 1 && currentColSum.get(col) == 0)
            {
               tmpSelectionMatrix.set(momentumSelectionMatrix);
               momentumSelectionMatrix.reshape(momentumSelectionMatrix.getNumRows()+1, SpatialForceVector.SIZE);

               // copy rows above
               if (currentRow > 0)
               {
                  CommonOps.extract(tmpSelectionMatrix, 0, currentRow, 0, SpatialForceVector.SIZE, momentumSelectionMatrix, 0, 0);
               }

               // insert new row
               CommonOps.extract(selectionMatrix, inputRow, inputRow + 1, 0, SpatialForceVector.SIZE, momentumSelectionMatrix, currentRow, 0);

               // copy rows below
               CommonOps.extract(tmpSelectionMatrix, currentRow, tmpSelectionMatrix.getNumRows(), 0, SpatialForceVector.SIZE, momentumSelectionMatrix,
                     currentRow+1, 0);

               currentRow++;
               inputRow++;
            }
         }
      }
   }

   private void processMomentumRateCommand()
   {
      processSelectionMatrices();

      int taskSize = momentumSelectionMatrix.getNumRows();
      taskObjective.reshape(taskSize, 1);
      taskWeightMatrix.reshape(taskSize, taskSize);

      if (taskSize == 0)
         return;

      // Compute the weight: W = S * W * S^T
      tempTaskWeight.reshape(SpatialAccelerationVector.SIZE, SpatialAccelerationVector.SIZE);
      tempTaskWeightSubspace.reshape(taskSize, SpatialAccelerationVector.SIZE);
      momentumRateCommand.getWeightMatrix(tempTaskWeight);
      CommonOps.mult(momentumSelectionMatrix, tempTaskWeight, tempTaskWeightSubspace);
      CommonOps.multTransB(tempTaskWeightSubspace, momentumSelectionMatrix, taskWeightMatrix);

      // Compute the task Jacobian: J = S * A
      DenseMatrix64F rhoJacobian = wrenchMatrixCalculator.getRhoJacobianMatrix();
      taskJacobian.reshape(taskSize, rhoJacobian.numCols);
      CommonOps.mult(momentumSelectionMatrix, rhoJacobian, taskJacobian);

      // Compute the task objective: p = S * (hDot - sum W_user - W_g)
      DenseMatrix64F additionalExternalWrench = externalWrenchHandler.getSumOfExternalWrenches();
      DenseMatrix64F gravityWrench = externalWrenchHandler.getGravitationalWrench();
      DenseMatrix64F momentumRate = momentumRateCommand.getMomentumRate();
      CommonOps.subtract(momentumRate, additionalExternalWrench, tempTaskObjective);
      CommonOps.subtract(tempTaskObjective, gravityWrench, tempTaskObjective);
      CommonOps.mult(momentumSelectionMatrix, tempTaskObjective, taskObjective);

      // get the selected objective back out
      CommonOps.multTransA(momentumSelectionMatrix, taskObjective, tempTaskObjective);

      if (DEBUG)
      {
         desiredLinearMomentumRate.set(tempTaskObjective.get(3), tempTaskObjective.get(4), tempTaskObjective.get(5));
         desiredAngularMomentumRate.set(tempTaskObjective.get(0), tempTaskObjective.get(1), tempTaskObjective.get(2));
      }

      if (useMomentumQP)
         qpSolver.addMomentumTask(taskJacobian, taskObjective, taskWeightMatrix);
   }

   public void submitPlaneContactStateCommand(PlaneContactStateCommand command)
   {
      wrenchMatrixCalculator.submitPlaneContactStateCommand(command);

      if (command.getNumberOfContactPoints() > 0)
         bodiesInContact.add(command.getContactingRigidBody());
   }

   public void submitExternalWrenchCommand(ExternalWrenchCommand command)
   {
      RigidBody rigidBody = command.getRigidBody();
      Wrench wrench = command.getExternalWrench();
      externalWrenchHandler.setExternalWrenchToCompensateFor(rigidBody, wrench);
   }

   public void submitExternalWrench(RigidBody rigidBody, Wrench wrench)
   {
      externalWrenchHandler.setExternalWrenchToCompensateFor(rigidBody, wrench);
   }
}
