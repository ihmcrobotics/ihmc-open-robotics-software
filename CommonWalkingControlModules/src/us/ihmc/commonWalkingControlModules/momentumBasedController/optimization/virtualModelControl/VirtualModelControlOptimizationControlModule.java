package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.virtualModelControl;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.ExternalWrenchCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.MomentumRateCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.PlaneContactStateCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.ExternalWrenchHandler;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.groundContactForce.GroundContactForceOptimizationControlModule;
import us.ihmc.commonWalkingControlModules.virtualModelControl.VirtualModelControlSolution;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.WrenchMatrixCalculator;
import us.ihmc.commons.PrintTools;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.frames.YoWrench;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SpatialForceVector;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.tools.exceptions.NoConvergenceException;

public class VirtualModelControlOptimizationControlModule
{
   private static final boolean DEBUG = false;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final ReferenceFrame centerOfMassFrame;

   private final ExternalWrenchHandler externalWrenchHandler;
   private final SpatialForceVector centroidalMomentumRateSolution = new SpatialForceVector();

   private final DenseMatrix64F momentumSelectionMatrix = new DenseMatrix64F(0, 0);

   private final InverseDynamicsJoint[] jointsToOptimizeFor;

   private final YoFrameVector achievedLinearMomentumRate;
   private final YoFrameVector achievedAngularMomentumRate;
   private final Map<RigidBody, YoWrench> contactWrenchSolutions = new HashMap<>();

   private final GroundContactForceOptimizationControlModule groundContactForceOptimization;
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
      jointsToOptimizeFor = toolbox.getJointIndexHandler().getIndexedJoints();
      centerOfMassFrame = toolbox.getCenterOfMassFrame();
      contactablePlaneBodies = toolbox.getContactablePlaneBodies();

      WrenchMatrixCalculator wrenchMatrixCalculator = new WrenchMatrixCalculator(toolbox, registry);
      groundContactForceOptimization = new GroundContactForceOptimizationControlModule(wrenchMatrixCalculator, contactablePlaneBodies,
            toolbox.getMomentumOptimizationSettings(), parentRegistry, toolbox.getYoGraphicsListRegistry());

      double gravityZ = toolbox.getGravityZ();

      if (DEBUG)
      {
         achievedLinearMomentumRate = new YoFrameVector("achievedLinearMomentumRateFromQP", null, registry);
         achievedAngularMomentumRate = new YoFrameVector("achievedAngularMomentumRateFromQP", null, registry);

         for (ContactablePlaneBody contactablePlaneBody : contactablePlaneBodies)
         {
            RigidBody rigidBody = contactablePlaneBody.getRigidBody();
            contactWrenchSolutions.put(rigidBody, new YoWrench(rigidBody.getName() + "_wrenchSolution", rigidBody.getBodyFixedFrame(),
                  rigidBody.getBodyFixedFrame(), registry));
         }
      }
      else
      {
         achievedLinearMomentumRate = null;
         achievedAngularMomentumRate = null;
      }

      externalWrenchHandler = new ExternalWrenchHandler(gravityZ, centerOfMassFrame, rootJoint, contactablePlaneBodies);

      parentRegistry.addChild(registry);
   }

   public void initialize()
   {
      groundContactForceOptimization.initialize();
      externalWrenchHandler.reset();
      bodiesInContact.clear();
      selectionMatrices.clear();
   }

   private final DenseMatrix64F contactWrench = new DenseMatrix64F(Wrench.SIZE, 1);
   private final DenseMatrix64F totalWrench = new DenseMatrix64F(Wrench.SIZE, 1);
   private final DenseMatrix64F tmpWrench = new DenseMatrix64F(Wrench.SIZE, 1);

   private final DenseMatrix64F momentumObjective = new DenseMatrix64F(Wrench.SIZE, 1);
   private final DenseMatrix64F additionalWrench = new DenseMatrix64F(Wrench.SIZE, 1);
   private final Map<RigidBody, Wrench> groundReactionWrenches = new HashMap<>();

   public void compute(VirtualModelControlSolution virtualModelControlSolutionToPack) throws VirtualModelControlModuleException
   {
      groundReactionWrenches.clear();

      processSelectionMatrices();

      DenseMatrix64F additionalExternalWrench = externalWrenchHandler.getSumOfExternalWrenches();
      DenseMatrix64F gravityWrench = externalWrenchHandler.getGravitationalWrench();
      CommonOps.add(additionalExternalWrench, gravityWrench, additionalWrench);

      groundContactForceOptimization.processMomentumRateCommand(additionalWrench);

      NoConvergenceException noConvergenceException = null;

      // use the force optimization algorithm
      if (useMomentumQP)
      {
         try
         {
            groundContactForceOptimization.compute(groundReactionWrenches);
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
      }
      // divide the load evenly among all contacting bodies
      else
      {
         momentumObjective.set(groundContactForceOptimization.getMomentumObjective());

         double loadFraction = 1.0 / (double) bodiesInContact.size();
         CommonOps.scale(loadFraction, momentumObjective);

         for (int i = 0; i < contactablePlaneBodies.size(); i++)
         {
            RigidBody controlledBody = contactablePlaneBodies.get(i).getRigidBody();

            Wrench wrench;
            if (bodiesInContact.contains(controlledBody))
               wrench = new Wrench(centerOfMassFrame, centerOfMassFrame, momentumObjective);
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

   public void submitMomentumRateCommand(MomentumRateCommand command)
   {
      groundContactForceOptimization.submitMomentumRateCommand(command);
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

      groundContactForceOptimization.submitMomentumSelectionMatrix(momentumSelectionMatrix);
   }

   public void submitPlaneContactStateCommand(PlaneContactStateCommand command)
   {
      groundContactForceOptimization.submitPlaneContactStateCommand(command);

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
