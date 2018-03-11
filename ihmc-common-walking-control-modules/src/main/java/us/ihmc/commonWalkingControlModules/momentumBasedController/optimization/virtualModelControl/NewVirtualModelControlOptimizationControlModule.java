package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.virtualModelControl;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.MomentumRateCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.PlaneContactStateCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.ExternalWrenchHandler;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.groundContactForce.GroundContactForceOptimizationControlModule;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.groundContactForce.NewGroundContactForceOptimizationControlModule;
import us.ihmc.commonWalkingControlModules.virtualModelControl.NewVirtualModelControlSolution;
import us.ihmc.commonWalkingControlModules.virtualModelControl.VirtualModelControlSolution;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.WrenchMatrixCalculator;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.frames.YoWrench;
import us.ihmc.robotics.screwTheory.*;
import us.ihmc.tools.exceptions.NoConvergenceException;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class NewVirtualModelControlOptimizationControlModule
{
   private static final boolean DEBUG = false;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final ReferenceFrame centerOfMassFrame;

   private final ExternalWrenchHandler externalWrenchHandler;
   private final SpatialForceVector centroidalMomentumRateSolution = new SpatialForceVector();

   private final DenseMatrix64F momentumSelectionMatrix = new DenseMatrix64F(0, 0);
   private final SelectionMatrix6D centroidalMomentumSelectionMatrix = new SelectionMatrix6D();

   private final InverseDynamicsJoint[] jointsToOptimizeFor;

   private final YoFrameVector achievedLinearMomentumRate;
   private final YoFrameVector achievedAngularMomentumRate;
   private final Map<RigidBody, YoWrench> contactWrenchSolutions = new HashMap<>();

   private final NewGroundContactForceOptimizationControlModule groundContactForceOptimization;
   private final YoBoolean hasNotConvergedInPast = new YoBoolean("hasNotConvergedInPast", registry);
   private final YoInteger hasNotConvergedCounts = new YoInteger("hasNotConvergedCounts", registry);

   private final NewVirtualModelControlSolution virtualModelControlSolution = new NewVirtualModelControlSolution();

   public NewVirtualModelControlOptimizationControlModule(WholeBodyControlCoreToolbox toolbox, YoVariableRegistry parentRegistry)
   {
      jointsToOptimizeFor = toolbox.getJointIndexHandler().getIndexedJoints();
      centerOfMassFrame = toolbox.getCenterOfMassFrame();
      List<? extends ContactablePlaneBody> contactablePlaneBodies = toolbox.getContactablePlaneBodies();

      WrenchMatrixCalculator wrenchMatrixCalculator = toolbox.getWrenchMatrixCalculator();
      groundContactForceOptimization = new NewGroundContactForceOptimizationControlModule(wrenchMatrixCalculator, contactablePlaneBodies,
                                                                                          toolbox.getCenterOfMassFrame(),  toolbox.getRootJoint(),
                                                                                          toolbox.getOptimizationSettings(), parentRegistry,
                                                                                          toolbox.getYoGraphicsListRegistry());

      double gravityZ = toolbox.getGravityZ();

      if (DEBUG)
      {
         achievedLinearMomentumRate = new YoFrameVector("achievedLinearMomentumRateFromQP", null, registry);
         achievedAngularMomentumRate = new YoFrameVector("achievedAngularMomentumRateFromQP", null, registry);

         for (ContactablePlaneBody contactablePlaneBody : contactablePlaneBodies)
         {
            RigidBody rigidBody = contactablePlaneBody.getRigidBody();
            contactWrenchSolutions.put(rigidBody,
                                       new YoWrench(rigidBody.getName() + "_wrenchSolution", rigidBody.getBodyFixedFrame(), rigidBody.getBodyFixedFrame(),
                                                    registry));
         }
      }
      else
      {
         achievedLinearMomentumRate = null;
         achievedAngularMomentumRate = null;
      }

      externalWrenchHandler = new ExternalWrenchHandler(gravityZ, centerOfMassFrame, toolbox.getTotalRobotMass(), contactablePlaneBodies);

      parentRegistry.addChild(registry);
   }

   public void initialize()
   {
      groundContactForceOptimization.initialize();
      externalWrenchHandler.reset();
   }

   private final DenseMatrix64F contactWrench = new DenseMatrix64F(Wrench.SIZE, 1);
   private final DenseMatrix64F totalWrench = new DenseMatrix64F(Wrench.SIZE, 1);
   private final DenseMatrix64F tmpWrench = new DenseMatrix64F(Wrench.SIZE, 1);


   public NewVirtualModelControlSolution compute() throws NewVirtualModelControlModuleException
   {
      DenseMatrix64F gravityWrench = externalWrenchHandler.getGravitationalWrench();

      setupWrenchesEquilibriumConstraint();

      NoConvergenceException noConvergenceException = null;
      Map<RigidBody, Wrench> groundReactionWrenches = null;
      try
      {
         groundReactionWrenches = groundContactForceOptimization.compute();
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

      virtualModelControlSolution.setJointsToCompute(jointsToOptimizeFor);
      virtualModelControlSolution.setExternalWrenchSolution(rigidBodiesWithExternalWrench, externalWrenchSolution);
      virtualModelControlSolution.setCentroidalMomentumRateSolution(centroidalMomentumRateSolution);
      virtualModelControlSolution.setCentroidalMomentumSelectionMatrix(centroidalMomentumSelectionMatrix);

      if (noConvergenceException != null)
      {
         throw new NewVirtualModelControlModuleException(noConvergenceException, virtualModelControlSolution);
      }

      return virtualModelControlSolution;
   }

   public void submitMomentumRateCommand(MomentumRateCommand command)
   {
      groundContactForceOptimization.submitMomentumRateCommand(command);
      command.getSelectionMatrix(centroidalMomentumSelectionMatrix);
      command.getSelectionMatrix(centerOfMassFrame, momentumSelectionMatrix);
   }

   public void submitPlaneContactStateCommand(PlaneContactStateCommand command)
   {
      groundContactForceOptimization.submitPlaneContactStateCommand(command);
   }

   public void submitExternalWrench(RigidBody rigidBody, Wrench wrench)
   {
      externalWrenchHandler.setExternalWrenchToCompensateFor(rigidBody, wrench);
   }

   private void setupWrenchesEquilibriumConstraint()
   {
      DenseMatrix64F additionalExternalWrench = externalWrenchHandler.getSumOfExternalWrenches();
      DenseMatrix64F gravityWrench = externalWrenchHandler.getGravitationalWrench();
      groundContactForceOptimization.setupWrenchesEquilibriumConstraint(additionalExternalWrench, gravityWrench);
   }
}
