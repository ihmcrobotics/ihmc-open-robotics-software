package us.ihmc.commonWalkingControlModules.controlModules.flight;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commonWalkingControlModules.configurations.JumpControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.SpatialAccelerationCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.CentroidalMomentumHandler;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class CentroidalAngularVelocityManager
{
   private final FlightWholeBodyAngularVelocityRegulatorQPSolver qpSolver;
   private final HighLevelHumanoidControllerToolbox controllerToolbox;
   private final JumpControllerParameters parameters;

   private final DenseMatrix64F inertiaTensor = new DenseMatrix64F(3, 3);
   private FrameVector3D currentAngularVelocity;
   private FrameVector3D desiredAngularVelocity;
   private DenseMatrix64F desiredInertiaRateOfChange = new DenseMatrix64F(3, 3);
   
   private CentroidalMomentumHandler centroidalMomentumHandler;
   private SpatialAccelerationCommand command;

   public CentroidalAngularVelocityManager(HighLevelHumanoidControllerToolbox controllerToolbox, JumpControllerParameters jumpControllerParameters,
                                           YoVariableRegistry registry)
   {
      this.controllerToolbox = controllerToolbox;
      this.parameters = jumpControllerParameters;
      qpSolver = new FlightWholeBodyAngularVelocityRegulatorQPSolver(controllerToolbox.getControlDT(), registry);
   }

   public void initialize(WholeBodyControlCoreToolbox controlCoreToolbox)
   {
      this.centroidalMomentumHandler = controlCoreToolbox.getCentroidalMomentumHandler();
      
      ReferenceFrame centerOfMassFrame = controlCoreToolbox.getCenterOfMassFrame();
      this.currentAngularVelocity = new FrameVector3D(centerOfMassFrame);
      this.desiredAngularVelocity = new FrameVector3D(centerOfMassFrame);

      qpSolver.initialize(centerOfMassFrame);
      qpSolver.setMaxPrincipalInertia(parameters.getMaximumPrincipalInertia());
      qpSolver.setMinPrincipalInertia(parameters.getMaximumPrincipalInertia());
      qpSolver.setMaxInertiaRateOfChange(parameters.getMaximumInertiaRateOfChange());
      qpSolver.setRegularizationWeights(parameters.getAngularVelocityRegularizationWeights());
   }

   public void compute()
   {
      centroidalMomentumHandler.getCenterOfMassAngularVelocity(currentAngularVelocity);
      centroidalMomentumHandler.getCentroidalAngularInertia(inertiaTensor);
      
      qpSolver.setCurrentCentroidalInertiaTensor(inertiaTensor);
      qpSolver.setCurrentVelocityEstimate(currentAngularVelocity);
      qpSolver.setVelocityCommand(desiredAngularVelocity);
      qpSolver.compute();
      qpSolver.getDesiredInertiaRateOfChange(desiredInertiaRateOfChange);
   }
}
