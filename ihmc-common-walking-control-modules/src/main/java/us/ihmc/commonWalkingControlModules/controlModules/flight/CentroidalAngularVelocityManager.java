package us.ihmc.commonWalkingControlModules.controlModules.flight;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.configurations.JumpControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.SpatialAccelerationCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.CentroidalMomentumHandler;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

public class CentroidalAngularVelocityManager
{
   private final FlightWholeBodyAngularVelocityRegulatorQPSolver qpSolver;
   private final HighLevelHumanoidControllerToolbox controllerToolbox;
   private final JumpControllerParameters parameters;

   private final DenseMatrix64F inertiaTensor = new DenseMatrix64F(3, 3);
   private FrameVector3D currentAngularVelocity;
   private FrameVector3D desiredAngularVelocity;
   private DenseMatrix64F desiredInertiaRateOfChange = new DenseMatrix64F(3, 3);
   // Adot v desired -> Adot is based on desired angular velocity v is from current system angular velocity
   private DenseMatrix64F desiredMomentumRateOfChange = new DenseMatrix64F(3, 1);
   private DenseMatrix64F tempMatrix = new DenseMatrix64F(3, 1);
   private FrameVector3D previousAngularMomentumRateLimit;

   private CentroidalMomentumHandler centroidalMomentumHandler;
   private SpatialAccelerationCommand command;

   private final YoFrameVector yoCurrentAngularVelocity;
   private final YoFrameVector yoDesiredAngularVelocity;
   private final YoBoolean yoEnable;

   public CentroidalAngularVelocityManager(HighLevelHumanoidControllerToolbox controllerToolbox, JumpControllerParameters jumpControllerParameters,
                                           YoVariableRegistry registry)
   {
      this.controllerToolbox = controllerToolbox;
      this.parameters = jumpControllerParameters;
      yoCurrentAngularVelocity = new YoFrameVector(getClass().getSimpleName() + "CurrentAngularVelocity", controllerToolbox.getCenterOfMassFrame(), registry);
      yoDesiredAngularVelocity = new YoFrameVector(getClass().getSimpleName() + "DesiredAngularVelocity", controllerToolbox.getCenterOfMassFrame(), registry);
      yoEnable = new YoBoolean(getClass().getSimpleName() + "Enable", registry);
      yoEnable.set(true);
      qpSolver = new FlightWholeBodyAngularVelocityRegulatorQPSolver(controllerToolbox.getControlDT(), registry);
   }

   public void initialize(WholeBodyControlCoreToolbox controlCoreToolbox)
   {
      this.centroidalMomentumHandler = controlCoreToolbox.getCentroidalMomentumHandler();

      ReferenceFrame centerOfMassFrame = controlCoreToolbox.getCenterOfMassFrame();
      this.currentAngularVelocity = new FrameVector3D(centerOfMassFrame);
      this.desiredAngularVelocity = new FrameVector3D(centerOfMassFrame);
      this.previousAngularMomentumRateLimit = new FrameVector3D(centerOfMassFrame);

      qpSolver.initialize(centerOfMassFrame);
      qpSolver.setMaxPrincipalInertia(parameters.getMaximumPrincipalInertia());
      qpSolver.setMinPrincipalInertia(parameters.getMaximumPrincipalInertia());
      qpSolver.setMaxInertiaRateOfChangeProportionalConstant(parameters.getMaximumInertiaRateOfChangeConstant());
      qpSolver.setDiagonalTermsRegularizationWeights(parameters.getAngularVelocityRegularizationWeights());
      qpSolver.setCrossTermsRegularizationWeights(parameters.getAngularVelocityRegularizationWeights());
      qpSolver.setDiagonalTermsDampingWeight(parameters.getDiagonalTermDampingWeight());
      qpSolver.setCrossTermsDampingWeight(parameters.getCrossTermDampingWeight());

   }

   public void setDesiredAngularVelocity(FrameVector3D desiredAngularVelocity)
   {
      this.desiredAngularVelocity.setIncludingFrame(desiredAngularVelocity);
      this.desiredAngularVelocity.changeFrame(qpSolver.getControlFrame());
   }

   public void compute()
   {
      if (yoEnable.getBooleanValue())
      {
         centroidalMomentumHandler.getCenterOfMassAngularVelocity(currentAngularVelocity);
         centroidalMomentumHandler.getCentroidalAngularInertia(inertiaTensor);
         qpSolver.setCurrentCentroidalInertiaTensor(inertiaTensor);
         qpSolver.setCurrentVelocityEstimate(currentAngularVelocity);
         qpSolver.setVelocityCommand(desiredAngularVelocity);
         qpSolver.compute();
         qpSolver.getDesiredInertiaRateOfChange(desiredInertiaRateOfChange);
      }
      else
      {
         desiredInertiaRateOfChange.reshape(3, 3);
         desiredInertiaRateOfChange.zero();
      }
      updateYoVariables();
   }

   private void updateYoVariables()
   {
      yoCurrentAngularVelocity.set(currentAngularVelocity);
      yoDesiredAngularVelocity.set(desiredAngularVelocity);
   }

   public void getInertiaRateOfChange(DenseMatrix64F desiredInertiaRateOfChange)
   {
      desiredInertiaRateOfChange.set(this.desiredInertiaRateOfChange);
   }

   public void getDesiredMomentumRateOfChange(FrameVector3D momentumRateOfChange)
   {
      computeDesiredMomentumRateOfChange();
      momentumRateOfChange.setIncludingFrame(qpSolver.getControlFrame(), desiredMomentumRateOfChange);
   }

   private void computeDesiredMomentumRateOfChange()
   {
      currentAngularVelocity.get(tempMatrix);
      CommonOps.mult(desiredInertiaRateOfChange, tempMatrix, desiredMomentumRateOfChange);
   }

   public void getFilteredDesiredMomentumRateOfChange(FrameVector3D desiredAngularMomentumRateOfChange)
   {
      double alpha = 1.5;
      computeDesiredMomentumRateOfChange();
      CommonOps.scale(alpha, desiredMomentumRateOfChange);
      previousAngularMomentumRateLimit.scale(1 - alpha);
      previousAngularMomentumRateLimit.add(desiredMomentumRateOfChange.get(0, 0), desiredMomentumRateOfChange.get(1, 0), desiredMomentumRateOfChange.get(2, 0));
      desiredAngularMomentumRateOfChange.setIncludingFrame(previousAngularMomentumRateLimit);
   }
}
