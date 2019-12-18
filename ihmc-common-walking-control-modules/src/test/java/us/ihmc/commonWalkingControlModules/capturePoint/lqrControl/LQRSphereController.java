package us.ihmc.commonWalkingControlModules.capturePoint.lqrControl;

import org.ejml.data.DenseMatrix64F;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.simulationConstructionSetTools.tools.RobotTools;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

public class LQRSphereController implements RobotController
{
   private final YoVariableRegistry registry = new YoVariableRegistry("SphereController");

   private final RobotTools.SCSRobotFromInverseDynamicsRobotModel scsRobot;
   private final SphereRobot sphereRobot;
   private final ExternalForcePoint externalForcePoint;

   private final LQRMomentumController lqrMomentumController;

   private final YoFrameVector3D lqrForce = new YoFrameVector3D("lqrForce", ReferenceFrame.getWorldFrame(), registry);

   private final SimpleDCMPlan dcmPlan;

   public LQRSphereController(RobotTools.SCSRobotFromInverseDynamicsRobotModel scsRobot, SphereRobot sphereRobot, ExternalForcePoint externalForcePoint)
   {
      this.scsRobot = scsRobot;
      this.sphereRobot = sphereRobot;
      this.externalForcePoint = externalForcePoint;
      
      dcmPlan = new SimpleDCMPlan(sphereRobot.getOmega0());
      dcmPlan.setNominalCoMHeight(sphereRobot.getDesiredHeight());

      lqrMomentumController = new LQRMomentumController();
   }

   private final DenseMatrix64F currentState = new DenseMatrix64F(6, 1);

   @Override
   public void doControl()
   {
      scsRobot.updateJointPositions_SCS_to_ID();
      scsRobot.updateJointVelocities_SCS_to_ID();

      sphereRobot.getScsRobot().update();

      dcmPlan.compute(sphereRobot.getScsRobot().getYoTime().getDoubleValue());

      sphereRobot.getDesiredDCM().set(dcmPlan.getDesiredDCMPosition());
      sphereRobot.getDesiredDCMVelocity().set(dcmPlan.getDesiredDCMVelocity());

      lqrMomentumController.setVrpTrajectory(dcmPlan.getVRPTrajectories());
      sphereRobot.getCenterOfMass().get(currentState);
      sphereRobot.getCenterOfMassVelocity().get(3, currentState);
      lqrMomentumController.computeControlInput(currentState, sphereRobot.getScsRobot().getYoTime().getDoubleValue());

      lqrForce.set(lqrMomentumController.getU());
      lqrForce.scale(sphereRobot.getTotalMass());

      externalForcePoint.setForce(lqrForce);

      scsRobot.updateJointPositions_ID_to_SCS();
      scsRobot.updateJointVelocities_ID_to_SCS();
      scsRobot.updateJointTorques_ID_to_SCS();
   }

   @Override
   public void initialize()
   {
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public String getDescription()
   {
      return registry.getName();
   }

   @Override
   public String getName()
   {
      return registry.getName();
   }

}
