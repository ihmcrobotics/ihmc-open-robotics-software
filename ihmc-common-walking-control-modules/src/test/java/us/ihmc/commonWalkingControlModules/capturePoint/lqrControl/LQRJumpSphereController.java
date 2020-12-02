package us.ihmc.commonWalkingControlModules.capturePoint.lqrControl;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.CoMTrajectoryProvider;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.ContactStateProvider;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.simulationConstructionSetTools.tools.RobotTools;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;
import java.util.List;

public class LQRJumpSphereController implements SphereControllerInterface
{
   private final YoRegistry registry = new YoRegistry("SphereController");

   private final RobotTools.SCSRobotFromInverseDynamicsRobotModel scsRobot;
   private final SphereRobot sphereRobot;
   private final ExternalForcePoint externalForcePoint;

   private final LQRJumpMomentumController lqrMomentumController;

   private final YoFrameVector3D lqrForce = new YoFrameVector3D("lqrForce", ReferenceFrame.getWorldFrame(), registry);

   private final CoMTrajectoryProvider dcmPlan;
   private final List<ContactStateProvider> contactStateProviders = new ArrayList<>();

   public LQRJumpSphereController(SphereRobot sphereRobot, CoMTrajectoryProvider dcmPlan, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.scsRobot = sphereRobot.getScsRobot();
      this.sphereRobot = sphereRobot;
      this.dcmPlan = dcmPlan;

      externalForcePoint = sphereRobot.getScsRobot().getAllExternalForcePoints().get(0);

      sphereRobot.getScsRobot().setController(this);

      lqrMomentumController = new LQRJumpMomentumController(sphereRobot.getOmega0Provider(), sphereRobot.getTotalMass(), registry);
   }

   private final DMatrixRMaj currentState = new DMatrixRMaj(6, 1);

   @Override
   public void doControl()
   {
      scsRobot.updateJointPositions_SCS_to_ID();
      scsRobot.updateJointVelocities_SCS_to_ID();

      sphereRobot.updateFrames();

      int segmentNumber = getSegmentNumber();
      dcmPlan.compute(segmentNumber, getTimeInPhase(segmentNumber));

      sphereRobot.getDesiredDCM().set(dcmPlan.getDesiredDCMPosition());
      sphereRobot.getDesiredDCMVelocity().set(dcmPlan.getDesiredDCMVelocity());

      if (contactStateProviders.get(segmentNumber).getContactState().isLoadBearing())
      {
         lqrMomentumController.setVRPTrajectory(dcmPlan.getVRPTrajectories(), contactStateProviders);
         sphereRobot.getCenterOfMass().get(currentState);
         sphereRobot.getCenterOfMassVelocity().get(3, currentState);
         lqrMomentumController.computeControlInput(currentState, sphereRobot.getScsRobot().getYoTime().getDoubleValue());

         lqrForce.set(lqrMomentumController.getU());
         lqrForce.addZ(sphereRobot.getGravityZ());
         lqrForce.scale(sphereRobot.getTotalMass());
      }
      else
      {
         lqrForce.setToZero();
      }

      externalForcePoint.setForce(lqrForce);

      scsRobot.updateJointPositions_ID_to_SCS();
      scsRobot.updateJointVelocities_ID_to_SCS();
      scsRobot.updateJointTorques_ID_to_SCS();
   }

   private int getSegmentNumber()
   {
      for (int i = 0; i < contactStateProviders.size(); i++)
      {
         if (contactStateProviders.get(i).getTimeInterval().intervalContains(sphereRobot.getScsRobot().getYoTime().getDoubleValue()))
            return i;
      }

      return contactStateProviders.size() - 1;
   }

   private double getTimeInPhase(int phase)
   {
      return sphereRobot.getScsRobot().getYoTime().getDoubleValue() - contactStateProviders.get(phase).getTimeInterval().getStartTime();
   }

   public void solveForTrajectory(List<? extends ContactStateProvider> stateProviders)
   {
      contactStateProviders.clear();
      contactStateProviders.addAll(stateProviders);

      dcmPlan.solveForTrajectory(stateProviders);
   }

   @Override
   public void initialize()
   {
   }

   @Override
   public YoRegistry getYoRegistry()
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
