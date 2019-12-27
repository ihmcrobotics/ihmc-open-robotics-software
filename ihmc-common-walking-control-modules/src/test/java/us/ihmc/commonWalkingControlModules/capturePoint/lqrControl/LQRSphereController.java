package us.ihmc.commonWalkingControlModules.capturePoint.lqrControl;

import org.ejml.data.DenseMatrix64F;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.ContactStateProvider;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.CornerPointViewer;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.SimpleCoMTrajectoryPlanner;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.simulationConstructionSetTools.tools.RobotTools;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

import java.util.ArrayList;
import java.util.List;

public class LQRSphereController implements SphereControllerInterface
{
   private final YoVariableRegistry registry = new YoVariableRegistry("SphereController");

   private final RobotTools.SCSRobotFromInverseDynamicsRobotModel scsRobot;
   private final SphereRobot sphereRobot;
   private final ExternalForcePoint externalForcePoint;

   private final LQRMomentumController lqrMomentumController;

   private final YoFrameVector3D lqrForce = new YoFrameVector3D("lqrForce", ReferenceFrame.getWorldFrame(), registry);

   private final SimpleCoMTrajectoryPlanner dcmPlan;
   private final List<ContactStateProvider> contactStateProviders = new ArrayList<>();

   public LQRSphereController(SphereRobot sphereRobot, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.scsRobot = sphereRobot.getScsRobot();
      this.sphereRobot = sphereRobot;
      externalForcePoint = sphereRobot.getScsRobot().getAllExternalForcePoints().get(0);

      dcmPlan = new SimpleCoMTrajectoryPlanner(sphereRobot.getOmega0Provider());
      dcmPlan.setNominalCoMHeight(sphereRobot.getDesiredHeight());
      dcmPlan.setCornerPointViewer(new CornerPointViewer(registry, yoGraphicsListRegistry));

      sphereRobot.getScsRobot().setController(this);

      lqrMomentumController = new LQRMomentumController();
   }

   private final DenseMatrix64F currentState = new DenseMatrix64F(6, 1);

   @Override
   public void doControl()
   {
      scsRobot.updateJointPositions_SCS_to_ID();
      scsRobot.updateJointVelocities_SCS_to_ID();

      sphereRobot.getScsRobot().update();

      int segmentNumber = getSegmentNumber();
      dcmPlan.compute(segmentNumber, getTimeInPhase(segmentNumber));

      sphereRobot.getDesiredDCM().set(dcmPlan.getDesiredDCMPosition());
      sphereRobot.getDesiredDCMVelocity().set(dcmPlan.getDesiredDCMVelocity());

      lqrMomentumController.setVRPTrajectory(dcmPlan.getVRPTrajectories());
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
