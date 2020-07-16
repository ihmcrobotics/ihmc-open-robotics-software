package us.ihmc.simpleWholeBodyWalking.SimpleSphere;

import java.util.ArrayList;
import java.util.List;

import org.ejml.data.DMatrixRMaj;

import us.ihmc.commonWalkingControlModules.capturePoint.lqrControl.LQRMomentumController;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.CoMTrajectoryProvider;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.ContactStateProvider;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.TranslationMovingReferenceFrame;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationConstructionSetTools.tools.RobotTools;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoFrameVector3D;
import us.ihmc.simpleWholeBodyWalking.SimpleBipedCoMTrajectoryPlanner;

public class SimpleLQRSphereController implements SimpleSphereControllerInterface
{
   private final YoVariableRegistry registry = new YoVariableRegistry("SphereLQRController");

   private final RobotTools.SCSRobotFromInverseDynamicsRobotModel scsRobot;
   private final SimpleSphereRobot sphereRobot;
   private final ExternalForcePoint externalForcePoint;

   private final LQRMomentumController lqrMomentumController;

   private final YoFrameVector3D lqrForce = new YoFrameVector3D("lqrForce", ReferenceFrame.getWorldFrame(), registry);

   private final SimpleBipedCoMTrajectoryPlanner dcmPlan;
   
   private final YoBoolean LeftInContact = new YoBoolean("LeftInContact", registry);
   private final YoBoolean RightInContact = new YoBoolean("RightInContact", registry);
   
   private final List<RobotSide> currentFeetInContact = new ArrayList<>();

   public SimpleLQRSphereController(SimpleSphereRobot sphereRobot, SimpleBipedCoMTrajectoryPlanner comTrajectoryProvider, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.scsRobot = sphereRobot.getScsRobot();
      this.sphereRobot = sphereRobot;
      externalForcePoint = sphereRobot.getScsRobot().getAllExternalForcePoints().get(0);

      dcmPlan = comTrajectoryProvider;

      sphereRobot.getScsRobot().setController(this);

      lqrMomentumController = new LQRMomentumController(sphereRobot.getOmega0Provider(), registry);
   }

   private final DMatrixRMaj currentState = new DMatrixRMaj(6, 1);

   @Override
   public void doControl()
   {
      scsRobot.updateJointPositions_SCS_to_ID();
      scsRobot.updateJointVelocities_SCS_to_ID();

      sphereRobot.updateFrames();

      double currentTime = sphereRobot.getScsRobot().getYoTime().getDoubleValue();
      updateFeetState(currentTime);
      updateFeetYoVar();
      dcmPlan.setInitialCenterOfMassState(sphereRobot.getCenterOfMass(), sphereRobot.getCenterOfMassVelocity());
      dcmPlan.computeSetpoints(currentTime, currentFeetInContact);

      sphereRobot.getDesiredDCM().set(dcmPlan.getDesiredDCMPosition());
      sphereRobot.getDesiredDCMVelocity().set(dcmPlan.getDesiredDCMVelocity());

      lqrMomentumController.setVRPTrajectory(dcmPlan.getVRPTrajectories());
      sphereRobot.getCenterOfMass().get(currentState);
      sphereRobot.getCenterOfMassVelocity().get(3, currentState);
      lqrMomentumController.computeControlInput(currentState, currentTime);

      lqrForce.set(lqrMomentumController.getU());
      lqrForce.addZ(sphereRobot.getGravityZ());
      lqrForce.scale(sphereRobot.getTotalMass());

      externalForcePoint.setForce(lqrForce);

      scsRobot.updateJointPositions_ID_to_SCS();
      scsRobot.updateJointVelocities_ID_to_SCS();
      scsRobot.updateJointTorques_ID_to_SCS();
   }

  private void updateFeetYoVar()
   {
     LeftInContact.set(false);
     RightInContact.set(false);
      for(int i=0; i<currentFeetInContact.size(); i++)
      {
         if (currentFeetInContact.get(i) == RobotSide.LEFT)
            LeftInContact.set(true);
         if (currentFeetInContact.get(i) == RobotSide.RIGHT)
            RightInContact.set(true);
      }
      
   }

 private void updateFeetState(double currentTime)
   {
      currentFeetInContact.clear();
      
      List<Footstep> footstepList= dcmPlan.getFootstepList();
      List<FootstepTiming> footstepTimingList= dcmPlan.getFootstepTimingList();
      
      if(currentTime < footstepTimingList.get(0).getExecutionStartTime())
      {
         for (RobotSide robotSide : RobotSide.values)
            currentFeetInContact.add(robotSide);
         //Simulation is in transfer prior to beginning steps, keep initial footstepPose
         return;
      }
      for (int i = 0; i < footstepTimingList.size(); i++)
      {
         double swingStartTime = footstepTimingList.get(i).getExecutionStartTime() + footstepTimingList.get(i).getSwingStartTime();
         double swingEndTime = swingStartTime + footstepTimingList.get(i).getSwingTime();
         //Note: if current time = a transition time then it should be in the next state, as that is what the CSPUpdater does
         if (MathTools.intervalContains(currentTime, swingStartTime, swingEndTime, 0.00001, true, false))
         {
            //Robot is in swing
            currentFeetInContact.add(footstepList.get(i).getRobotSide().getOppositeSide());
            sphereRobot.updateSoleFrame(footstepList.get(i).getRobotSide(), footstepList.get(i).getFootstepPose().getPosition());               
            return;
         }
            
      }
      //Simulation has finished all planned steps or is in transfer
      for (RobotSide robotSide : RobotSide.values)
         currentFeetInContact.add(robotSide);
      return;
   }

   public void solveForTrajectory()
   {
      dcmPlan.initialize();
      double currentTime = sphereRobot.getScsRobot().getYoTime().getDoubleValue();
      updateFeetState(currentTime);
      updateFeetYoVar();
      dcmPlan.setInitialCenterOfMassState(sphereRobot.getCenterOfMass(), sphereRobot.getCenterOfMassVelocity());
      dcmPlan.computeSetpoints(currentTime, currentFeetInContact);
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
