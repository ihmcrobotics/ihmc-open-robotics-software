package us.ihmc.simpleWholeBodyWalking.simpleSphere;

import java.util.ArrayList;
import java.util.List;

import org.ejml.data.DMatrixRMaj;

import us.ihmc.commonWalkingControlModules.capturePoint.lqrControl.LQRJumpMomentumController;
import us.ihmc.commonWalkingControlModules.capturePoint.lqrControl.LQRMomentumController;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simpleWholeBodyWalking.SimpleBipedCoMTrajectoryPlanner;
import us.ihmc.simulationConstructionSetTools.tools.RobotTools;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

public class SimpleLQRSphereController implements SimpleSphereControllerInterface
{
   private final YoRegistry registry = new YoRegistry("SphereLQRController");

   private final RobotTools.SCSRobotFromInverseDynamicsRobotModel scsRobot;
   private final SimpleSphereRobot sphereRobot;
   private final ExternalForcePoint externalForcePoint;

   private final LQRJumpMomentumController lqrMomentumController;

   private final YoFrameVector3D lqrForce = new YoFrameVector3D("lqrForce", ReferenceFrame.getWorldFrame(), registry);

   private final SimpleBipedCoMTrajectoryPlanner dcmPlan;
   
   private final YoBoolean leftInContact = new YoBoolean("LeftInContact", registry);
   private final YoBoolean rightInContact = new YoBoolean("RightInContact", registry);
   
   private final List<RobotSide> currentFeetInContact = new ArrayList<>();
   
   private final SimpleSphereVisualizer vizSphere;

   private final List<Footstep> footstepList = new ArrayList<>();
   private final List<FootstepTiming> footstepTimingList = new ArrayList<>();
   private boolean isDoubleSupport;

   public SimpleLQRSphereController(SimpleSphereRobot sphereRobot, SimpleBipedCoMTrajectoryPlanner comTrajectoryProvider, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.scsRobot = sphereRobot.getScsRobot();
      this.sphereRobot = sphereRobot;
      externalForcePoint = sphereRobot.getScsRobot().getAllExternalForcePoints().get(0);

      dcmPlan = comTrajectoryProvider;

      sphereRobot.getScsRobot().setController(this);

      lqrMomentumController = new LQRJumpMomentumController(sphereRobot.getOmega0Provider(), sphereRobot.getTotalMass(), registry);
      
      vizSphere = new SimpleSphereVisualizer(dcmPlan, yoGraphicsListRegistry, sphereRobot, registry);
      
      dcmPlan.initialize();
      
      //start in transfer
      isDoubleSupport = false;
      dcmPlan.initializeForStanding(0);
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
      double timeInPhase = dcmPlan.computeSetpoints(currentTime, footstepList, footstepTimingList);

      sphereRobot.getDesiredDCM().set(dcmPlan.getDesiredDCMPosition());
      sphereRobot.getDesiredDCMVelocity().set(dcmPlan.getDesiredDCMVelocity());

      lqrMomentumController.setVRPTrajectory(dcmPlan.getVRPTrajectories(), dcmPlan.getContactStateProviders());
      sphereRobot.getCenterOfMass().get(currentState);
      sphereRobot.getCenterOfMassVelocity().get(3, currentState);
      lqrMomentumController.computeControlInput(currentState, timeInPhase);

      lqrForce.set(lqrMomentumController.getU());
      lqrForce.addZ(sphereRobot.getGravityZ());
      lqrForce.scale(sphereRobot.getTotalMass());

      externalForcePoint.setForce(lqrForce);

      scsRobot.updateJointPositions_ID_to_SCS();
      scsRobot.updateJointVelocities_ID_to_SCS();
      scsRobot.updateJointTorques_ID_to_SCS();
      
      vizSphere.updateVizPoints(currentTime, lqrForce);
      vizSphere.updateVizFeet(currentTime, currentFeetInContact, footstepList, footstepTimingList);
   }

  private void updateFeetYoVar()
   {
     leftInContact.set(false);
     rightInContact.set(false);
      for(int i=0; i<currentFeetInContact.size(); i++)
      {
         if (currentFeetInContact.get(i) == RobotSide.LEFT)
            leftInContact.set(true);
         if (currentFeetInContact.get(i) == RobotSide.RIGHT)
            rightInContact.set(true);
      }
      
   }

  private void updateFeetState(double currentTime)
  {
     currentFeetInContact.clear();
     
     if(footstepList.size() == 0)
     {//Simulation has finished all planned steps
        for (RobotSide robotSide : RobotSide.values)
           currentFeetInContact.add(robotSide);
        return;
     }
     
     //Simulation is in initial transfer
     if(currentTime < footstepTimingList.get(0).getExecutionStartTime())
     {
        if (!isDoubleSupport)
        {
           dcmPlan.initializeForTransfer(currentTime);
           isDoubleSupport = true;
        }
        
        for (RobotSide robotSide : RobotSide.values)
           currentFeetInContact.add(robotSide);
        return;
     }
     
     for (int i = 0; i < footstepTimingList.size(); i++)
     {
        double swingStartTime = footstepTimingList.get(i).getExecutionStartTime() + footstepTimingList.get(i).getSwingStartTime();
        double swingEndTime = swingStartTime + footstepTimingList.get(i).getSwingTime();
        double footstepEndTime = swingEndTime + footstepTimingList.get(i).getTransferTime();
        
        if (currentTime >= swingStartTime && currentTime < swingEndTime)
        {
           //Robot is in swing
           if (isDoubleSupport)
           {
              dcmPlan.initializeForSingleSupport(currentTime);
              dcmPlan.setSupportLeg(footstepList.get(i).getRobotSide().getOppositeSide());
              isDoubleSupport = false;
           }
           currentFeetInContact.add(footstepList.get(i).getRobotSide().getOppositeSide());
           sphereRobot.updateSoleFrame(footstepList.get(i).getRobotSide(), footstepList.get(i).getFootstepPose().getPosition());               
           return;
        }
        else if (currentTime >= swingEndTime && currentTime < footstepEndTime)
        {
           //Robot is in transfer after swing
           if (!isDoubleSupport)
           {
              dcmPlan.initializeForTransfer(currentTime);
              isDoubleSupport = true;
              footstepList.remove(i);
              footstepTimingList.remove(i);
              dcmPlan.setTransferToSide(footstepList.get(i).getRobotSide());
           }
           currentFeetInContact.add(footstepList.get(i).getRobotSide().getOppositeSide());
           sphereRobot.updateSoleFrame(footstepList.get(i).getRobotSide(), footstepList.get(i).getFootstepPose().getPosition());               
           return;
        }
     }
  }

  @Override
  public void setFootstepPlan(List<Footstep> footstepList, List<FootstepTiming> footstepTimingList)
  {
     this.footstepList.clear();
     this.footstepTimingList.clear();
     this.footstepList.addAll(footstepList);
     this.footstepTimingList.addAll(footstepTimingList);
     dcmPlan.setFinalTransferDuration(footstepTimingList.get(0).getExecutionStartTime());
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
