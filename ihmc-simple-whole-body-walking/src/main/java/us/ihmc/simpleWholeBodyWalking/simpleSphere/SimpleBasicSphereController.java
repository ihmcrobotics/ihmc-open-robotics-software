package us.ihmc.simpleWholeBodyWalking.simpleSphere;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.capturePoint.YoICPControlGains;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.WrenchDistributorTools;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simpleWholeBodyWalking.SimpleBipedCoMTrajectoryPlanner;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;

public class SimpleBasicSphereController implements SimpleSphereControllerInterface
{
   private final YoRegistry registry = new YoRegistry("SphereBasicController");

   private final SimpleSphereRobot sphereRobot;
   private final ExternalForcePoint externalForcePoint;

   private final SimpleBasicHeightController heightController;

   private final SimpleICPProportionalController icpProportionalController;
   private final YoFramePoint3D desiredCMP = new YoFramePoint3D("desiredCMP", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint3D perfectVRP = new YoFramePoint3D("perfectVRP", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector3D vrpForces = new YoFrameVector3D("vrpForces", ReferenceFrame.getWorldFrame(), registry);

   private final SimpleBipedCoMTrajectoryPlanner dcmPlan;
   
   private final List<RobotSide> currentFeetInContact = new ArrayList<>();
   
   private final SimpleSphereVisualizer vizSphere;
   
   private final List<Footstep> footstepList = new ArrayList<>();
   private final List<FootstepTiming> footstepTimingList = new ArrayList<>();
   private boolean isDoubleSupport;

   public SimpleBasicSphereController(SimpleSphereRobot sphereRobot, SimpleBipedCoMTrajectoryPlanner comTrajectoryProvider, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.sphereRobot = sphereRobot;
      externalForcePoint = sphereRobot.getScsRobot().getAllExternalForcePoints().get(0);
      dcmPlan = comTrajectoryProvider;

      YoICPControlGains gains = new YoICPControlGains("", registry);
      gains.setKpOrthogonalToMotion(3.0);
      gains.setKpParallelToMotion(2.0);

      icpProportionalController = new SimpleICPProportionalController(gains, sphereRobot.getControlDT(), registry);

      String name = sphereRobot.getScsRobot().getName();
      YoGraphicPosition desiredCMPViz = new YoGraphicPosition(name + "Desired CMP", desiredCMP, 0.012, YoAppearance.Purple(), YoGraphicPosition.GraphicType.BALL_WITH_CROSS);
      yoGraphicsListRegistry.registerArtifact("Proportional", desiredCMPViz.createArtifact());

      heightController = new SimpleBasicHeightController(sphereRobot, registry);

      sphereRobot.getScsRobot().setController(this);
      
      vizSphere = new SimpleSphereVisualizer(dcmPlan, yoGraphicsListRegistry, sphereRobot, registry);
      
      dcmPlan.initialize();
      
      //start in transfer
      isDoubleSupport = false;
      dcmPlan.initializeForStanding(0);
   }


   private final FrameVector3D forces = new FrameVector3D();
   private final FramePoint3D tempDesiredCMP = new FramePoint3D();

   @Override
   public void doControl()
   {
      sphereRobot.updateJointPositions_SCS_to_ID();
      sphereRobot.updateJointVelocities_SCS_to_ID();

      sphereRobot.updateFrames();

      double currentTime = sphereRobot.getScsRobot().getYoTime().getDoubleValue();
      updateFeetState(currentTime);
      dcmPlan.setInitialCenterOfMassState(sphereRobot.getCenterOfMass(), sphereRobot.getCenterOfMassVelocity());
      dcmPlan.computeSetpoints(currentTime, footstepList, footstepTimingList);
      
      double check = 0;
      if(sphereRobot.getScsRobot().getYoTime().getDoubleValue()>=1.65)
      {
         check += 1;
      }
      sphereRobot.getDesiredDCM().set(dcmPlan.getDesiredDCMPosition());
      sphereRobot.getDesiredDCMVelocity().set(dcmPlan.getDesiredDCMVelocity());
      perfectVRP.set(dcmPlan.getDesiredVRPPosition());

      tempDesiredCMP.set(icpProportionalController.doProportionalControl(sphereRobot.getDCM(), dcmPlan.getDesiredDCMPosition(),
                                                                         dcmPlan.getDesiredDCMVelocity(), sphereRobot.getOmega0()));
      tempDesiredCMP.subZ(sphereRobot.getDesiredHeight());
      desiredCMP.set(tempDesiredCMP);

      heightController.doControl();

      double fZ = heightController.getVerticalForce();
      WrenchDistributorTools.computePseudoCMP3d(tempDesiredCMP, sphereRobot.getCenterOfMass(), new FramePoint2D(tempDesiredCMP), fZ,
                                                sphereRobot.getTotalMass(), sphereRobot.getOmega0());
      WrenchDistributorTools.computeForce(forces, sphereRobot.getCenterOfMass(), tempDesiredCMP, fZ);

      vrpForces.setMatchingFrame(forces);
      externalForcePoint.setForce(forces);

      if (forces.containsNaN())
         throw new RuntimeException("Illegal forces.");

      sphereRobot.updateJointPositions_ID_to_SCS();
      sphereRobot.updateJointVelocities_ID_to_SCS();
      sphereRobot.updateJointTorques_ID_to_SCS();
      
      vizSphere.updateVizPoints(currentTime, vrpForces);
      vizSphere.updateVizFeet(currentTime, currentFeetInContact, footstepList, footstepTimingList);
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
