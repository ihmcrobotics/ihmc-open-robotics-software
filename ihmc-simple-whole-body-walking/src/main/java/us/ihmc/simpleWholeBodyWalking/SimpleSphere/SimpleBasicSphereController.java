package us.ihmc.simpleWholeBodyWalking.SimpleSphere;

import us.ihmc.commonWalkingControlModules.capturePoint.YoICPControlGains;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.CoMTrajectoryProvider;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.ContactStateProvider;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.CornerPointViewer;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.SimpleCoMTrajectoryPlanner;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.WrenchDistributorTools;
import us.ihmc.commons.MathTools;
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
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFrameVector3D;
import us.ihmc.simpleWholeBodyWalking.SimpleBipedCoMTrajectoryPlanner;

import java.util.ArrayList;
import java.util.List;

public class SimpleBasicSphereController implements SimpleSphereControllerInterface
{
   private final YoVariableRegistry registry = new YoVariableRegistry("SphereBasicController");

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
      updateFeetState(0);
      vizSphere.updateVizFeet(0, currentFeetInContact);
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
      dcmPlan.computeSetpoints(currentTime, currentFeetInContact);
      
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
      vizSphere.updateVizFeet(currentTime, currentFeetInContact);
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
   
   @Override
   public void initialize()
   {
   }

   public void solveForTrajectory()
   {
      dcmPlan.initialize();
      double currentTime = sphereRobot.getScsRobot().getYoTime().getDoubleValue();
      updateFeetState(currentTime);
      dcmPlan.setInitialCenterOfMassState(sphereRobot.getCenterOfMass(), sphereRobot.getCenterOfMassVelocity());
      double timeInPhase = dcmPlan.computeSetpoints(currentTime, currentFeetInContact);
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
