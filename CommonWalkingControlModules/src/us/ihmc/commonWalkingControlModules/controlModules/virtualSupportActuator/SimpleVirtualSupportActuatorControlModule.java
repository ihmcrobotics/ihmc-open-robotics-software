package us.ihmc.commonWalkingControlModules.controlModules.virtualSupportActuator;


import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.DoubleSupportForceDistributor;
import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.VirtualSupportActuatorControlModule;
import us.ihmc.commonWalkingControlModules.kinematics.StanceFullLegJacobian;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegTorques;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LowerBodyTorques;
import us.ihmc.sensorProcessing.ProcessedSensorsInterface;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.yoUtilities.math.trajectories.YoMinimumJerkTrajectory;


public class SimpleVirtualSupportActuatorControlModule implements VirtualSupportActuatorControlModule
{
   private final YoVariableRegistry registry = new YoVariableRegistry(this.getClass().getSimpleName());
   private final ProcessedSensorsInterface processedSensors;
   private final DoubleSupportForceDistributor doubleSupportForceDistributor;
   private final SideDependentList<StanceFullLegJacobian> stanceFullLegJacobians;
   private final SideDependentList<ReferenceFrame> footFrames;
   private final SideDependentList<ReferenceFrame> footZUpFrames;
   
   private final BooleanYoVariable lastTickWasDoubleSupport = new BooleanYoVariable("lastTickWasDoubleSupport", registry);
   private final YoMinimumJerkTrajectory transferToSingleSupportTrajectory = new YoMinimumJerkTrajectory("transferToSingleSupportTrajectory", registry);
   private final SideDependentList<DoubleYoVariable> smoothenedLegStrenghts = new SideDependentList<DoubleYoVariable>();
   private final DoubleYoVariable timeToTransferToSingleSupport = new DoubleYoVariable("timeToTransferToSingleSupport", registry);
   
      
   public SimpleVirtualSupportActuatorControlModule(ProcessedSensorsInterface processedSensors, DoubleSupportForceDistributor doubleSupportForceDistributor, CommonHumanoidReferenceFrames referenceFrames,
           SideDependentList<StanceFullLegJacobian> stanceFullLegJacobians, YoVariableRegistry parentRegistry)
   {
      this.processedSensors = processedSensors;
      this.doubleSupportForceDistributor = doubleSupportForceDistributor;
      this.stanceFullLegJacobians = new SideDependentList<StanceFullLegJacobian>(stanceFullLegJacobians);
      footFrames = new SideDependentList<ReferenceFrame>(referenceFrames.getFootFrame(RobotSide.LEFT),
            referenceFrames.getFootFrame(RobotSide.RIGHT));
      footZUpFrames = referenceFrames.getAnkleZUpReferenceFrames();
      
      for(RobotSide robotSide : RobotSide.values)
      {
         smoothenedLegStrenghts.put(robotSide, new DoubleYoVariable("smoothened" + robotSide.getCamelCaseNameForMiddleOfExpression() + "LegStrenght", parentRegistry));
      }
      
      this.lastTickWasDoubleSupport.set(true);
      

      
      parentRegistry.addChild(registry);
   }

   public void setParametersForM2V2()
   {
      timeToTransferToSingleSupport.set(0.02);
   }
   
   public void setParametersForR2()
   {
      timeToTransferToSingleSupport.set(0.05);
   }
   

   public void controlDoubleSupport(LowerBodyTorques lowerBodyTorquesToPack, SideDependentList<FramePoint2d> virtualToePoints,
         SideDependentList<Double> legStrengths, double fZOnPelvisInPelvisFrame, FrameVector torqueOnPelvis, double deltaNx)
   {
      lastTickWasDoubleSupport.set(true);
      
      
      
      // TODO: Rethink deltaNx and figure out what it should do...
      
      // distribute forces
      SideDependentList<Double> fZs = new SideDependentList<Double>();
      SideDependentList<FrameVector> torques = new SideDependentList<FrameVector>();
      doubleSupportForceDistributor.packForcesAndTorques(fZs, torques, fZOnPelvisInPelvisFrame, torqueOnPelvis, legStrengths, virtualToePoints);

      for (RobotSide robotSide : RobotSide.values)
      {
         LegTorques supportLegTorquesToPack = lowerBodyTorquesToPack.getLegTorques(robotSide);
         FramePoint2d vtp = new FramePoint2d(virtualToePoints.get(robotSide));
         double fZ = fZs.get(robotSide);
         FrameVector torque = torques.get(robotSide);

         controlSingleLeg(supportLegTorquesToPack, vtp, fZ, torque);
         
         smoothenedLegStrenghts.get(robotSide).set(legStrengths.get(robotSide));
         
      }
      
      
   }


   public void controlSingleSupport(LegTorques supportLegTorquesToPack, FramePoint2d virtualToePoint, double fZOnPelvisInPelvisFrame,
                                    FrameVector torqueOnPelvisInPelvisFrame, Wrench upperBodyWrench)
   {
      RobotSide stanceSide = supportLegTorquesToPack.getRobotSide();
      
      if(lastTickWasDoubleSupport.getBooleanValue())
      {
         initializeLegStrengthTrajectory(stanceSide);
      }
      lastTickWasDoubleSupport.set(false);
      
      
      transferToSingleSupportTrajectory.computeTrajectory(processedSensors.getTime());
      SideDependentList<Double> legStrengths = new SideDependentList<Double>();
      
      legStrengths.put(stanceSide, transferToSingleSupportTrajectory.getPosition());
      legStrengths.put(stanceSide.getOppositeSide(), 1.0 - legStrengths.get(stanceSide));
      
      for (RobotSide robotSide : RobotSide.values)
      {
         smoothenedLegStrenghts.get(robotSide).set(legStrengths.get(robotSide));
      }
      
      
      SideDependentList<Double> fZs = new SideDependentList<Double>();
      SideDependentList<FrameVector> torques = new SideDependentList<FrameVector>();

      doubleSupportForceDistributor.packForcesAndTorques(fZs, torques, fZOnPelvisInPelvisFrame, torqueOnPelvisInPelvisFrame, legStrengths, null);
      Double fZOnPelvisForLeg = fZs.get(stanceSide);
      FrameVector torqueOnPelvisForLeg = torques.get(stanceSide);


      controlSingleLeg(supportLegTorquesToPack, virtualToePoint, fZOnPelvisForLeg, torqueOnPelvisForLeg);
   }

   private void controlSingleLeg(LegTorques supportLegTorquesToPack, FramePoint2d virtualToePoint, Double fZOnPelvisForLeg,
         FrameVector torqueOnPelvisForLeg)
   {
      RobotSide stanceSide = supportLegTorquesToPack.getRobotSide();
      StanceFullLegJacobian stanceFullLegJacobian = stanceFullLegJacobians.get(stanceSide);

      // compute Jacobian
      // TODO: Check all this reference frame stuff and make sure the Jacobians are using the correct frames. It may be that everything only works on flat ground.
      // TODO: Line below is BAD and we need a fix. VTPs need to be in foot frames, but we are computing them in ZUp Frames.
      // TODO: Still not OK. DoubleSupport uses Zup frames
      virtualToePoint.changeFrame(footZUpFrames.get(stanceSide));
      virtualToePoint = new FramePoint2d(footFrames.get(stanceSide), virtualToePoint.getX(), virtualToePoint.getY());
      stanceFullLegJacobian.computeJacobians(virtualToePoint);

      // compute a wrench in the nullspace of the VTP columns of the Jacobian
      Wrench wrenchOnPelvisInPelvisFrame = stanceFullLegJacobian.getWrenchInVTPNullSpace(fZOnPelvisForLeg, torqueOnPelvisForLeg);
      
      // compute joint torques
      stanceFullLegJacobian.packLegTorques(supportLegTorquesToPack, wrenchOnPelvisInPelvisFrame);
   }
   

   private void initializeLegStrengthTrajectory(RobotSide stanceSide)
   {
      double stanceLegStrength = smoothenedLegStrenghts.get(stanceSide).getDoubleValue();
      double startTime = processedSensors.getTime();
      transferToSingleSupportTrajectory.setParams(stanceLegStrength, 0.0, 0.0, 1.0, 0.0, 0.0, startTime, startTime + timeToTransferToSingleSupport.getDoubleValue());
   }
}

