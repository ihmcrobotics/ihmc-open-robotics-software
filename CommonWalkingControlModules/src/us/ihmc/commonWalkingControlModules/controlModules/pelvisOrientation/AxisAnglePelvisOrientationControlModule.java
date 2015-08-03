package us.ihmc.commonWalkingControlModules.controlModules.pelvisOrientation;

import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.PelvisOrientationControlModule;
import us.ihmc.commonWalkingControlModules.couplingRegistry.CouplingRegistry;
import us.ihmc.sensorProcessing.ProcessedSensorsInterface;
import us.ihmc.robotics.humanoidRobot.frames.CommonHumanoidReferenceFrames;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.utilities.screwTheory.Twist;
import us.ihmc.utilities.screwTheory.Wrench;
import us.ihmc.yoUtilities.controllers.AxisAngleOrientationController;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.math.frames.YoFrameVector;


public class AxisAnglePelvisOrientationControlModule implements PelvisOrientationControlModule
{
   private final ProcessedSensorsInterface processedSensors;
   private final CouplingRegistry couplingRegistry;
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
  
   private final ReferenceFrame bodyFrame;
   private final FrameVector desiredPelvisAngularVelocity;
   private final FrameVector pelvisAngularVelocity;
   private final YoFrameVector tauPelvis;
   private final YoFrameVector tauSwingLegCompensation;

   private final boolean useFeedforward;

   private final Wrench upperBodyWrench = new Wrench();
   private final AxisAngleOrientationController axisAngleOrientationController;
   


   public AxisAnglePelvisOrientationControlModule(ProcessedSensorsInterface processedSensors, CommonHumanoidReferenceFrames referenceFrames,
         CouplingRegistry couplingRegistry, double dt, YoVariableRegistry parentRegistry, boolean useFeedforward)
   {
      this.processedSensors = processedSensors;
      ReferenceFrame bodyFrame = referenceFrames.getPelvisFrame();
      this.couplingRegistry = couplingRegistry;
      this.bodyFrame = bodyFrame;
      this.desiredPelvisAngularVelocity = new FrameVector(bodyFrame);
      this.pelvisAngularVelocity = new FrameVector(bodyFrame);
      this.tauPelvis = new YoFrameVector("tauPelvis", "", bodyFrame, registry);
      this.tauSwingLegCompensation = new YoFrameVector("tauSwingLegCompensation", "", bodyFrame, registry);
      this.useFeedforward = useFeedforward;
      this.axisAngleOrientationController = new AxisAngleOrientationController("pelvis", bodyFrame, dt, registry);


      parentRegistry.addChild(registry);
   }

   public void reset()
   {
      axisAngleOrientationController.reset();
   }
   
   public void setupParametersForR2()
   {
      axisAngleOrientationController.setProportionalGains(1500.0, 1500.0, 1500.0);
      axisAngleOrientationController.setDerivativeGains(200.0, 150.0, 50.0);

   }

   public void setupParametersForM2V2()
   {
      axisAngleOrientationController.setProportionalGains(500.0, 250.0, 150.0);
//      axisAngleOrientationController.setProportionalGains(250.0, 250.0, 150.0);
//      axisAngleOrientationController.setProportionalGains(150.0, 100.0, 100.0); // TODO: test using these lower gains again
      axisAngleOrientationController.setDerivativeGains(30.0, 18.0, 5.0); // 80.0, 75.0, 30.0
   }

   public FrameVector computePelvisTorque(RobotSide supportLeg, FrameOrientation desiredPelvisOrientation)
   {
      Twist twistOfPelvisWithRespectToWorld = processedSensors.getTwistOfPelvisWithRespectToWorld();
      twistOfPelvisWithRespectToWorld.changeFrame(bodyFrame);
      twistOfPelvisWithRespectToWorld.packAngularPart(pelvisAngularVelocity);
     

      FrameVector ret = new FrameVector(bodyFrame);
      axisAngleOrientationController.compute(ret, desiredPelvisOrientation, desiredPelvisAngularVelocity, pelvisAngularVelocity, new FrameVector(bodyFrame));

      if (useFeedforward)
      {
         FrameVector feedForwardTerm = computeFeedForwardTerm(supportLeg);
         ret.add(feedForwardTerm);
      }

      tauPelvis.set(ret);

      return ret;
   }


   private FrameVector computeFeedForwardTerm(RobotSide supportLeg)
   {
      FrameVector ret;
      if (supportLeg != null && couplingRegistry.getDesiredUpperBodyWrench() != null)
      {
         upperBodyWrench.set(couplingRegistry.getDesiredUpperBodyWrench());
         upperBodyWrench.changeFrame(bodyFrame);
         ret = new FrameVector(upperBodyWrench.getExpressedInFrame(), upperBodyWrench.getAngularPartCopy());
      } else
      {
         ret = new FrameVector(bodyFrame);
      }
      tauSwingLegCompensation.set(ret);
      return ret;
   }
}
