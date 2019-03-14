package us.ihmc.commonWalkingControlModules.capturePoint;

import java.util.List;

import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.PlaneContactStateCommand;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class LinearMomentumRateControlModuleInput
{
   private double omega0;

   private final FramePoint2D capturePoint = new FramePoint2D();
   private final FrameVector2D capturePointVelocity = new FrameVector2D();
   private final FramePoint2D desiredCapturePoint = new FramePoint2D();
   private final FrameVector2D desiredCapturePointVelocity = new FrameVector2D();
   private final FramePoint2D finalDesiredCapturePoint = new FramePoint2D();

   private final FixedFramePoint2DBasics perfectCMP = new FramePoint2D();
   private final FixedFramePoint2DBasics perfectCoP = new FramePoint2D();

   private final FrameVector3D achievedLinearMomentumRate = new FrameVector3D();

   private boolean controlHeightWithMomentum;
   private double desiredCoMHeightAcceleration = 0.0;

   private RobotSide supportSide = null;
   private RobotSide transferToSide = null;

   private boolean initializeForStanding;
   private boolean initializeForSingleSupport;
   private boolean initializeForTransfer;
   private boolean keepCoPInsideSupportPolygon;
   private boolean minimizeAngularMomentumRateZ;
   private boolean updatePlanarRegions;
   private double finalTransferDuration;
   private double remainingTimeInSwingUnderDisturbance;
   private final RecyclingArrayList<Footstep> footsteps = new RecyclingArrayList<>(Footstep.class);
   private final RecyclingArrayList<FootstepTiming> footstepTimings = new RecyclingArrayList<>(FootstepTiming.class);
   private final RecyclingArrayList<PlanarRegion> planarRegions = new RecyclingArrayList<>(PlanarRegion.class);

   private final SideDependentList<PlaneContactStateCommand> contactStateCommands = new SideDependentList<>(new PlaneContactStateCommand(),
                                                                                                            new PlaneContactStateCommand());

   public void setOmega0(double omega0)
   {
      this.omega0 = omega0;
   }

   public double getOmega0()
   {
      return omega0;
   }

   @Deprecated // TODO: this should be computed inside the module and be an output rather then an input.
   public void setCapturePoint(FramePoint2DReadOnly capturePoint)
   {
      this.capturePoint.setIncludingFrame(capturePoint);
   }

   public FramePoint2DReadOnly getCapturePoint()
   {
      return capturePoint;
   }

   @Deprecated // TODO: this should be computed inside the module and be an output rather then an input.
   public void setCapturePointVelocity(FrameVector2DReadOnly capturePointVelocity)
   {
      this.capturePointVelocity.setIncludingFrame(capturePointVelocity);
   }

   public FrameVector2DReadOnly getCapturePointVelocity()
   {
      return capturePointVelocity;
   }

   public void setDesiredCapturePoint(FramePoint2DReadOnly desiredCapturePoint)
   {
      this.desiredCapturePoint.setIncludingFrame(desiredCapturePoint);
   }

   public FramePoint2DReadOnly getDesiredCapturePoint()
   {
      return desiredCapturePoint;
   }

   public void setDesiredCapturePointVelocity(FrameVector2DReadOnly desiredCapturePointVelocity)
   {
      this.desiredCapturePointVelocity.setIncludingFrame(desiredCapturePointVelocity);
   }

   public FrameVector2DReadOnly getDesiredCapturePointVelocity()
   {
      return desiredCapturePointVelocity;
   }

   @Deprecated // TODO: This should not be coming from the walking controller.
   public void setDesiredCenterOfMassHeightAcceleration(double desiredCoMHeightAcceleration)
   {
      this.desiredCoMHeightAcceleration = desiredCoMHeightAcceleration;
   }

   public double getDesiredCoMHeightAcceleration()
   {
      return desiredCoMHeightAcceleration;
   }

   public void setMinimizeAngularMomentumRateZ(boolean minimizeAngularMomentumRateZ)
   {
      this.minimizeAngularMomentumRateZ = minimizeAngularMomentumRateZ;
   }

   public boolean getMinimizeAngularMomentumRateZ()
   {
      return minimizeAngularMomentumRateZ;
   }

   public void setFinalDesiredCapturePoint(FramePoint2DReadOnly finalDesiredCapturePoint)
   {
      this.finalDesiredCapturePoint.setIncludingFrame(finalDesiredCapturePoint);
   }

   public FramePoint2DReadOnly getFinalDesiredCapturePoint()
   {
      return finalDesiredCapturePoint;
   }

   public void setPerfectCMP(FramePoint2DReadOnly perfectCMP)
   {
      this.perfectCMP.setMatchingFrame(perfectCMP);
   }

   public FramePoint2DReadOnly getPerfectCMP()
   {
      return perfectCMP;
   }

   public void setPerfectCoP(FramePoint2DReadOnly perfectCoP)
   {
      this.perfectCoP.setMatchingFrame(perfectCoP);
   }

   public FramePoint2DReadOnly getPerfectCoP()
   {
      return perfectCoP;
   }

   @Deprecated // TODO: This is used to compute and visualize the achieved CMP only. Lets do that somewhere else and not pass this around needlessly.
   public void setAchievedLinearMomentumRate(FrameVector3DReadOnly achievedLinearMomentumRate)
   {
      this.achievedLinearMomentumRate.setIncludingFrame(achievedLinearMomentumRate);
   }

   public FrameVector3DReadOnly getAchievedLinearMomentumRate()
   {
      return achievedLinearMomentumRate;
   }

   public void setControlHeightWithMomentum(boolean controlHeightWithMomentum)
   {
      this.controlHeightWithMomentum = controlHeightWithMomentum;
   }

   public boolean getControlHeightWithMomentum()
   {
      return controlHeightWithMomentum;
   }

   public void setSupportSide(RobotSide supportSide)
   {
      this.supportSide = supportSide;
   }

   public RobotSide getSupportSide()
   {
      return supportSide;
   }

   public void setTransferToSide(RobotSide transferToSide)
   {
      this.transferToSide = transferToSide;
   }

   public RobotSide getTransferToSide()
   {
      return transferToSide;
   }

   public void setFootsteps(List<Footstep> footsteps)
   {
      this.footsteps.clear();
      for (int i = 0; i < footsteps.size(); i++)
      {
         this.footsteps.add().set(footsteps.get(i));
      }
   }

   public List<Footstep> getFootsteps()
   {
      return footsteps;
   }

   public void setFootstepTimings(List<FootstepTiming> footstepTimings)
   {
      this.footstepTimings.clear();
      for (int i = 0; i < footsteps.size(); i++)
      {
         this.footstepTimings.add().set(footstepTimings.get(i));
      }
   }

   public List<FootstepTiming> getFootstepTimings()
   {
      return footstepTimings;
   }

   public void setFinalTransferDuration(double finalTransferDuration)
   {
      this.finalTransferDuration = finalTransferDuration;
   }

   public double getFinalTransferDuration()
   {
      return finalTransferDuration;
   }

   public void setInitializeForStanding(boolean initializeForStanding)
   {
      this.initializeForStanding = initializeForStanding;
   }

   public boolean getInitializeForStanding()
   {
      return initializeForStanding;
   }

   public void setInitializeForSingleSupport(boolean initializeForSingleSupport)
   {
      this.initializeForSingleSupport = initializeForSingleSupport;
   }

   public boolean getInitializeForSingleSupport()
   {
      return initializeForSingleSupport;
   }

   public void setInitializeForTransfer(boolean initializeForTransfer)
   {
      this.initializeForTransfer = initializeForTransfer;
   }

   public boolean getInitializeForTransfer()
   {
      return initializeForTransfer;
   }

   public void setRemainingTimeInSwingUnderDisturbance(double remainingTimeInSwingUnderDisturbance)
   {
      this.remainingTimeInSwingUnderDisturbance = remainingTimeInSwingUnderDisturbance;
   }

   public double getRemainingTimeInSwingUnderDisturbance()
   {
      return remainingTimeInSwingUnderDisturbance;
   }

   @Deprecated // TODO: This should not be coming from the walking controller. Listen to the planar regions message.
   public void setUpdatePlanarRegions(boolean updatePlanarRegions)
   {
      this.updatePlanarRegions = updatePlanarRegions;
   }

   public boolean getUpdatePlanarRegions()
   {
      return updatePlanarRegions;
   }

   @Deprecated // TODO: This should not be coming from the walking controller. Listen to the planar regions message.
   public void setPlanarRegions(List<PlanarRegion> planarRegions)
   {
      this.planarRegions.clear();
      for (int i = 0; i < planarRegions.size(); i++)
      {
         this.planarRegions.add().set(planarRegions.get(i));
      }
   }

   public List<PlanarRegion> getPlanarRegions()
   {
      return planarRegions;
   }

   public void setKeepCoPInsideSupportPolygon(boolean keepCoPInsideSupportPolygon)
   {
      this.keepCoPInsideSupportPolygon = keepCoPInsideSupportPolygon;
   }

   public boolean getKeepCoPInsideSupportPolygon()
   {
      return keepCoPInsideSupportPolygon;
   }

   public void setContactStateCommand(SideDependentList<PlaneContactStateCommand> contactStateCommands)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         this.contactStateCommands.get(robotSide).set(contactStateCommands.get(robotSide));
      }
   }

   public SideDependentList<PlaneContactStateCommand> getContactStateCommands()
   {
      return contactStateCommands;
   }
}
