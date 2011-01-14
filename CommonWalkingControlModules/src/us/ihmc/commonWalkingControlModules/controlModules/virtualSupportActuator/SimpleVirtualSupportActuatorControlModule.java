package us.ihmc.commonWalkingControlModules.controlModules.virtualSupportActuator;


import us.ihmc.commonWalkingControlModules.RobotSide;
import us.ihmc.commonWalkingControlModules.SideDependentList;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedLegStrengthAndVirtualToePoint;
import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.DoubleSupportForceDistributor;
import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.VirtualSupportActuatorControlModule;
import us.ihmc.commonWalkingControlModules.kinematics.StanceFullLegJacobian;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegTorques;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LowerBodyTorques;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.Wrench;

public class SimpleVirtualSupportActuatorControlModule implements VirtualSupportActuatorControlModule
{
   private final DoubleSupportForceDistributor doubleSupportForceDistributor;
   private final SideDependentList<StanceFullLegJacobian> stanceFullLegJacobians;
   private final SideDependentList<ReferenceFrame> anklePitchFrames;
   private final SideDependentList<ReferenceFrame> ankleZUpFrames;
   private final SideDependentList<ReferenceFrame> legAttachmentPointFrames;

   public SimpleVirtualSupportActuatorControlModule(DoubleSupportForceDistributor doubleSupportForceDistributor, CommonWalkingReferenceFrames referenceFrames,
           SideDependentList<StanceFullLegJacobian> stanceFullLegJacobians, SideDependentList<ReferenceFrame> legAttachmentPointFrames)
   {
      this.doubleSupportForceDistributor = doubleSupportForceDistributor;
      this.stanceFullLegJacobians = new SideDependentList<StanceFullLegJacobian>(stanceFullLegJacobians);
      this.legAttachmentPointFrames = new SideDependentList<ReferenceFrame>(legAttachmentPointFrames);
      anklePitchFrames = new SideDependentList<ReferenceFrame>(referenceFrames.getFootFrame(RobotSide.LEFT),
            referenceFrames.getFootFrame(RobotSide.RIGHT));
      ankleZUpFrames = referenceFrames.getAnkleZUpReferenceFrames();
   }


   public void controlDoubleSupport(LowerBodyTorques lowerBodyTorquesToPack, 
         SideDependentList<BipedLegStrengthAndVirtualToePoint> legStrengthsAndVirtualToePoints, 
         double fZOnPelvisInPelvisFrame, FrameVector torqueOnPelvis)
   {
      // distribute forces
      SideDependentList<Double> fZs = new SideDependentList<Double>();
      SideDependentList<FrameVector> torques = new SideDependentList<FrameVector>();
      doubleSupportForceDistributor.packForcesAndTorques(fZs, torques, fZOnPelvisInPelvisFrame, torqueOnPelvis, legStrengthsAndVirtualToePoints);


      for (RobotSide robotSide : RobotSide.values())
      {
         LegTorques supportLegTorquesToPack = lowerBodyTorquesToPack.getLegTorques(robotSide);
         FramePoint2d vtp = legStrengthsAndVirtualToePoints.get(robotSide).getVirtualToePointCopy();
         double fZ = fZs.get(robotSide);
         FrameVector torque = torques.get(robotSide);

         controlSingleSupport(supportLegTorquesToPack, vtp, fZ, torque, null);
      }
   }


   public void controlSingleSupport(LegTorques supportLegTorquesToPack, FramePoint2d virtualToePoint, double fZOnPelvisInPelvisFrame,
                                    FrameVector torqueOnPelvisInPelvisFrame, Wrench upperBodyWrench)
   {
      RobotSide stanceSide = supportLegTorquesToPack.getRobotSide();
      RobotSide swingSide = stanceSide.getOppositeSide();

      StanceFullLegJacobian stanceFullLegJacobian = stanceFullLegJacobians.get(stanceSide);

      // compute Jacobian
      // TODO: Check all this reference frame stuff and make sure the Jacobians are using the correct frames. It may be that everything only works on flat ground.
      // TODO: Line below is BAD and we need a fix. VTPs need to be in foot frames, but we are computing them in ZUp Frames.
      // TODO: Still not OK. DoubleSupport uses Zup frames
      // TODO: Still not OK, but at least the frame in which the VTP is given is not implied to be the AnkleZUpframe anymore.
      // anklePitchFrames.get(robotSide).checkReferenceFrameMatch(virtualToePoint.getReferenceFrame());
      virtualToePoint = virtualToePoint.changeFrameCopy(ankleZUpFrames.get(stanceSide));
      virtualToePoint = new FramePoint2d(anklePitchFrames.get(stanceSide), virtualToePoint.getX(), virtualToePoint.getY());
      stanceFullLegJacobian.computeJacobians(virtualToePoint);

      // add torque part of upper body wrench // TODO: think about this some more
      if (upperBodyWrench != null)
      {
         upperBodyWrench.changeFrame(legAttachmentPointFrames.get(swingSide));
         FrameVector torque = new FrameVector(upperBodyWrench.getExpressedInFrame(), upperBodyWrench.getTorque());
         torque.changeFrame(torqueOnPelvisInPelvisFrame.getReferenceFrame());
         torqueOnPelvisInPelvisFrame.add(torque);
      }

      // compute a wrench in the nullspace of the VTP columns of the Jacobian
      Wrench wrenchOnPelvisInPelvisFrame = stanceFullLegJacobian.getWrenchInVTPNullSpace(fZOnPelvisInPelvisFrame, torqueOnPelvisInPelvisFrame);
      
      // compute joint torques
      stanceFullLegJacobian.packLegTorques(supportLegTorquesToPack, wrenchOnPelvisInPelvisFrame);

   }
}

