package us.ihmc.commonWalkingControlModules.wrenchDistribution;

import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.DoubleSupportForceDistributor;
import us.ihmc.robotics.humanoidRobot.frames.CommonHumanoidReferenceFrames;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class SimpleDoubleSupportForceDistributor implements DoubleSupportForceDistributor
{
   private final ReferenceFrame pelvisFrame;

   public SimpleDoubleSupportForceDistributor(CommonHumanoidReferenceFrames referenceFrames)
   {
      this.pelvisFrame = referenceFrames.getPelvisFrame();
   }

   public void packForcesAndTorques(SideDependentList<Double> zForcesInPelvisFrameToPack, SideDependentList<FrameVector> torquesOnPelvis,
                                    double zForceInPelvisFrameTotal, FrameVector torqueOnPelvisTotal,
                                    SideDependentList<Double> legStrengths, SideDependentList<FramePoint2d> virtualToePoints)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         double legStrength = legStrengths.get(robotSide);

         zForcesInPelvisFrameToPack.put(robotSide, zForceInPelvisFrameTotal * legStrength);

         FrameVector torque = new FrameVector(torqueOnPelvisTotal);
         torque.changeFrame(pelvisFrame);
         torque.scale(legStrength);

         torquesOnPelvis.set(robotSide, torque);
      }
   }

}
