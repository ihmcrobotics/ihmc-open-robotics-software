package us.ihmc.commonWalkingControlModules.calculators;

import us.ihmc.commonWalkingControlModules.controlModules.CenterOfPressureResolver;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.referenceFrames.OriginAndPointFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.SpatialForceVector;

public class Omega0Calculator implements Omega0CalculatorInterface
{
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final CenterOfPressureResolver centerOfPressureResolver = new CenterOfPressureResolver();
   private final OriginAndPointFrame copToCoPFrame = new OriginAndPointFrame("copToCoP", worldFrame);
   private final ReferenceFrame centerOfMassFrame;
   private final double totalMass;
   private final SideDependentList<FramePoint3D> cops = new SideDependentList<>(); // Max of 2 CoPs assumed here
   private final FramePoint2D pseudoCoP2d = new FramePoint2D();
   private final FramePoint3D pseudoCoP = new FramePoint3D();
   private final SpatialForceVector totalGroundReactionWrench = new SpatialForceVector();

   private double omega0;

   public Omega0Calculator(ReferenceFrame centerOfMassFrame, double totalMass, double initialOmega0)
   {
      this.centerOfMassFrame = centerOfMassFrame;
      this.totalMass = totalMass;

      omega0 = initialOmega0;

      for (RobotSide robotSide : RobotSide.values) // Max of 2 CoPs assumed here
         cops.put(robotSide, new FramePoint3D());
   }

   private final FramePoint3D tempCoP3d = new FramePoint3D();

   public double computeOmega0(SideDependentList<FramePoint2D> cop2ds, SpatialForceVector newTotalGroundReactionWrench)
   {
      totalGroundReactionWrench.set(newTotalGroundReactionWrench);
      totalGroundReactionWrench.changeFrame(centerOfMassFrame);
      double fz = totalGroundReactionWrench.getLinearPartZ();

      int numberOfValidCoPs = 0;
      for (RobotSide robotSide : RobotSide.values)
         numberOfValidCoPs += cop2ds.get(robotSide).containsNaN() ? 0 : 1;

      double deltaZ = Double.NaN;
      if (numberOfValidCoPs == 1)
      {
         for (RobotSide robotSide : RobotSide.values)
         {
            FramePoint2D cop2d = cop2ds.get(robotSide);
            if (!cop2d.containsNaN())
            {
               tempCoP3d.setIncludingFrame(cop2d, 0.0);
               tempCoP3d.changeFrame(centerOfMassFrame);
               deltaZ = -tempCoP3d.getZ();
               break;
            }
         }
      }
      else // assume 2 CoPs
      {
         for (RobotSide robotSide : RobotSide.values)
         {
            FramePoint2D cop2d = cop2ds.get(robotSide);
            cops.get(robotSide).setIncludingFrame(cop2d.getReferenceFrame(), cop2d.getX(), cop2d.getY(), 0.0);
            cops.get(robotSide).changeFrame(copToCoPFrame.getParent());
         }

         copToCoPFrame.setOriginAndPositionToPointAt(cops.get(RobotSide.LEFT), cops.get(RobotSide.RIGHT));
         copToCoPFrame.update();
         pseudoCoP2d.setToZero(copToCoPFrame);
         centerOfPressureResolver.resolveCenterOfPressureAndNormalTorque(pseudoCoP2d, totalGroundReactionWrench, copToCoPFrame);
         pseudoCoP.setIncludingFrame(pseudoCoP2d, 0.0);
         pseudoCoP.changeFrame(centerOfMassFrame);
         deltaZ = -pseudoCoP.getZ();
      }

      double newOmega0 = Math.sqrt(fz / (totalMass * deltaZ));
      if (!Double.isNaN(newOmega0))
         omega0 = newOmega0;
      return omega0;
   }

}
