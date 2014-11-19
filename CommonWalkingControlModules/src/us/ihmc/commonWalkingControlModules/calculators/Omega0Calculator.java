package us.ihmc.commonWalkingControlModules.calculators;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.controlModules.CenterOfPressureResolver;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.OriginAndPointFrame;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.SpatialForceVector;

public class Omega0Calculator implements Omega0CalculatorInterface
{
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final CenterOfPressureResolver centerOfPressureResolver = new CenterOfPressureResolver();
   private final OriginAndPointFrame copToCoPFrame = new OriginAndPointFrame("copToCoP", worldFrame);
   private final ReferenceFrame centerOfMassFrame;
   private final double totalMass;
   private final List<FramePoint> cops = new ArrayList<FramePoint>(2); // Max of 2 CoPs assumed here
   private final FramePoint2d pseudoCoP2d = new FramePoint2d();
   private final FramePoint pseudoCoP = new FramePoint();
   private final SpatialForceVector totalGroundReactionWrench = new SpatialForceVector();

   public Omega0Calculator(ReferenceFrame centerOfMassFrame, double totalMass)
   {
      this.centerOfMassFrame = centerOfMassFrame;
      this.totalMass = totalMass;
      
      for (int i = 0; i < 2; i++) // Max of 2 CoPs assumed here
         cops.add(new FramePoint());
   }

   public double computeOmega0(List<FramePoint2d> cop2ds, SpatialForceVector newTotalGroundReactionWrench)
   {
      totalGroundReactionWrench.set(newTotalGroundReactionWrench);
      totalGroundReactionWrench.changeFrame(centerOfMassFrame);
      double fz = totalGroundReactionWrench.getLinearPartZ();

      double deltaZ;
      if (cop2ds.size() == 1)
      {
         FramePoint cop = cop2ds.get(0).toFramePoint();
         cop.changeFrame(centerOfMassFrame);
         deltaZ = -cop.getZ();
      }
      else    // assume 2 CoPs
      {
         for (int i = 0; i < 2; i++)
         {
            FramePoint2d cop2d = cop2ds.get(i);
            cops.get(i).setIncludingFrame(cop2d.getReferenceFrame(), cop2d.getX(), cop2d.getY(), 0.0);
            cops.get(i).changeFrame(copToCoPFrame.getParent());
         }

         copToCoPFrame.setOriginAndPositionToPointAt(cops.get(0), cops.get(1));
         copToCoPFrame.update();
         pseudoCoP2d.setToZero(copToCoPFrame);
         centerOfPressureResolver.resolveCenterOfPressureAndNormalTorque(pseudoCoP2d, totalGroundReactionWrench, copToCoPFrame);
         pseudoCoP.setXYIncludingFrame(pseudoCoP2d);
         pseudoCoP.changeFrame(centerOfMassFrame);
         deltaZ = -pseudoCoP.getZ();
      }

      double omega0 = Math.sqrt(fz / (totalMass * deltaZ));
      if (Double.isNaN(omega0))
         throw new RuntimeException("omega0 is NaN");
      return omega0;
   }

}
