package us.ihmc.commonWalkingControlModules.controlModuleInterfaces;

import us.ihmc.commonWalkingControlModules.SideDependentList;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.utilities.math.geometry.FramePoint2d;

public interface VirtualToePointCalculator
{
   /**
    * Computes and packs the virtual toe points and the leg strengths, based on the desired Center of Pressure
    * @param virtualToePoints a SideDependentList of FramePoint2ds, which are expressed in each foot's ankleZUpFrame
    * @param legStrengths a SideDependentList of Doubles, expressing how much weight is on each foot
    * @param CoPDesiredInMidFeetZUp the desired CoP to be realized by the virtual toe points and the leg strengths. Must be inside support polygon.
    */
   public abstract void packVirtualToePoints(SideDependentList<FramePoint2d> virtualToePoints, BipedSupportPolygons bipedSupportPolygons, FramePoint2d coPDesired);

   /**
    * Call during single support to hide the visualization graphics that will clutter things up.
    */
   public abstract void hideVisualizationGraphics();
}
