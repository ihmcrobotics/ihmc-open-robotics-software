package us.ihmc.commonWalkingControlModules.controlModuleInterfaces;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.utilities.math.geometry.FrameConvexPolygon2dAndConnectingEdges;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.robotSide.SideDependentList;

public interface VirtualToePointCalculator
{
   /**
    * Computes and packs the virtual toe points, based on the desired Center of Pressure
    * @param virtualToePoints a SideDependentList of FramePoint2ds, which are expressed in each foot's ankleZUpFrame
    * @param legStrengths a SideDependentList of Doubles, expressing how much weight is on each foot
    * @param CoPDesiredInMidFeetZUp the desired CoP to be realized by the virtual toe points and the leg strengths. Must be inside support polygon.
    * @param upcomingSupportSide upcoming leg to support on. null signifies staying in double support
    */
   public abstract void packVirtualToePoints(SideDependentList<FramePoint2d> virtualToePoints, BipedSupportPolygons bipedSupportPolygons, FramePoint2d coPDesired, RobotSide upcomingSupportSide);

   public abstract void packVirtualToePoints(SideDependentList<FramePoint2d> originalVirtualToePoints, FramePoint2d copDesired,
         FrameConvexPolygon2dAndConnectingEdges supportPolygonAndEdges, RobotSide upcomingSupportSide);
}
