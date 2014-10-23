package us.ihmc.commonWalkingControlModules.bipedSupportPolygons;

import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.robotSide.RobotSide;



/**
 * <p>Title: </p>
 *
 * <p>Description: </p>
 *
 * <p>Copyright: Copyright (c) 2007</p>
 *
 * <p>Company: </p>
 *
 * @author not attributable
 * @version 1.0
 */
public interface BipedFeetUpdater
{
   public void updateBipedFeet(BipedFootInterface leftFoot, BipedFootInterface rightFoot, RobotSide supportLeg, FramePoint capturePoint, boolean forceHindOnToes);
   
   public void setResizePolygonInDoubleSupport(boolean doResize);
}

