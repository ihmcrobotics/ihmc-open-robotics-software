package us.ihmc.commonWalkingControlModules.trajectories;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFramePoint3D;

import java.awt.*;

public class YoCoMHeightTrajectoryWaypoint
{
   private final YoFramePoint3D waypoint;
   private final YoFramePoint3D minWaypoint;
   private final YoFramePoint3D maxWaypoint;

   public YoCoMHeightTrajectoryWaypoint(String name, ReferenceFrame referenceFrame, YoVariableRegistry registry)
   {
      waypoint = new YoFramePoint3D(name + "InWorld", referenceFrame, registry);
      minWaypoint = new YoFramePoint3D(name + "MinInWorld", referenceFrame, registry);
      maxWaypoint = new YoFramePoint3D(name + "MaxInWorld", referenceFrame, registry);
   }

   public void setMatchingFrame(CoMHeightTrajectoryWaypoint waypoint)
   {
      this.waypoint.setMatchingFrame(waypoint.getWaypoint());
      this.minWaypoint.setMatchingFrame(waypoint.getMinWaypoint());
      this.maxWaypoint.setMatchingFrame(waypoint.getMaxWaypoint());
   }

   public YoFramePoint3D getWaypointInWorld()
   {
      return waypoint;
   }

   public YoFramePoint3D getMinWaypointInWorld()
   {
      return minWaypoint;
   }

   public YoFramePoint3D getMaxWaypointInWorld()
   {
      return maxWaypoint;
   }

   public void setupViz(String graphicListName, String name, AppearanceDefinition color, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      double pointSize = 0.03;

      YoGraphicPosition pointD0Viz = new YoGraphicPosition(name, waypoint, pointSize, color);
      YoGraphicPosition pointD0MinViz = new YoGraphicPosition(name + "Min", minWaypoint, 0.8 * pointSize, color);
      YoGraphicPosition pointD0MaxViz = new YoGraphicPosition(name + "Max", maxWaypoint, 0.9 * pointSize, color);

      yoGraphicsListRegistry.registerYoGraphic(graphicListName, pointD0Viz);
      yoGraphicsListRegistry.registerYoGraphic(graphicListName, pointD0MinViz);
      yoGraphicsListRegistry.registerYoGraphic(graphicListName, pointD0MaxViz);
   }
}
