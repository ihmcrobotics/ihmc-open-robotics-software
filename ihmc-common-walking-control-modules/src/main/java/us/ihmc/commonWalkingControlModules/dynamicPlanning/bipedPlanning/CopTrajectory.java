package us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning;

import java.awt.Color;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.function.ObjDoubleConsumer;

import org.apache.commons.math3.util.Precision;

import gnu.trove.list.TDoubleList;
import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.ContactStateProvider;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.SettableContactStateProvider;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPosition;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint2D;
import us.ihmc.yoVariables.registry.YoRegistry;

public class CopTrajectory
{
   private final RecyclingArrayList<SettableContactStateProvider> contactStateProviders = new RecyclingArrayList<>(50, SettableContactStateProvider::new);

   private final List<YoFramePoint2D> yoWaypoints = new ArrayList<>();

   private final LineSegment2D tempLine = new LineSegment2D();

   public CopTrajectory()
   {
      this(null, null);
   }

   public CopTrajectory(YoRegistry parentRegistry, YoGraphicsListRegistry graphicsRegistry)
   {
      YoRegistry registry = new YoRegistry(getClass().getSimpleName());
      for (int i = 0; i < 10; i++)
         yoWaypoints.add(new YoFramePoint2D("CopWaypoint" + i, ReferenceFrame.getWorldFrame(), registry));

      if (graphicsRegistry != null)
      {
         for (int i = 0; i < yoWaypoints.size(); i++)
         {
            YoArtifactPosition artifact = new YoArtifactPosition("CopWaypoint" + i, yoWaypoints.get(i), GraphicType.DIAMOND, Color.GREEN, 0.002);
            graphicsRegistry.registerArtifact("CopWaypoints", artifact);
         }
      }

      if (parentRegistry != null)
         parentRegistry.addChild(registry);
   }

   public void set(Point2DReadOnly constantCop)
   {
      clear();

      SettableContactStateProvider contactState = contactStateProviders.add();
      contactState.getTimeInterval().setInterval(0.0, Double.POSITIVE_INFINITY);
      contactState.setStartCopPosition(constantCop);
      contactState.setEndCopPosition(constantCop);

      updateViz();
   }

   public void set(List<ConvexPolygon2D> supportPolygons, TDoubleList supportTimes, double finalTransferDuration)
   {
      update(supportPolygons, supportTimes, finalTransferDuration);
   }

   public void update(SupportSequence supportSeqence, double finalTransferDuration)
   {
      update(supportSeqence.getSupportPolygons(), supportSeqence.getSupportTimes(), finalTransferDuration);
   }

   public void update(List<? extends ConvexPolygon2DReadOnly> supportPolygons, TDoubleList supportTimes, double finalTransferDuration)
   {
      clear();

      // First waypoint is a center of initial support.
      Point2DReadOnly centroid = supportPolygons.get(0).getCentroid();
      SettableContactStateProvider contactStateProvider = contactStateProviders.add();
      contactStateProvider.setStartCopPosition(centroid);
      contactStateProvider.getTimeInterval().setStartTime(0.0);

      for (int i = 1; i < supportPolygons.size(); i++)
         addPolygon(supportPolygons, supportTimes, i);

      // Last waypoint is at center of final support.
      contactStateProviders.getLast().setEndCopPosition(supportPolygons.get(supportPolygons.size() - 1).getCentroid());
      contactStateProviders.getLast().getTimeInterval().setEndTime(supportTimes.get(supportTimes.size() - 1) + finalTransferDuration);

      updateViz();
   }

   private final Point2D waypoint = new Point2D();

   private void addPolygon(List<? extends ConvexPolygon2DReadOnly> supportPolygons, TDoubleList supportTimes, int i)
   {
      ConvexPolygon2DReadOnly previousPolygon = supportPolygons.get(i - 1);
      ConvexPolygon2DReadOnly polygon = supportPolygons.get(i);
      Point2DReadOnly centroid = polygon.getCentroid();

      SettableContactStateProvider previousContactState = contactStateProviders.getLast();
      SettableContactStateProvider contactState = contactStateProviders.add();

      // Add a waypoint at the start time of this polygon.
      previousContactState.getTimeInterval().setEndTime(supportTimes.get(i));
      contactState.getTimeInterval().setStartTime(supportTimes.get(i));
      waypoint.set(centroid);

      // This waypoint must be both in this polygon and the previous one:
      if (!previousPolygon.isPointInside(waypoint))
      {
         // The line intersection works better then orthogonal projection for funny shaped feet.
         tempLine.set(previousPolygon.getCentroid(), centroid);
         previousPolygon.intersectionWith(tempLine, waypoint, waypoint);
      }

      previousContactState.setEndCopPosition(waypoint);
      contactState.setStartCopPosition(waypoint);
   }

   public List<? extends ContactStateProvider> getContactStates()
   {
      return contactStateProviders;
   }

   private void updateViz()
   {
      int max = Math.min(yoWaypoints.size(), contactStateProviders.size());
      for (int i = 0; i < max; i++)
         yoWaypoints.get(i).set(contactStateProviders.get(i).getCopStartPosition());
      for (int i = max; i < yoWaypoints.size(); i++)
         yoWaypoints.get(i).setToNaN();
   }

   private void clear()
   {
      contactStateProviders.clear();
   }
}