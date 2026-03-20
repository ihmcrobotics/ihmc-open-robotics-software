package us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose2DReadOnly;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.yoGraphic.SCS2YoGraphicHolder;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.function.IntFunction;

import static us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory.newYoGraphicArrow3D;
import static us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory.newYoGraphicPoint3D;

/**
 * Visualizes the next {@value #NUM_WAYPOINTS} upcoming waypoints queued in a
 * {@link PDVelocityBasedGoalReacher}.
 *
 * <p>Each waypoint is shown as:
 * <ul>
 *   <li>A blue sphere at the waypoint's XY position.</li>
 *   <li>A blue arrow pointing in the waypoint's heading direction.</li>
 * </ul>
 * Consecutive waypoints are connected by a thin blue line segment.
 */
public class GoalReacherWaypointVisualizer implements SCS2YoGraphicHolder
{
   static final int NUM_WAYPOINTS = 6;

   private static final double WAYPOINT_Z_OFFSET = 0.1;
   private static final double BALL_SIZE = 0.08;
   private static final double ORIENTATION_ARROW_LENGTH = 0.4;
   private static final double CONNECTION_LINE_RADIUS = 0.01;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   /** World-frame position of each upcoming waypoint (NaN when the slot is unused). */
   private final YoFramePoint3D[] positions = new YoFramePoint3D[NUM_WAYPOINTS];

   /** Unit-scaled heading vector for each waypoint's orientation arrow (NaN when unused). */
   private final YoFrameVector3D[] orientationDirections = new YoFrameVector3D[NUM_WAYPOINTS];

   /**
    * Displacement vector from waypoint {@code i} to waypoint {@code i+1}, used to draw the
    * connecting line segment (NaN when either endpoint is unused).
    */
   private final YoFrameVector3D[] connectionVectors = new YoFrameVector3D[NUM_WAYPOINTS - 1];

   public GoalReacherWaypointVisualizer(YoRegistry parentRegistry)
   {
      for (int i = 0; i < NUM_WAYPOINTS; i++)
      {
         positions[i] = new YoFramePoint3D("upcomingWaypointPosition" + i, ReferenceFrame.getWorldFrame(), registry);
         positions[i].setToNaN();

         orientationDirections[i] = new YoFrameVector3D("upcomingWaypointDirection" + i, ReferenceFrame.getWorldFrame(), registry);
         orientationDirections[i].setToNaN();
      }

      for (int i = 0; i < NUM_WAYPOINTS - 1; i++)
      {
         connectionVectors[i] = new YoFrameVector3D("upcomingWaypointConnection" + i, ReferenceFrame.getWorldFrame(), registry);
         connectionVectors[i].setToNaN();
      }

      parentRegistry.addChild(registry);
   }

   /**
    * Updates all YoVariables from the currently queued waypoints.
    *
    * @param waypointCount total number of waypoints currently in the queue
    * @param waypointPoseAt function that returns the {@link FramePose2DReadOnly} for index {@code i}
    */
   public void update(int waypointCount, IntFunction<FramePose2DReadOnly> waypointPoseAt)
   {
      int visible = Math.min(waypointCount, NUM_WAYPOINTS);

      for (int i = 0; i < NUM_WAYPOINTS; i++)
      {
         if (i < visible)
         {
            FramePose2DReadOnly pose = waypointPoseAt.apply(i);
            positions[i].set(pose.getX(), pose.getY(), WAYPOINT_Z_OFFSET);

            double yaw = pose.getYaw();
            orientationDirections[i].set(Math.cos(yaw) * ORIENTATION_ARROW_LENGTH,
                                         Math.sin(yaw) * ORIENTATION_ARROW_LENGTH,
                                         0.0);
         }
         else
         {
            positions[i].setToNaN();
            orientationDirections[i].setToNaN();
         }
      }

      for (int i = 0; i < NUM_WAYPOINTS - 1; i++)
      {
         if (i + 1 < visible)
         {
            connectionVectors[i].set(positions[i + 1].getX() - positions[i].getX(),
                                     positions[i + 1].getY() - positions[i].getY(),
                                     positions[i + 1].getZ() - positions[i].getZ());
         }
         else
         {
            connectionVectors[i].setToNaN();
         }
      }
   }

   @Override
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(getClass().getSimpleName());

      for (int i = 0; i < NUM_WAYPOINTS; i++)
      {
         group.addChild(newYoGraphicPoint3D("Waypoint " + i, positions[i], BALL_SIZE, ColorDefinitions.Blue()));
         group.addChild(newYoGraphicArrow3D("Waypoint Orientation " + i,
                                            positions[i],
                                            orientationDirections[i],
                                            1.0,
                                            ColorDefinitions.Blue()));
      }

      // Headless arrows render as thin tubes, giving the appearance of a line segment.
      for (int i = 0; i < NUM_WAYPOINTS - 1; i++)
      {
         group.addChild(newYoGraphicArrow3D("Waypoint Connection " + i,
                                            positions[i],
                                            connectionVectors[i],
                                            true, 1.0, 0.0,
                                            false, CONNECTION_LINE_RADIUS, 0.0,
                                            ColorDefinitions.Blue()));
      }

      return group;
   }
}
