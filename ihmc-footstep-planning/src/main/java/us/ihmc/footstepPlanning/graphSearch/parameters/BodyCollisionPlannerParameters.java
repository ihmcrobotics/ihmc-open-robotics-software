package us.ihmc.footstepPlanning.graphSearch.parameters;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.footstepPlanning.filters.BodyCollisionRegionFilter;
import us.ihmc.footstepPlanning.filters.SteppableRegionFilter;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.polygonWiggling.PolygonWiggler;
import us.ihmc.robotics.geometry.PlanarRegion;

public interface BodyCollisionPlannerParameters
{
   /**
    * Sets whether or not the search should check if the body is colliding with the world. This may cause the planner
    * to run slower.
    */
   default boolean checkForBodyBoxCollisions()
   {
      return false;
   }

   
   /**
    * Some node checkers will check if a bounding box that describes the body of the robot will move
    * through a planar region (e.g. a wall) when going from one footstep to the next one. To avoid these
    * collisions, this defines the box height. Note that this box will go from {@code getBodyBoxBaseZ}
    * to {@code getBodyBoxBaseHeight + getBodyBoxHeight}
    */
   default double getBodyBoxHeight()
   {
      return 1.5;
   }

   /**
    * Some node checkers will check if a bounding box that describes the body of the robot will move
    * through a planar region (e.g. a wall) when going from one footstep to the next one. To avoid these
    * collisions, this defines the box depth.
    */
   default double getBodyBoxDepth()
   {
      return 0.3;
   }

   /**
    * Some node checkers will check if a bounding box that describes the body of the robot will move
    * through a planar region (e.g. a wall) when going from one footstep to the next one. To avoid these
    * collisions, this defines the box width.
    */
   default double getBodyBoxWidth()
   {
      return 0.7;
   }

   /**
    * Some node checkers will check if a bounding box that describes the body of the robot will move
    * through a planar region (e.g. a wall) when going from one footstep to the next one. To avoid these
    * collisions, this defines the x-offset of a bounding box relative to the average frame between sequential footsteps.
    */
   default double getBodyBoxBaseX()
   {
      return 0.0;
   }

   /**
    * Some node checkers will check if a bounding box that describes the body of the robot will move
    * through a planar region (e.g. a wall) when going from one footstep to the next one. To avoid these
    * collisions, this defines the y-offset of a bounding box relative to the average frame between sequential footsteps.
    */
   default double getBodyBoxBaseY()
   {
      return 0.0;
   }

   /**
    * Some node checkers will check if a bounding box that describes the body of the robot will move
    * through a planar region (e.g. a wall) when going from one footstep to the next one. To avoid these
    * collisions, this defines the z-offset of a bounding box relative to the average frame between sequential footsteps.
    */
   default double getBodyBoxBaseZ()
   {
      return 0.25;
   }

}
