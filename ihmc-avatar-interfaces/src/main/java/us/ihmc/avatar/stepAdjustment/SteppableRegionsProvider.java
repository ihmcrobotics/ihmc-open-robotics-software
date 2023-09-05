package us.ihmc.avatar.stepAdjustment;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PlanarRegionsListCommand;
import us.ihmc.robotics.geometry.PlanarRegion;

import java.util.List;

/**
 * Class that provides the regions that the robot can step on, if a planar reigons list is provided.
 */
public interface SteppableRegionsProvider
{
   void consume(PlanarRegionsListCommand command);

   List<PlanarRegion> getSteppableRegions();

   ConvexPolygon2DReadOnly getConvexHullOfAllRegions();
}
