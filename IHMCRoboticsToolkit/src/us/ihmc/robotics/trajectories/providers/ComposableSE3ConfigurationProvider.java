package us.ihmc.robotics.trajectories.providers;

import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;

/**
 * @author twan
 *         Date: 5/21/13
 */
public class ComposableSE3ConfigurationProvider implements SE3ConfigurationProvider
{
   private final PositionProvider positionProvider;
   private final OrientationProvider orientationProvider;

   public ComposableSE3ConfigurationProvider(PositionProvider positionProvider, OrientationProvider orientationProvider)
   {
      this.positionProvider = positionProvider;
      this.orientationProvider = orientationProvider;
   }

   public void get(FrameOrientation orientationToPack)
   {
      orientationProvider.get(orientationToPack);
   }

   public void get(FramePoint positionToPack)
   {
      positionProvider.get(positionToPack);
   }
}
