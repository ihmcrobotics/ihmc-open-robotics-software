package us.ihmc.robotics.trajectories.providers;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;

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

   public void getOrientation(FrameQuaternion orientationToPack)
   {
      orientationProvider.getOrientation(orientationToPack);
   }

   public void getPosition(FramePoint3D positionToPack)
   {
      positionProvider.getPosition(positionToPack);
   }
}
