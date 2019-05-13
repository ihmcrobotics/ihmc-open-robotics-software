package us.ihmc.humanoidBehaviors.upDownExploration;

import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.messager.Messager;
import us.ihmc.tools.thread.TypedNotification;

import java.util.Optional;

/**
 * Keep track of state and manage the specific flow of exploration for the May 2019 demo.
 *
 * Flow is after reset, go up or down. if went down, turn 180. if went up, turn random.
 *
 */
public class UpDownExplorer
{
   private final UpDownFlatAreaFinder upDownFlatAreaFinder;
   private TypedNotification<Optional<FramePose3D>> planNotification = new TypedNotification<>();

   public UpDownExplorer(Messager messager)
   {
      upDownFlatAreaFinder = new UpDownFlatAreaFinder(messager);
   }

   public void navigate()
   {
      // TODO this should plan only if

      planNotification = upDownFlatAreaFinder.upOrDownOnAThread(remoteSyncedHumanoidFrames.pollHumanoidReferenceFrames().getMidFeetZUpFrame(),
                                                                    PlanarRegionMessageConverter.convertToPlanarRegionsList(planarRegionsList.getLatest()));
   }

   public void abortPlanning()
   {
      upDownFlatAreaFinder.abort();
   }

   public TypedNotification<Optional<FramePose3D>> getPlanNotification()
   {
      return planNotification;
   }
}
