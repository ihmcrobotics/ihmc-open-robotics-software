package us.ihmc.humanoidRobotics.footstep.footstepGenerator;

import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;

/**
 * Created by agrabertilton on 2/20/15.
 */
public interface FootstepValidityMetric
{
   /**
    * Basic check of if taking the prospective footstep is possible.
    * @param stanceFootstep the footstep of the other leg/ the stance leg
    * @param prospectiveFootstep the prospective footstep to take.
    * @return
    */
   public boolean footstepValid(FootstepDataMessage stanceFootstep, FootstepDataMessage prospectiveFootstep);

   /**
    * More Comprehensive check that also checks things related to the initial position of the prospective foot.
    * Allows the checking of things like the swing height vs the stance foot height, line collision checks (previous to prospective does not collide with stance), etc.
    * @param previousFootstep the previous/initial position of the prospective footstep (where it would be coming from)
    * @param stanceFootstep the stance footstep (the other side of the prospective footstep)
    * @param prospectiveFootstep the prospective footstep to take.
    * @return
    */
   public boolean footstepValid(FootstepDataMessage previousFootstep, FootstepDataMessage stanceFootstep, FootstepDataMessage prospectiveFootstep);
}
