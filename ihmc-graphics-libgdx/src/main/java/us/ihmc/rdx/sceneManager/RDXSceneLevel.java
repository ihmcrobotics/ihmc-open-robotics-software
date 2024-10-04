package us.ihmc.rdx.sceneManager;

import java.util.Set;

public enum RDXSceneLevel
{
   GROUND_TRUTH, MODEL, VIRTUAL, VR_EYE_LEFT, VR_EYE_RIGHT;

   public final Set<RDXSceneLevel> SINGLETON_SET = Set.of(this);

   RDXSceneLevel()
   {
   }
}
