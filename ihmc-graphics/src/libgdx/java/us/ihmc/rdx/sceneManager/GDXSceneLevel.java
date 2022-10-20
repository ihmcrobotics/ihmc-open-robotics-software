package us.ihmc.rdx.sceneManager;

import java.util.Set;

public enum GDXSceneLevel
{
   GROUND_TRUTH, MODEL, VIRTUAL;

   public final Set<GDXSceneLevel> SINGLETON_SET = Set.of(this);

   GDXSceneLevel()
   {
   }
}
