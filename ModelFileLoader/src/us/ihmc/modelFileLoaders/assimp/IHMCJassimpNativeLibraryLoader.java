package us.ihmc.modelFileLoaders.assimp;

import jassimp.JassimpLibraryLoader;
import us.ihmc.tools.nativelibraries.NativeLibraryLoader;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class IHMCJassimpNativeLibraryLoader extends JassimpLibraryLoader
{
   @Override
   public void loadLibrary()
   {
      NativeLibraryLoader.loadLibrary("jassimp", "jassimp");
   }
}
