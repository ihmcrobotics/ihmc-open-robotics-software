package us.ihmc.rdx.vr.minimalGDX;

import com.badlogic.gdx.*;
import com.badlogic.gdx.utils.Clipboard;

import java.util.ArrayDeque;

public class MinimalGDXApplication implements Application
{
   private final ArrayDeque<Runnable> postRunnables = new ArrayDeque();

   @Override
   public ApplicationListener getApplicationListener()
   {
      return null;
   }

   @Override
   public Graphics getGraphics()
   {
      return Gdx.graphics;
   }

   @Override
   public Audio getAudio()
   {
      return null;
   }

   @Override
   public Input getInput()
   {
      return null;
   }

   @Override
   public Files getFiles()
   {
      return Gdx.files;
   }

   @Override
   public Net getNet()
   {
      return null;
   }

   @Override
   public void log(String tag, String message)
   {

   }

   @Override
   public void log(String tag, String message, Throwable exception)
   {

   }

   @Override
   public void error(String tag, String message)
   {

   }

   @Override
   public void error(String tag, String message, Throwable exception)
   {

   }

   @Override
   public void debug(String tag, String message)
   {

   }

   @Override
   public void debug(String tag, String message, Throwable exception)
   {

   }

   @Override
   public void setLogLevel(int logLevel)
   {

   }

   @Override
   public int getLogLevel()
   {
      return 0;
   }

   @Override
   public void setApplicationLogger(ApplicationLogger applicationLogger)
   {

   }

   @Override
   public ApplicationLogger getApplicationLogger()
   {
      return null;
   }

   @Override
   public ApplicationType getType()
   {
      return null;
   }

   @Override
   public int getVersion()
   {
      return 0;
   }

   @Override
   public long getJavaHeap()
   {
      return 0;
   }

   @Override
   public long getNativeHeap()
   {
      return 0;
   }

   @Override
   public Preferences getPreferences(String name)
   {
      return null;
   }

   @Override
   public Clipboard getClipboard()
   {
      return null;
   }

   @Override
   public void postRunnable(Runnable runnable)
   {
      postRunnables.addLast(runnable);
   }

   @Override
   public void exit()
   {

   }

   @Override
   public void addLifecycleListener(LifecycleListener listener)
   {

   }

   @Override
   public void removeLifecycleListener(LifecycleListener listener)
   {

   }

   public ArrayDeque<Runnable> getPostRunnables()
   {
      return postRunnables;
   }
}
