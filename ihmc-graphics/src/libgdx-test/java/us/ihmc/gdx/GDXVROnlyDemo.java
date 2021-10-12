package us.ihmc.gdx;

import us.ihmc.gdx.vr.GDXVRManager;

public class GDXVROnlyDemo
{
   private final GDXVRManager vrManager;

   public GDXVROnlyDemo()
   {
      vrManager = new GDXVRManager();
   }

   public static void main(String[] args)
   {
      new GDXVROnlyDemo();
   }
}
