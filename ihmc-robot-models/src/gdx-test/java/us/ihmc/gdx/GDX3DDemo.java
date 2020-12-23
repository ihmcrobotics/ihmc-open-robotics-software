package us.ihmc.gdx;

public class GDX3DDemo
{
   public GDX3DDemo()
   {
      GDXApplicationCreator.launchGDXApplication(new PrivateGDXApplication(), "GDX3DDemo", 1100, 800);
   }

   class PrivateGDXApplication extends GDX3DApplication
   {
      @Override
      public void create()
      {
         super.create();

         addCoordinateFrame(0.3);
         addModelInstance(new BoxesDemoModel().newInstance());
      }
   }

   public static void main(String[] args)
   {
      new GDX3DDemo();
   }
}