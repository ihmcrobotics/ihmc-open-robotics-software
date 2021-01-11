package us.ihmc.gdx;

public class GDX3DDemo
{
   public GDX3DDemo()
   {
      GDX3DApplication baseApplication = new GDX3DApplication();
      GDXApplicationCreator.launchGDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseApplication.create();

            baseApplication.addCoordinateFrame(0.3);
            baseApplication.addModelInstance(new BoxesDemoModel().newInstance());
         }

         @Override
         public void render()
         {
            baseApplication.glClearGrayscale();
            baseApplication.render();
         }
      }, "GDX3DDemo", 1100, 800);
   }

   public static void main(String[] args)
   {
      new GDX3DDemo();
   }
}