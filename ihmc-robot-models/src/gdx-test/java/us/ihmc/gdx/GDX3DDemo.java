package us.ihmc.gdx;

public class GDX3DDemo
{
   public GDX3DDemo()
   {
      GDX3DSceneManager sceneManager = new GDX3DSceneManager();
      GDXApplicationCreator.launchGDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            sceneManager.create();

            sceneManager.addCoordinateFrame(0.3);
            sceneManager.addModelInstance(new BoxesDemoModel().newInstance());
         }

         @Override
         public void render()
         {
            sceneManager.glClearGray();
            sceneManager.render();
         }
      }, "GDX3DDemo", 1100, 800);
   }

   public static void main(String[] args)
   {
      new GDX3DDemo();
   }
}