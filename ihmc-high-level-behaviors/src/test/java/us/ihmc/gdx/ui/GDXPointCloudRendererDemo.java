package us.ihmc.gdx.ui;

import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.gdx.GDXPointCloudRenderer;
import us.ihmc.gdx.Lwjgl3ApplicationAdapter;
import us.ihmc.gdx.sceneManager.GDX3DBareBonesScene;
import us.ihmc.gdx.sceneManager.GDX3DSceneTools;
import us.ihmc.gdx.sceneManager.GDXSceneLevel;
import us.ihmc.gdx.tools.GDXApplicationCreator;

import java.util.Random;

public class GDXPointCloudRendererDemo
{
   private final RecyclingArrayList<Point3D32> points = new RecyclingArrayList<>(500, Point3D32::new);
   private final Random random = new Random();

   public GDXPointCloudRendererDemo()
   {
      GDX3DBareBonesScene sceneManager = new GDX3DBareBonesScene();
      GDXPointCloudRenderer pointCloudRenderer = new GDXPointCloudRenderer();
      GDXApplicationCreator.launchGDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            sceneManager.create();

            sceneManager.addCoordinateFrame(0.3);

            pointCloudRenderer.create(5000);
            sceneManager.addRenderableProvider(pointCloudRenderer, GDXSceneLevel.VIRTUAL);
         }

         @Override
         public void render()
         {
            points.clear();
            for (int i = 0; i < 5000; i++)
            {
               Point3D32 point = points.add();
               point.set(new Point3D32(random.nextFloat(), random.nextFloat(), random.nextFloat()));
            }

            pointCloudRenderer.setPointsToRender(points);

            GDX3DSceneTools.glClearGray();
            pointCloudRenderer.updateMesh();
            sceneManager.setViewportBoundsToWindow();
            sceneManager.render();
         }
      }, "GDX3DDemo", 1100, 800);
   }

   public static void main(String[] args)
   {
      new GDXPointCloudRendererDemo();
   }
}
