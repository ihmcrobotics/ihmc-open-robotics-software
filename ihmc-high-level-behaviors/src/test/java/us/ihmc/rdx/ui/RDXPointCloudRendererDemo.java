package us.ihmc.rdx.ui;

import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.rdx.RDXPointCloudRenderer;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.sceneManager.RDX3DBareBonesScene;
import us.ihmc.rdx.sceneManager.RDX3DSceneTools;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.tools.LibGDXApplicationCreator;

import java.util.Random;

public class RDXPointCloudRendererDemo
{
   private final RecyclingArrayList<Point3D32> points = new RecyclingArrayList<>(500, Point3D32::new);
   private final Random random = new Random();

   public RDXPointCloudRendererDemo()
   {
      RDX3DBareBonesScene sceneManager = new RDX3DBareBonesScene();
      RDXPointCloudRenderer pointCloudRenderer = new RDXPointCloudRenderer();
      LibGDXApplicationCreator.launchGDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            sceneManager.create();

            sceneManager.addCoordinateFrame(0.3);

            pointCloudRenderer.create(5000);
            sceneManager.addRenderableProvider(pointCloudRenderer, RDXSceneLevel.VIRTUAL);
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

            RDX3DSceneTools.glClearGray();
            pointCloudRenderer.updateMesh();
            sceneManager.setViewportBoundsToWindow();
            sceneManager.render();
         }
      }, "RDX3DDemo", 1100, 800);
   }

   public static void main(String[] args)
   {
      new RDXPointCloudRendererDemo();
   }
}
