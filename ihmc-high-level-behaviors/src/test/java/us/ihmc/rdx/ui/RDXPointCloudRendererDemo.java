package us.ihmc.rdx.ui;

import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.rdx.RDXPointCloudRenderer;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;

import java.util.Random;

public class RDXPointCloudRendererDemo
{
   private final RecyclingArrayList<Point3D32> points = new RecyclingArrayList<>(500, Point3D32::new);
   private final Random random = new Random();

   public RDXPointCloudRendererDemo()
   {
      RDXBaseUI baseUI = new RDXBaseUI();
      RDXPointCloudRenderer pointCloudRenderer = new RDXPointCloudRenderer();
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            baseUI.getPrimaryScene().addCoordinateFrame(0.3);

            pointCloudRenderer.create(5000);
            baseUI.getPrimaryScene().addRenderableProvider(pointCloudRenderer, RDXSceneLevel.VIRTUAL);
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

            pointCloudRenderer.updateMesh();

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            baseUI.dispose();
         }
      });
   }

   public static void main(String[] args)
   {
      new RDXPointCloudRendererDemo();
   }
}
