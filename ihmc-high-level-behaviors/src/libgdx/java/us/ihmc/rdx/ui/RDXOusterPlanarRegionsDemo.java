package us.ihmc.rdx.ui;

import us.ihmc.bytedeco.mapsenseWrapper.MapsenseWrapper;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.perception.BytedecoTools;
import us.ihmc.perception.OusterRegionsCalculator;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.RDXPointCloudRenderer;
import us.ihmc.scs2.session.DaemonThreadFactory;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

public class RDXOusterPlanarRegionsDemo
{
   private final int MAX_POINTS = 100000;

   private final RDXBaseUI baseUI = new RDXBaseUI(getClass(), "ihmc-open-robotics-software", "ihmc-perception/src/mapsense-wrapper/resources");
   private final RDXPointCloudRenderer pointCloudRenderer;
   private final RecyclingArrayList<Point3D32> pointsToRender = new RecyclingArrayList<>(MAX_POINTS, Point3D32::new);
   private final MapsenseWrapper.MapsenseExternal mapsense;
   private final OusterRegionsCalculator ousterRegionsCalculator;
   private final ScheduledExecutorService backgroundExecutor = Executors.newScheduledThreadPool(1,
                                                                                                new DaemonThreadFactory(getClass().getSimpleName(),
                                                                                                                        Thread.NORM_PRIORITY));

   public RDXOusterPlanarRegionsDemo()
   {
      BytedecoTools.loadMapsenseLibraries();
      mapsense = new MapsenseWrapper.MapsenseExternal();
      ousterRegionsCalculator = new OusterRegionsCalculator(mapsense);
      pointCloudRenderer = new RDXPointCloudRenderer();


      ousterRegionsCalculator.getPointCloud(pointsToRender);



      backgroundExecutor.scheduleAtFixedRate(this::update, 0, 100L, TimeUnit.MILLISECONDS);

      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            pointCloudRenderer.create(400000);
            pointCloudRenderer.setPointsToRender(pointsToRender);
            pointCloudRenderer.updateMesh();

            baseUI.getPrimaryScene().addRenderableProvider(pointCloudRenderer);
         }

         @Override
         public void render()
         {
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

   private void update()
   {
      System.out.println("Extracting Planar Regions using Mapsense.");
      ousterRegionsCalculator.update();
   }

   public static void main(String[] args)
   {
      new RDXOusterPlanarRegionsDemo();
   }
}
