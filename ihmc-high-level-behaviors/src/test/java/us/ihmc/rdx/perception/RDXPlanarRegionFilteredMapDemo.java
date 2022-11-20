package us.ihmc.rdx.perception;

import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.perception.mapping.PlanarRegionFilteredMap;
import us.ihmc.perception.mapping.PlanarRegionMapHandler;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.visualizers.RDXPlanarRegionsGraphic;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.tools.thread.Activator;

public class RDXPlanarRegionFilteredMapDemo
{
   private final RDXPlanarRegionsGraphic graphic = new RDXPlanarRegionsGraphic();

   private Activator nativesLoadedActivator;
   private PlanarRegionMapHandler mapHandler;

   private final RDXBaseUI baseUI = new RDXBaseUI(getClass(), "ihmc-open-robotics-software", "ihmc-high-level-behaviors/src/test/resources");

   private final ROS2Node ros2Node;
   private final ROS2Helper ros2Helper;

   public RDXPlanarRegionFilteredMapDemo()
   {


      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            mapHandler = new PlanarRegionMapHandler();

            graphic.generateMeshes(mapHandler.getMapRegions());
            graphic.update();

            baseUI.getPrimaryScene().addRenderableProvider(graphic);
         }

         @Override
         public void render()
         {
            if (mapHandler.getFilteredMap().isModified())
            {
               graphic.clear();
               graphic.generateMeshes(mapHandler.getMapRegions());
               graphic.update();
               mapHandler.setModified(false);
            }

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            baseUI.dispose();
            graphic.destroy();
            super.dispose();
         }
      });
   }

   public static void main(String[] args)
   {
      new RDXPlanarRegionFilteredMapDemo();
   }
}
