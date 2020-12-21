package us.ihmc.humanoidBehaviors.ui.gdx;

import com.badlogic.gdx.graphics.g3d.ModelInstance;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.gdx.GDX3DApplication;
import us.ihmc.gdx.GDXApplicationCreator;
import us.ihmc.gdx.GDXModelPrimitives;
import us.ihmc.ros2.ROS2Node;

public class GDXROS2PointCloudViewer extends GDX3DApplication
{
   private final ROS2Node ros2Node;

   public GDXROS2PointCloudViewer(ROS2Node ros2Node)
   {
      this.ros2Node = ros2Node;

      GDXApplicationCreator.launchGDXApplication(new PrivateGDX3DApplication(), "GDX3DDemo", 1100, 800);
   }

   class PrivateGDX3DApplication extends GDX3DApplication
   {
      @Override
      public void create()
      {
         super.create();

         addModelInstance(new ModelInstance(GDXModelPrimitives.createCoordinateFrame(0.3)));
      }
   }

   public static void main(String[] args)
   {
      new GDXROS2PointCloudViewer(ROS2Tools.createInterprocessROS2Node("point_cloud_viewer"));
   }
}
