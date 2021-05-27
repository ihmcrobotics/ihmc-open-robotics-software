package us.ihmc.gdx.ui.behaviors.registry;

import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.vr.GDXVRManager;

public abstract class GDXBehaviorUIInterface implements RenderableProvider
{
   protected GDXBehaviorUIInterface()
   {
   }

   public abstract void create(GDXImGuiBasedUI baseUI);

//   public abstract void setEnabled(boolean enabled);

   public void handleVREvents(GDXVRManager vrManager)
   {

   }

   public abstract void render();

   public abstract void destroy();

//   protected void enable3DVisualizations()
//   {
//   }
//
//   protected ROS2NodeInterface getRos2Node()
//   {
//      return ros2Node;
//   }
//
//   protected Messager getBehaviorMessager()
//   {
//      return behaviorMessager;
//   }
//
//   protected DRCRobotModel getRobotModel()
//   {
//      return robotModel;
//   }
}
