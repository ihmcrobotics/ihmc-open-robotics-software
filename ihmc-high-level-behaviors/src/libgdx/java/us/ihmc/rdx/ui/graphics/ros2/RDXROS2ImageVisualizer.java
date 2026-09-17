package us.ihmc.rdx.ui.graphics.ros2;

import com.badlogic.gdx.graphics.Texture;
import imgui.type.ImBoolean;
import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.jros2.ROS2Message;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.ui.RDXImagePanel;
import us.ihmc.rdx.ui.graphics.RDXImageVisualizer;

import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Consumer;

public abstract class RDXROS2ImageVisualizer<T extends ROS2Message<T>> extends RDXROS2SingleTopicVisualizer<T>
{
   private final RDXImageVisualizer imageVisualizer;
   private final ImBoolean subscriptionOnly = new ImBoolean();

   private final AtomicReference<Consumer<RDXImageVisualizer>> latestImageUpdate = new AtomicReference<>(null);
   private final RepeatingTaskThread imageUpdateThread = new RepeatingTaskThread(getClass().getSimpleName() + "ImageUpdateThread", this::updateImage);

   private boolean destroyed = false;

   public RDXROS2ImageVisualizer(String title, String panelName, boolean flipY)
   {
      super(title);

      imageVisualizer = new RDXImageVisualizer(title, panelName, flipY);
      imageUpdateThread.startRepeating();
   }

   @Override
   public void renderImGuiWidgets()
   {
      imageVisualizer.renderImGuiWidgets();
   }

   @Override
   public void renderImGuiWidgetsPost()
   {
      ImGuiTools.smallCheckbox(labels.get("Subscription only"), subscriptionOnly);
   }

   @Override
   public void update()
   {
      super.update();
      imageVisualizer.update();

      RDXPanel panel = imageVisualizer.getPanel();
      if (panel != null)
         panel.getIsShowing().set(isActive() && !subscriptionOnly.get());
   }

   @Override
   public void setActive(boolean active)
   {
      super.setActive(active);
      imageVisualizer.setActive(active);
   }

   @Override
   public RDXImagePanel getPanel()
   {
      return imageVisualizer.getPanel();
   }

   public void submitImageUpdate(Consumer<RDXImageVisualizer> imageUpdate)
   {
      synchronized (latestImageUpdate)
      {
         latestImageUpdate.set(imageUpdate);
         latestImageUpdate.notifyAll();
      }
   }

   public ImBoolean getSubscriptionOnly()
   {
      return subscriptionOnly;
   }

   public Texture getTexture()
   {
      return imageVisualizer.getTexture();
   }

   @Override
   public void destroy()
   {
      super.destroy();

      destroyed = true;
      imageUpdateThread.kill();
      imageUpdateThread.interrupt();
      imageVisualizer.destroy();
   }

   private void updateImage()
   {
      try
      {
         // Wait until a new image update is available
         Consumer<RDXImageVisualizer> imageUpdate = null;
         while (!destroyed && (imageUpdate = latestImageUpdate.getAndSet(null)) == null)
         {
            synchronized (latestImageUpdate)
            {
               latestImageUpdate.wait();
            }
         }

         // Run the update
         if (imageUpdate != null)
            imageUpdate.accept(imageVisualizer);
      }
      catch (InterruptedException ignored) {}
   }
}
