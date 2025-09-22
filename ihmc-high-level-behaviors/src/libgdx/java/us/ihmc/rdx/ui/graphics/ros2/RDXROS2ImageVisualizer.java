package us.ihmc.rdx.ui.graphics.ros2;

import imgui.type.ImBoolean;
import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.ui.RDXImagePanel;
import us.ihmc.rdx.ui.graphics.RDXImageVisualizer;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Consumer;

public abstract class RDXROS2ImageVisualizer<T> extends RDXROS2SingleTopicVisualizer<T>
{
   private final RDXImageVisualizer imageVisualizer;
   private final ImBoolean subscriptionOnly = new ImBoolean();

   private final AtomicReference<Consumer<RDXImageVisualizer>> latestImageUpdate = new AtomicReference<>(null);
   private final List<Runnable> onImageUpdateRunnables = new ArrayList<>();
   private final RepeatingTaskThread imageUpdateThread = new RepeatingTaskThread("ImageUpdateThread", this::updateImage);

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
      if (ImGuiTools.smallCheckbox(labels.get("Subscription only"), subscriptionOnly))
      {
         RDXPanel panel = getPanel();
         if (panel != null)
            panel.getIsShowing().set(!subscriptionOnly.get());
      }
   }

   @Override
   public void update()
   {
      super.update();
      imageVisualizer.update();
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

   public void onImageUpdate(Runnable runnable)
   {
      onImageUpdateRunnables.add(runnable);
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
         {
            imageUpdate.accept(imageVisualizer);

            for (Runnable runnable : onImageUpdateRunnables)
               runnable.run();
         }
      }
      catch (InterruptedException ignored) {}
   }
}
