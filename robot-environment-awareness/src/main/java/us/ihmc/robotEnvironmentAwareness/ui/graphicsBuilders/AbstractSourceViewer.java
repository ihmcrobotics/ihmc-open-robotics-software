package us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders;

import java.util.concurrent.atomic.AtomicReference;

import javafx.collections.ObservableList;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.shape.MeshView;
import us.ihmc.communication.packets.Packet;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorAdaptivePalette;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.communication.REAUIMessager;

public abstract class AbstractSourceViewer<T extends Packet<?>> implements SourceViewerInterface<T>
{
   protected static final float SCAN_POINT_SIZE = 0.0075f;

   private final Group root = new Group();
   protected final ObservableList<Node> children = root.getChildren();

   protected final JavaFXMultiColorMeshBuilder meshBuilder;

   protected final AtomicReference<T> newMessageToRender;
   protected final AtomicReference<MeshView> scanMeshToRender = new AtomicReference<>(null);

   protected final AtomicReference<Boolean> enable;
   protected final AtomicReference<Boolean> clear;

   public AbstractSourceViewer(Topic<T> messageState, REAUIMessager uiMessager, int palleteSizeForMeshBuilder)
   {
      newMessageToRender = uiMessager.createInput(messageState);
      meshBuilder = new JavaFXMultiColorMeshBuilder(new TextureColorAdaptivePalette(palleteSizeForMeshBuilder));

      enable = uiMessager.createInput(REAModuleAPI.UILidarScanShow, false);
      clear = uiMessager.createInput(REAModuleAPI.UILidarScanClear, false);
   }

   @Override
   public void run()
   {
      if (!enable.get())
         return;

      if(newMessageToRender.get() == null)
         return;
      
      unpackPointCloud(newMessageToRender.getAndSet(null));
   }

   @Override
   public void clear()
   {
      children.clear();
   }

   @Override
   public Node getRoot()
   {
      return root;
   }
}
