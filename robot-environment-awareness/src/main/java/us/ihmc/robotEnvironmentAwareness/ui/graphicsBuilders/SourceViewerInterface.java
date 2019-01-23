package us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders;

import javafx.scene.Node;
import us.ihmc.communication.packets.Packet;

public interface SourceViewerInterface<T extends Packet<?>> extends Runnable
{
   abstract void render();

   abstract void clear();

   abstract Node getRoot();

   abstract void unpackPointCloud(T message);
}
