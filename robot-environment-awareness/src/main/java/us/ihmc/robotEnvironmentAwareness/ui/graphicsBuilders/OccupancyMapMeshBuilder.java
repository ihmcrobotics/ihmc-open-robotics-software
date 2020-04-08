package us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders;

import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.atomic.AtomicReference;

import com.google.common.util.concurrent.AtomicDouble;

import javafx.collections.ObservableList;
import javafx.scene.Group;
import javafx.scene.Node;
import us.ihmc.jOctoMap.key.OcTreeKey;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorPalette1D;
import us.ihmc.robotEnvironmentAwareness.communication.REAUIMessager;
import us.ihmc.robotEnvironmentAwareness.communication.SLAMModuleAPI;
import us.ihmc.robotEnvironmentAwareness.communication.packets.NormalOcTreeMessage;
import us.ihmc.robotEnvironmentAwareness.ui.UIOcTree;

public class OccupancyMapMeshBuilder implements Runnable
{
   /**
    * size ratio to the octree resolution.
    */
   private static final double DEFAULT_CELL_SIZE_RATIO = 0.7;
   private AtomicDouble cellSize = new AtomicDouble(0.0);
   
   private final Group root = new Group();
   private final ObservableList<Node> children = root.getChildren();

   private final JavaFXMultiColorMeshBuilder meshBuilder;

   private final TextureColorPalette1D normalBasedColorPalette1D = new TextureColorPalette1D();

   private final AtomicReference<NormalOcTreeMessage> ocTreeState;
   
   private final AtomicReference<Boolean> occupancyEnable;
   private final AtomicReference<Boolean> normalVectorEnable;
   private final AtomicReference<Boolean> clear;
   
   private final REAUIMessager uiMessager;
   private final AtomicReference<UIOcTree> uiOcTree = new AtomicReference<UIOcTree>(null);
   
   private final Map<OcTreeKey, Double> nodeKeyToConfidenceFactorMap = new HashMap<>();
   
   public OccupancyMapMeshBuilder(REAUIMessager uiMessager)
   {
      this.uiMessager = uiMessager;
      
      ocTreeState = uiMessager.createInput(SLAMModuleAPI.SLAMOctreeMapState);
      
      occupancyEnable = uiMessager.createInput(SLAMModuleAPI.ShowSLAMOctreeMap, true);
      normalVectorEnable = uiMessager.createInput(SLAMModuleAPI.ShowSLAMOctreeNormalMap, true);
      clear = uiMessager.createInput(SLAMModuleAPI.SLAMClear);
      
      meshBuilder = new JavaFXMultiColorMeshBuilder(normalBasedColorPalette1D);
   }
   
   public void render()
   {
      
   }
   
   @Override
   public void run()
   {
      
   }

   public Node getRoot()
   {
      return root;
   }
   
   
}
