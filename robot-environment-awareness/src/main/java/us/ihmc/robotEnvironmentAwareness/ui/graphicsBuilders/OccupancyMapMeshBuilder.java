package us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders;

import static us.ihmc.jOctoMap.iterators.OcTreeIteratorFactory.createLeafIterable;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Deque;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.atomic.AtomicReference;
import java.util.stream.Collectors;

import com.google.common.util.concurrent.AtomicDouble;

import javafx.collections.ObservableList;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.paint.Color;
import javafx.scene.paint.Material;
import javafx.scene.shape.Mesh;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.jOctoMap.key.OcTreeKey;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorPalette1D;
import us.ihmc.log.LogTools;
import us.ihmc.robotEnvironmentAwareness.communication.REAUIMessager;
import us.ihmc.robotEnvironmentAwareness.communication.SLAMModuleAPI;
import us.ihmc.robotEnvironmentAwareness.communication.packets.NormalOcTreeMessage;
import us.ihmc.robotEnvironmentAwareness.ui.UIOcTree;
import us.ihmc.robotEnvironmentAwareness.ui.UIOcTreeNode;

public class OccupancyMapMeshBuilder implements Runnable
{
   /**
    * size ratio to the octree resolution.
    */
   private static final double DEFAULT_CELL_SIZE_RATIO = 0.7;
   private AtomicDouble cellSize = new AtomicDouble(0.0);
   private static final int FX_NODE_DEPTH = 8;

   private final Group root = new Group();
   private final ObservableList<Node> children = root.getChildren();

   private final JavaFXMultiColorMeshBuilder meshBuilder;

   private final TextureColorPalette1D normalBasedColorPalette1D = new TextureColorPalette1D();

   private final AtomicReference<NormalOcTreeMessage> ocTreeState;
   private final AtomicReference<Double> confidenceFactorState;

   private final AtomicReference<Boolean> occupancyEnable;
   private final AtomicReference<Boolean> normalVectorEnable;
   private final AtomicReference<Boolean> clear;

   private final REAUIMessager uiMessager;
   private final AtomicReference<UIOcTree> uiOcTree = new AtomicReference<UIOcTree>(null);
   private final Map<OcTreeKey, Integer> nodeKeyToColorMap = new HashMap<>();

   private final AtomicReference<Set<UIOcTreeNodeMeshView>> newSubOcTreeMeshViews = new AtomicReference<>(null);
   private final Deque<UIOcTreeNodeMeshView> meshViewsBeingProcessed = new ArrayDeque<>();

   private final Map<OcTreeKey, Double> nodeKeyToConfidenceFactorMap = new HashMap<>();

   public OccupancyMapMeshBuilder(REAUIMessager uiMessager)
   {
      this.uiMessager = uiMessager;

      ocTreeState = uiMessager.createInput(SLAMModuleAPI.SLAMOctreeMapState);
      confidenceFactorState = uiMessager.createInput(SLAMModuleAPI.LatestFrameConfidenceFactor, 1.0);

      occupancyEnable = uiMessager.createInput(SLAMModuleAPI.ShowSLAMOctreeMap, true);
      normalVectorEnable = uiMessager.createInput(SLAMModuleAPI.ShowSLAMOctreeNormalMap, true);
      clear = uiMessager.createInput(SLAMModuleAPI.SLAMClear, false);

      meshBuilder = new JavaFXMultiColorMeshBuilder(normalBasedColorPalette1D);
   }

   public void render()
   {
      //TODO: update confidence factor map.
      //TODO: render occupancy with color.
      //TODO: render normal vector.

      if (clear.getAndSet(false))
      {
         children.clear();
         nodeKeyToColorMap.clear();
         newSubOcTreeMeshViews.set(null);
         meshViewsBeingProcessed.clear();
         return;
      }

      Set<UIOcTreeNodeMeshView> newMeshViews = newSubOcTreeMeshViews.getAndSet(null);

      if (newMeshViews != null)
      {
         List<Node> newChildren = children.stream().filter(newMeshViews::contains).collect(Collectors.toList());

         children.clear();
         children.addAll(newChildren);

         meshViewsBeingProcessed.clear();
         meshViewsBeingProcessed.addAll(newMeshViews);
      }

      if (meshViewsBeingProcessed.isEmpty())
         return;

      UIOcTreeNodeMeshView newMeshView = meshViewsBeingProcessed.pop();
      children.remove(newMeshView);
      children.add(newMeshView);
   }

   @Override
   public void run()
   {
      if (newSubOcTreeMeshViews.get() != null)
      {
         LogTools.warn("Rendering job is not done, waiting before creating new meshes.");
         return;
      }

      if (occupancyEnable.get())
      {
         NormalOcTreeMessage newMessage = ocTreeState.get();
         double confidenceFactor = confidenceFactorState.get();

         if (newMessage == null)
            return;

         updateNodeKeyToColorMap(newMessage, confidenceFactor);

         uiOcTree.set(new UIOcTree(ocTreeState.getAndSet(null), nodeKeyToColorMap));
         buildUIOcTreeMesh(uiOcTree.get());
      }
   }

   private void updateNodeKeyToColorMap(NormalOcTreeMessage newMessage, double confidenceFactor)
   {
      //nodeKeyToColorMap
   }

   private void buildUIOcTreeMesh(UIOcTree ocTree)
   {
      Set<UIOcTreeNodeMeshView> meshViews = new HashSet<>();

      List<UIOcTreeNode> rootNodes = new ArrayList<>();
      createLeafIterable(ocTree.getRoot(), FX_NODE_DEPTH).forEach(rootNodes::add);

      for (UIOcTreeNode rootNode : rootNodes)
         meshViews.add(createSubTreeMeshView(rootNode));

      newSubOcTreeMeshViews.set(meshViews);
   }

   private UIOcTreeNodeMeshView createSubTreeMeshView(UIOcTreeNode subTreeRoot)
   {
      meshBuilder.clear();

      Iterable<UIOcTreeNode> iterable = createLeafIterable(subTreeRoot, 16);

      for (UIOcTreeNode node : iterable)
      {
         if (!node.isPartOfRegion())
            addNodeMesh(meshBuilder, node);
      }

      OcTreeKey rootKey = subTreeRoot.getKeyCopy();
      Mesh mesh = meshBuilder.generateMesh();
      Material material = meshBuilder.generateMaterial();
      UIOcTreeNodeMeshView meshView = new UIOcTreeNodeMeshView(rootKey, mesh, material);
      meshBuilder.clear();
      return meshView;
   }

   private final Point3D hitLocation = new Point3D();
   private final Vector3D normalVector = new Vector3D();
   private final Point3D normalVectorEnd = new Point3D();

   private void addNodeMesh(JavaFXMultiColorMeshBuilder meshBuilder, UIOcTreeNode node)
   {
      Color color = OcTreeMeshBuilder.getRegionColor(node.getRegionId());
      double size = 0.02 * DEFAULT_CELL_SIZE_RATIO;
      node.getHitLocation(hitLocation);
      meshBuilder.addCube(size, hitLocation.getX(), hitLocation.getY(), hitLocation.getZ(), color);
      if(normalVectorEnable.get())
      {
         node.getNormal(normalVector);
         normalVectorEnd.set(normalVector);
         normalVectorEnd.scaleAdd(0.02, hitLocation);
         meshBuilder.addLine(hitLocation, normalVectorEnd, 0.002, Color.BEIGE);
      }
   }

   public Node getRoot()
   {
      return root;
   }

}
