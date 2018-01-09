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
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;
import java.util.stream.Collectors;

import javafx.beans.property.Property;
import javafx.beans.value.ObservableValue;
import javafx.collections.ObservableList;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.paint.Color;
import javafx.scene.paint.Material;
import javafx.scene.shape.Mesh;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.Vector3D32;
import us.ihmc.graphicsDescription.MeshDataHolder;
import us.ihmc.graphicsDescription.TexCoord2f;
import us.ihmc.jOctoMap.key.OcTreeKey;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorPalette1D;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.communication.REAUIMessager;
import us.ihmc.robotEnvironmentAwareness.communication.packets.NormalOcTreeMessage;
import us.ihmc.robotEnvironmentAwareness.communication.packets.PlanarRegionSegmentationMessage;
import us.ihmc.robotEnvironmentAwareness.geometry.IntersectionPlaneBoxCalculator;
import us.ihmc.robotEnvironmentAwareness.ui.UIOcTree;
import us.ihmc.robotEnvironmentAwareness.ui.UIOcTreeNode;
import us.ihmc.robotics.lists.GenericTypeBuilder;
import us.ihmc.robotics.lists.RecyclingArrayList;

/**
 * Created by adrien on 11/21/16.
 */
public class OcTreeMeshBuilder implements Runnable
{
   private static final int FX_NODE_DEPTH = 8;
   private static final Color DEFAULT_COLOR = Color.DARKCYAN;

   public enum ColoringType
   {
      DEFAULT, NORMAL, HAS_CENTER, REGION
   }

   public enum DisplayType
   {
      HIDE, CELL, PLANE, HIT_LOCATION
   }

   private final AtomicReference<Boolean> enable;
   private final AtomicReference<Boolean> clear;

   private final Property<ColoringType> coloringType;
   private final Property<DisplayType> displayType;
   private final Property<Integer> treeDepthForDisplay;
   private final Property<Boolean> hidePlanarRegionNodes;

   private final Group root = new Group();
   private final ObservableList<Node> children = root.getChildren();

   private final JavaFXMultiColorMeshBuilder meshBuilder;

   private final TextureColorPalette1D normalBasedColorPalette1D = new TextureColorPalette1D();

   private final RecyclingArrayList<Point3D> plane = new RecyclingArrayList<>(GenericTypeBuilder.createBuilderWithEmptyConstructor(Point3D.class));
   private final IntersectionPlaneBoxCalculator intersectionPlaneBoxCalculator = new IntersectionPlaneBoxCalculator();

   private final AtomicReference<NormalOcTreeMessage> ocTreeState;
   private final AtomicReference<PlanarRegionSegmentationMessage[]> planarRegionSegmentationState;

   private final AtomicBoolean processChange = new AtomicBoolean(false);

   private final AtomicReference<Set<UIOcTreeNodeMeshView>> newSubOcTreeMeshViews = new AtomicReference<>(null);
   private final Deque<UIOcTreeNodeMeshView> meshViewsBeingProcessed = new ArrayDeque<>();

   private final REAUIMessager uiMessager;
   private final AtomicReference<UIOcTree> uiOcTree = new AtomicReference<UIOcTree>(null);

   public OcTreeMeshBuilder(REAUIMessager uiMessager)
   {
      this.uiMessager = uiMessager;
      enable = uiMessager.createInput(REAModuleAPI.OcTreeEnable, false);
      clear = uiMessager.createInput(REAModuleAPI.OcTreeClear, false);

      treeDepthForDisplay = uiMessager.createPropertyInput(REAModuleAPI.UIOcTreeDepth, Integer.MAX_VALUE);
      treeDepthForDisplay.addListener(this::setProcessChange);
      coloringType = uiMessager.createPropertyInput(REAModuleAPI.UIOcTreeColoringMode, ColoringType.DEFAULT);
      coloringType.addListener(this::setProcessChange);

      displayType = uiMessager.createPropertyInput(REAModuleAPI.UIOcTreeDisplayType, DisplayType.PLANE);
      displayType.addListener(this::setProcessChange);
      hidePlanarRegionNodes = uiMessager.createPropertyInput(REAModuleAPI.UIPlanarRegionHideNodes, false);
      hidePlanarRegionNodes.addListener(this::setProcessChange);

      ocTreeState = uiMessager.createInput(REAModuleAPI.OcTreeState);
      planarRegionSegmentationState = uiMessager.createInput(REAModuleAPI.PlanarRegionsSegmentationState);

      normalBasedColorPalette1D.setHueBased(0.9, 0.8);
      meshBuilder = new JavaFXMultiColorMeshBuilder(normalBasedColorPalette1D);
   }

   private <T> void setProcessChange(ObservableValue<? extends T> observableValue, T oldValue, T newValue)
   {
      try
      {
         processChange.set(!oldValue.equals(newValue));
      }
      catch (NullPointerException e)
      {
         processChange.set(oldValue != newValue);
      }
   }

   public void render()
   {
      if (clear.getAndSet(false))
      {
         children.clear();
         newSubOcTreeMeshViews.set(null);
         meshViewsBeingProcessed.clear();
         return;
      }

      Set<UIOcTreeNodeMeshView> newMeshViews = newSubOcTreeMeshViews.getAndSet(null);

      if (newMeshViews != null)
      {
         List<Node> newChildren = children.stream()
                                          .filter(newMeshViews::contains)
                                          .collect(Collectors.toList());

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
         PrintTools.warn("Rendering job is not done, waiting before creating new meshes.");
         return;
      }

      if (displayType.getValue() == DisplayType.HIDE)
         return;

      if (enable.get())
      {
         uiMessager.submitStateRequestToModule(REAModuleAPI.RequestOctree);
         uiMessager.submitStateRequestToModule(REAModuleAPI.RequestPlanarRegionSegmentation);

         NormalOcTreeMessage newMessage = ocTreeState.get();
         Map<OcTreeKey, Integer> nodeKeyToRegionIdMap = createNodeKeyToRegionIdMap(planarRegionSegmentationState.getAndSet(null));

         if (newMessage == null || nodeKeyToRegionIdMap == null)
            return;

         uiOcTree.set(new UIOcTree(ocTreeState.getAndSet(null), nodeKeyToRegionIdMap));
         buildUIOcTreeMesh(uiOcTree.get());
      }
      else if (processChange.getAndSet(false) && uiOcTree.get() != null)
      {
         buildUIOcTreeMesh(uiOcTree.get());
      }
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

      Iterable<UIOcTreeNode> iterable = createLeafIterable(subTreeRoot, treeDepthForDisplay.getValue());

      for (UIOcTreeNode node : iterable)
      {
         if (!node.isPartOfRegion() || !hidePlanarRegionNodes.getValue())
            addNodeMesh(meshBuilder, displayType.getValue(), coloringType.getValue(), node);
      }

      OcTreeKey rootKey = subTreeRoot.getKeyCopy();
      Mesh mesh = meshBuilder.generateMesh();
      Material material = meshBuilder.generateMaterial();
      UIOcTreeNodeMeshView meshView = new UIOcTreeNodeMeshView(rootKey, mesh, material);
      meshBuilder.clear();
      return meshView;
   }

   private void addNodeMesh(JavaFXMultiColorMeshBuilder meshBuilder, DisplayType displayType, ColoringType coloringType, UIOcTreeNode node)
   {
      Color color = getNodeColor(coloringType, node);
      double size = node.getSize();

      switch (displayType)
      {
      case CELL:
         meshBuilder.addCube(size, node.getX(), node.getY(), node.getZ(), color);
         break;
      case PLANE:
         if (node.isNormalSet())
            meshBuilder.addMesh(createNormalBasedPlane(node), color);
         break;
      case HIT_LOCATION:
         if (node.isHitLocationSet())
         {
            Point3D hitLocation = new Point3D();
            node.getHitLocation(hitLocation);
            meshBuilder.addTetrahedron(0.0075, hitLocation, color);
         }
         break;
      default:
         throw new RuntimeException("Unexpected value for display type: " + displayType);
      }      
   }

   private Color getNodeColor(ColoringType coloringType, UIOcTreeNode node)
   {
      switch (coloringType)
      {
      case REGION:
         if (node.isPartOfRegion())
         {
            return getRegionColor(node.getRegionId());
         }
         else
         {
            return DEFAULT_COLOR;
         }
      case HAS_CENTER:
         return node.isHitLocationSet() ? Color.DARKGREEN : Color.RED;
      case NORMAL:
         if (node.isNormalSet())
         {
            Vector3D normal = new Vector3D();
            node.getNormal(normal);
            Vector3D zUp = new Vector3D(0.0, 0.0, 1.0);
            normal.normalize();
            double angle = Math.abs(zUp.dot(normal));
            double hue = 120.0 * angle;
            return Color.hsb(hue, 1.0, 1.0);
         }
         else
            return DEFAULT_COLOR;
      case DEFAULT:
      default:
         return DEFAULT_COLOR;
      }
   }

   public static Color getRegionColor(int regionId)
   {
      java.awt.Color awtColor = new java.awt.Color(regionId);
      return Color.rgb(awtColor.getRed(), awtColor.getGreen(), awtColor.getBlue());
   }

   private MeshDataHolder createNormalBasedPlane(UIOcTreeNode node)
   {
      if (!node.isNormalSet() || !node.isHitLocationSet())
         return null;

      Vector3D planeNormal = new Vector3D();
      Point3D pointOnPlane = new Point3D();
      double size = node.getSize();

      node.getNormal(planeNormal);
      node.getHitLocation(pointOnPlane);

      intersectionPlaneBoxCalculator.setCube(size, node.getX(), node.getY(), node.getZ());
      intersectionPlaneBoxCalculator.setPlane(pointOnPlane, planeNormal);
      intersectionPlaneBoxCalculator.computeIntersections(plane);

      if (plane.size() < 3)
         return null;

      int numberOfTriangles = plane.size() - 2;
      int[] triangleIndices = new int[3 * numberOfTriangles];
      int index = 0;
      for (int j = 2; j < plane.size(); j++)
      {
         triangleIndices[index++] = 0;
         triangleIndices[index++] = j - 1;
         triangleIndices[index++] = j;
      }

      Point3D32[] vertices = new Point3D32[plane.size()];
      TexCoord2f[] texCoords = new TexCoord2f[plane.size()];
      Vector3D32[] normals = new Vector3D32[plane.size()];

      for (int i = 0; i < plane.size(); i++)
      {
         vertices[i] = new Point3D32(plane.get(i));
         texCoords[i] = new TexCoord2f(); // No need for real coordinates, the MultiColorMeshBuilder creates new ones.
         normals[i] = new Vector3D32(planeNormal);
      }

      return new MeshDataHolder(vertices, texCoords, triangleIndices, normals);
   }

   private Map<OcTreeKey, Integer> createNodeKeyToRegionIdMap(PlanarRegionSegmentationMessage[] planarRegionSegmentationMessages)
   {
      if (planarRegionSegmentationMessages == null)
         return null;

      Map<OcTreeKey, Integer> nodeKeyToRegionIdMap = new HashMap<>();

      for (PlanarRegionSegmentationMessage planarRegionSegmentationMessage : planarRegionSegmentationMessages)
         registerNodeKeysIntoMap(nodeKeyToRegionIdMap, planarRegionSegmentationMessage);

      return nodeKeyToRegionIdMap;
   }

   private void registerNodeKeysIntoMap(Map<OcTreeKey, Integer> nodeKeyToRegionIdMap, PlanarRegionSegmentationMessage planarRegionSegmentationMessage)
   {
      for (int i = 0; i < planarRegionSegmentationMessage.getNumberOfNodes(); i++)
      {
         OcTreeKey nodeKey = new OcTreeKey();
         planarRegionSegmentationMessage.getNodeKey(i, nodeKey);
         nodeKeyToRegionIdMap.put(nodeKey, planarRegionSegmentationMessage.getRegionId());
      }
   }

   public Node getRoot()
   {
      return root;
   }
}
