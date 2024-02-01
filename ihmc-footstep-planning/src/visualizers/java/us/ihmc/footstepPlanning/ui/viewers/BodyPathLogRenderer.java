package us.ihmc.footstepPlanning.ui.viewers;

import com.google.common.util.concurrent.AtomicDouble;
import javafx.animation.AnimationTimer;
import javafx.scene.Group;
import javafx.scene.paint.Color;
import javafx.scene.paint.Material;
import javafx.scene.shape.Mesh;
import javafx.scene.shape.MeshView;
import org.apache.commons.lang3.tuple.Pair;
import org.apache.commons.lang3.tuple.Triple;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.footstepPlanning.bodyPath.BodyPathLatticePoint;
import us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.footstepPlanning.log.AStarBodyPathEdgeData;
import us.ihmc.footstepPlanning.log.AStarBodyPathIterationData;
import us.ihmc.footstepPlanning.log.VariableDescriptor;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorAdaptivePalette;
import us.ihmc.javafx.MeshHolder;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.pathPlanning.graph.structure.GraphEdge;
import us.ihmc.tools.thread.ExecutorServiceTools;

import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.atomic.AtomicReference;

public class BodyPathLogRenderer extends AnimationTimer
{
   private static final Color startNodeColor = Color.color(0.1, 0.1, 0.1, 0.9);
   private static final Color candidateNodeColor = Color.color(0.9, 0.9, 0.9, 0.8);

   private final Group root = new Group();
   private final JavaFXMultiColorMeshBuilder meshBuilder = new JavaFXMultiColorMeshBuilder(new TextureColorAdaptivePalette());

   private final MeshView startNodeGraphic = new MeshView();
   private final MeshView candidateNodeGraphic = new MeshView();

   private final AtomicReference<Pair<BodyPathLatticePoint, Double>> startNodeToVisualize;
   private final AtomicReference<Pair<BodyPathLatticePoint, Double>> candidateNodeToVisualize;
   private final AtomicReference<MeshView> fullPlanMesh = new AtomicReference<>(null);
   private final AtomicReference<Boolean> showBodyPathPlanData;

   private AtomicDouble latestStanceHeight = new AtomicDouble();

   private final ExecutorService executorService = ExecutorServiceTools.newSingleThreadScheduledExecutor(getClass(), ExecutorServiceTools.ExceptionHandling.CANCEL_AND_REPORT);

   public BodyPathLogRenderer(Messager messager)
   {
      messager.addTopicListener(FootstepPlannerMessagerAPI.BodyPathStartNodeToVisualize, d -> latestStanceHeight.set(d.getRight()));
      startNodeToVisualize = messager.createInput(FootstepPlannerMessagerAPI.BodyPathStartNodeToVisualize);
      candidateNodeToVisualize = messager.createInput(FootstepPlannerMessagerAPI.BodyPathCandidateNodeToVisualize);
      showBodyPathPlanData = messager.createInput(FootstepPlannerMessagerAPI.ShowBodyPathPlanData, false);

      messager.addTopicListener(FootstepPlannerMessagerAPI.ShowBodyPathLogGraphics, root::setVisible);
      root.setVisible(false);

      messager.addTopicListener(FootstepPlannerMessagerAPI.BodyPathGraphData, (data) -> executorService.execute(() -> this.buildFullPlanMesh(data)));
   }

   @Override
   public void handle(long now)
   {
      try
      {
         handleInternal();
      }
      catch (Exception e)
      {
         // don't hard crash if RuntimeException is thrown
         e.printStackTrace();
      }
   }

   private void handleInternal()
   {
      root.getChildren().clear();
      root.getChildren().add(startNodeGraphic);
      root.getChildren().add(candidateNodeGraphic);

      Pair<BodyPathLatticePoint, Double> startData = startNodeToVisualize.getAndSet(null);
      if (startData != null)
      {
         generateMesh(startData, startNodeGraphic, startNodeColor);
      }

      Pair<BodyPathLatticePoint, Double> candidateData = candidateNodeToVisualize.getAndSet(null);
      if (candidateData != null)
      {
         generateMesh(candidateData, candidateNodeGraphic, candidateNodeColor);
      }

      if (showBodyPathPlanData.get() && fullPlanMesh.get() != null)
         root.getChildren().add(fullPlanMesh.get());
   }

   public Group getRoot()
   {
      return root;
   }

   private void generateMesh(Pair<BodyPathLatticePoint, Double> data, MeshView meshView, Color color)
   {
      meshBuilder.clear();
      BodyPathLatticePoint node = data.getLeft();
      double height = data.getRight();

      if (Double.isNaN(height))
      {
         height = latestStanceHeight.get() + 0.1;
      }

      double bodyWidth = 0.05;
      meshBuilder.addBox(bodyWidth, bodyWidth, bodyWidth, new Point3D(node.getX(), node.getY(), height), color);

      meshView.setMesh(meshBuilder.generateMesh());
      meshView.setMaterial(meshBuilder.generateMaterial());
   }


   private void buildFullPlanMesh(Triple<Map<GraphEdge<BodyPathLatticePoint>, AStarBodyPathEdgeData>, List<AStarBodyPathIterationData>, List<VariableDescriptor>> graphData)
   {
      Map<GraphEdge<BodyPathLatticePoint> , AStarBodyPathEdgeData> edgeDataMap = graphData.getLeft();
      List<AStarBodyPathIterationData> iterationDataList = graphData.getMiddle();
      List<VariableDescriptor> variableDescriptors = graphData.getRight();

      int costIndex = -1;
      for (int i = 0; i < variableDescriptors.size(); i++)
      {
         if (variableDescriptors.get(i).getName().equals("edgeCost"))
         {
            costIndex = i;
            break;
         }
      }
      buildFullPlanMesh(costIndex, iterationDataList, edgeDataMap);
   }

   private void buildFullPlanMesh(int costIndex, List<AStarBodyPathIterationData> iterationDataList, Map<GraphEdge<BodyPathLatticePoint>, AStarBodyPathEdgeData> edgeDataMap)
   {
      if (iterationDataList.size() < 1)
         return;

      LogTools.info("Generating the full plan mesh");

      JavaFXMultiColorMeshBuilder meshBuilder = new JavaFXMultiColorMeshBuilder(new TextureColorAdaptivePalette());

      recursivelyBuildSolutionMesh(meshBuilder, iterationDataList.get(0), iterationDataList, edgeDataMap);
      buildFullDataMap(costIndex, meshBuilder, iterationDataList, edgeDataMap);

      MeshView mesh = new MeshView(meshBuilder.generateMesh());
      mesh.setMaterial(meshBuilder.generateMaterial());
      LogTools.info("Generated the full plan mesh");
      fullPlanMesh.set(mesh);
   }

   private static void recursivelyBuildSolutionMesh(JavaFXMultiColorMeshBuilder meshBuilder,
                                                    AStarBodyPathIterationData iterationData,
                                                    List<AStarBodyPathIterationData> iterationDataList,
                                                    Map<GraphEdge<BodyPathLatticePoint>, AStarBodyPathEdgeData> edgeDataMap)
   {
      BodyPathLatticePoint stanceNode = iterationData.getParentNode();

      for (int i = 0; i < iterationData.getChildNodes().size(); i++)
      {
         BodyPathLatticePoint childNode = iterationData.getChildNodes().get(i);
         AStarBodyPathEdgeData edgeData = edgeDataMap.get(new GraphEdge<>(stanceNode, childNode));
         Point3D stancePoint = new Point3D(stanceNode.getX(), stanceNode.getY(), iterationData.getParentNodeHeight());
         Point3D childPoint = new Point3D(childNode.getX(), childNode.getY(), edgeData.getChildSnapHeight());
         if (edgeData.isSolutionEdge())
         {
            meshBuilder.addLine(stancePoint, childPoint, 0.01, candidateNodeColor);

            Optional<AStarBodyPathIterationData> childIteration = iterationDataList.stream().filter(data -> data.getParentNode().equals(childNode)).findAny();
            childIteration.ifPresent(data -> recursivelyBuildSolutionMesh(meshBuilder, data, iterationDataList, edgeDataMap));

         }
      }
   }

   private static void buildFullDataMap(int costIndex,
                                        JavaFXMultiColorMeshBuilder meshBuilder,
                                        List<AStarBodyPathIterationData> iterationDataList,
                                        Map<GraphEdge<BodyPathLatticePoint>, AStarBodyPathEdgeData> edgeDataMap)
   {
      for (GraphEdge<BodyPathLatticePoint> edge : edgeDataMap.keySet())
      {
         AStarBodyPathEdgeData edgeData = edgeDataMap.get(edge);
         BodyPathLatticePoint stanceNode = edge.getStartNode();
         BodyPathLatticePoint childNode = edge.getEndNode();

         Optional<AStarBodyPathIterationData> stanceIteration = iterationDataList.stream().filter(data -> data.getParentNode().equals(stanceNode)).findAny();
         double parentHeight = stanceIteration.get().getParentNodeHeight();

         Point3D stancePoint = new Point3D(stanceNode.getX(), stanceNode.getY(), parentHeight);
         Point3D childPoint = new Point3D(childNode.getX(), childNode.getY(), edgeData.getChildSnapHeight());

         if (!edgeData.isSolutionEdge())
         {
            double cost = Double.longBitsToDouble(edgeData.getDataBuffer()[costIndex]);
            if (Double.isFinite(cost))
               meshBuilder.addLine(stancePoint, childPoint, 0.01, Color.GREEN);
            else
               meshBuilder.addLine(stancePoint, childPoint, 0.01, Color.RED);
         }
      }
   }
}
