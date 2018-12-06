package us.ihmc.robotEnvironmentAwareness.polygonizer;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;
import java.util.stream.Collectors;

import javafx.animation.AnimationTimer;
import javafx.scene.Group;
import javafx.scene.Node;
import us.ihmc.robotEnvironmentAwareness.polygonizer.ConcaveHullViewer.ConcaveHullViewerInput;
import us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders.OcTreeMeshBuilder;

public class MultipleConcaveHullViewer extends AnimationTimer
{
   private final Group rootNode = new Group();

   private final List<ConcaveHullViewer> concaveHullViewerRendered = new ArrayList<>();
   private AtomicReference<List<ConcaveHullViewer>> concaveHullViewerToRender = new AtomicReference<>(null);

   public MultipleConcaveHullViewer()
   {
   }

   @Override
   public void handle(long now)
   {
      List<ConcaveHullViewer> newViewers = concaveHullViewerToRender.getAndSet(null);

      if (newViewers != null)
      {
         rootNode.getChildren().clear();
         concaveHullViewerRendered.clear();
         concaveHullViewerRendered.addAll(newViewers);
         concaveHullViewerRendered.forEach(viewer -> rootNode.getChildren().add(viewer.getRootNode()));
      }

      concaveHullViewerRendered.forEach(viewer -> viewer.handle(now));
   }

   public void submit(Collection<ConcaveHullViewerInput> inputs)
   {
      List<ConcaveHullViewer> newViewers = new ArrayList<>();

      for (ConcaveHullViewerInput input : inputs)
      {
         ConcaveHullViewer concaveHullViewer = new ConcaveHullViewer();
         concaveHullViewer.submit(input);
         newViewers.add(concaveHullViewer);
      }

      concaveHullViewerToRender.set(newViewers);
   }

   public Node getRootNode()
   {
      return rootNode;
   }

   public static List<ConcaveHullViewerInput> toConcaveHullViewerInputs(Collection<Polygonizer.Output> polygonizerOutputs)
   {
      return polygonizerOutputs.stream().map(MultipleConcaveHullViewer::toConcaveHullViewerInput).collect(Collectors.toList());
   }

   public static ConcaveHullViewerInput toConcaveHullViewerInput(Polygonizer.Output polygonizerOutput)
   {
      return new ConcaveHullViewerInput(OcTreeMeshBuilder.getRegionColor(polygonizerOutput.getInput().getId()),
                                        polygonizerOutput.getInput().getTransformToWorld(), polygonizerOutput.getConcaveHullFactoryResult());
   }
}
