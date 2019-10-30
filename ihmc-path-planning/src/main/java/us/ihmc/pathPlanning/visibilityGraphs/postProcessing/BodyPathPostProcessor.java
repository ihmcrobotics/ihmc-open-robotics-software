package us.ihmc.pathPlanning.visibilityGraphs.postProcessing;

import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.VisibilityGraphNode;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.VisibilityMapSolution;

import java.util.List;

public interface BodyPathPostProcessor
{
   List<Point3DReadOnly> computePathFromNodes(List<VisibilityGraphNode> nodePath, VisibilityMapSolution visibilityMapSolution);
}
