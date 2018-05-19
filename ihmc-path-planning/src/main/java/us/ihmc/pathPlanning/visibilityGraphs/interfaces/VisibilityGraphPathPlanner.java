package us.ihmc.pathPlanning.visibilityGraphs.interfaces;

import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.ConnectionPoint3D;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.VisibilityMapHolder;

import java.util.Collection;
import java.util.List;

public interface VisibilityGraphPathPlanner
{
   List<Point3DReadOnly> calculatePath(ConnectionPoint3D start, ConnectionPoint3D goal, Collection<VisibilityMapHolder> visibilityMapHolders);
}
