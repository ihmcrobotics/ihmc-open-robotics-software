package us.ihmc.simulationconstructionset.util.visualizers;

import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.graphics3DAdapter.structure.Graphics3DNode;
import us.ihmc.graphics3DAdapter.structure.Graphics3DNodeType;
import us.ihmc.robotics.dataStructures.listener.RewoundListener;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

import javax.vecmath.Point3d;
import java.util.ArrayList;
import java.util.List;

public class SCSPointCloudVisualizer implements RewoundListener
{
   private final double VOXEL_SIZE = 0.01;
   private final ArrayList<Graphics3DNode> nodes = new ArrayList<>();

   private final SimulationConstructionSet scs;

   private boolean disableGraphics = false;

   public SCSPointCloudVisualizer(SimulationConstructionSet scs)
   {
      this.scs = scs;

   }

   public void addPoints(List<Point3d> points)
   {
      if(!disableGraphics)
      {
         for(Point3d point : points)
         {
            Graphics3DObject testCubeGraphicObject = new Graphics3DObject();
            testCubeGraphicObject.addCube(VOXEL_SIZE, VOXEL_SIZE, VOXEL_SIZE, YoAppearance.Aqua());
            Graphics3DNode cubeNode = scs.addStaticLinkGraphics(testCubeGraphicObject, Graphics3DNodeType.VISUALIZATION);
            cubeNode.translate(point.getX(), point.getY(), point.getZ());

            nodes.add(cubeNode);
         }
      }
   }

   public void clear()
   {
      for(Graphics3DNode node : nodes)
      {
         scs.removeGraphics3dNode(node);
      }

      nodes.clear();
   }

   @Override
   public void wasRewound()
   {
      disableGraphics = true;
      clear();
   }
}
