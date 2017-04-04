package us.ihmc.simulationConstructionSetTools.util.visualizers;

import java.util.List;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.structure.Graphics3DNode;
import us.ihmc.graphicsDescription.structure.Graphics3DNodeType;
import us.ihmc.robotics.dataStructures.listener.RewoundListener;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class SCSPointCloudVisualizer implements RewoundListener
{
   private final double VOXEL_SIZE = 0.1;
   private final SimulationConstructionSet scs;

   private Graphics3DNode rootNode;
   private boolean disableGraphics = false;

   public SCSPointCloudVisualizer(SimulationConstructionSet scs)
   {
      this.scs = scs;
   }

   public void addPoints(List<Point3D> points)
   {
      if(!disableGraphics)
      {
         for(Point3D point : points)
         {
            Graphics3DObject testCubeGraphicObject = new Graphics3DObject();
            testCubeGraphicObject.addCube(VOXEL_SIZE, VOXEL_SIZE, VOXEL_SIZE, YoAppearance.Aqua());
            Graphics3DNode cubeNode = scs.addStaticLinkGraphics(testCubeGraphicObject, Graphics3DNodeType.VISUALIZATION);
            cubeNode.translate(point.getX(), point.getY(), point.getZ());

            if(rootNode == null)
               rootNode = cubeNode;
            else
               rootNode.addChild(cubeNode);
         }
      }
   }

   public void clear()
   {
      scs.removeGraphics3dNode(rootNode);
      rootNode = null;
   }

   @Override
   public void wasRewound()
   {
      disableGraphics = true;
      clear();
   }
}
