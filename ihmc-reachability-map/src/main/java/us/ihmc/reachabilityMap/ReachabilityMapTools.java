package us.ihmc.reachabilityMap;

import java.awt.Color;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.MeshDataGenerator;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;

public class ReachabilityMapTools
{

   public static Graphics3DObject createBoundingBoxGraphics(FramePoint3DReadOnly min, FramePoint3DReadOnly max)
   {
      double width = 0.01;
      AppearanceDefinition appearance = YoAppearance.LightBlue();
      Graphics3DObject boundingBox = new Graphics3DObject();
      FramePoint3D modifiableMin = new FramePoint3D(min);
      modifiableMin.changeFrame(ReferenceFrame.getWorldFrame());
      FramePoint3D modifiableMax = new FramePoint3D(max);
      modifiableMax.changeFrame(ReferenceFrame.getWorldFrame());
      double x0 = modifiableMin.getX();
      double y0 = modifiableMin.getY();
      double z0 = modifiableMin.getZ();
      double x1 = modifiableMax.getX();
      double y1 = modifiableMax.getY();
      double z1 = modifiableMax.getZ();
      // The three segments originating from min
      boundingBox.addMeshData(MeshDataGenerator.Line(x0, y0, z0, x1, y0, z0, width), appearance);
      boundingBox.addMeshData(MeshDataGenerator.Line(x0, y0, z0, x0, y1, z0, width), appearance);
      boundingBox.addMeshData(MeshDataGenerator.Line(x0, y0, z0, x0, y0, z1, width), appearance);
      // The three segments originating from min
      boundingBox.addMeshData(MeshDataGenerator.Line(x1, y1, z1, x0, y1, z1, width), appearance);
      boundingBox.addMeshData(MeshDataGenerator.Line(x1, y1, z1, x1, y0, z1, width), appearance);
      boundingBox.addMeshData(MeshDataGenerator.Line(x1, y1, z1, x1, y1, z0, width), appearance);

      boundingBox.addMeshData(MeshDataGenerator.Line(x1, y0, z0, x1, y1, z0, width), appearance);
      boundingBox.addMeshData(MeshDataGenerator.Line(x1, y0, z0, x1, y0, z1, width), appearance);

      boundingBox.addMeshData(MeshDataGenerator.Line(x0, y1, z0, x1, y1, z0, width), appearance);
      boundingBox.addMeshData(MeshDataGenerator.Line(x0, y1, z0, x0, y1, z1, width), appearance);

      boundingBox.addMeshData(MeshDataGenerator.Line(x0, y0, z1, x1, y0, z1, width), appearance);
      boundingBox.addMeshData(MeshDataGenerator.Line(x0, y0, z1, x0, y1, z1, width), appearance);

      return boundingBox;
   }

   public static Graphics3DObject createReachibilityColorScale()
   {
      Graphics3DObject voxelViz = new Graphics3DObject();
      double maxReachability = 0.7;
      double resolution = 0.1;
      voxelViz.translate(-1.0, -1.0, 0.0);

      for (double z = 0; z <= maxReachability; z += maxReachability * resolution)
      {
         AppearanceDefinition appearance = YoAppearance.RGBColorFromHex(Color.HSBtoRGB((float) z, 1.0f, 1.0f));
         voxelViz.translate(0.0, 0.0, resolution);
         voxelViz.addSphere(0.025, appearance);
      }

      return voxelViz;
   }
}
