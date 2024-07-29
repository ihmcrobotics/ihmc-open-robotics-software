package us.ihmc.rdx.tools;

import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.model.data.*;
import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.Vector3D32;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.referenceFrames.MutableReferenceFrame;

import java.util.ArrayList;

/**
 * This class works with a copy of ModelData that it will mutate to create scaled ModelInstances.
 * Every time scale is called, the modelIntance will be recreated as a new object.
 *
 * The scaling is about the center of mass of the vertices, i.e. the centroid. This is so the model will
 * get bigger but not translate much, so that scaled up models will be in the same place roughly.
 * This is useful when you want a bigger model to enclose a smaller version.
 *
 * TODO: Dynamically scale using the shader or scale in parallel on the GPU somehow.
 *   Usually, the scale is represented as a uniform and sent to the shader to be used for changing scale before rendering.
 *   i.e. how OpenGL changes point cloud point size for spheres/points. It never modifies the actual vertex data on the CPU.
 */
public class RDXModelInstanceScaler
{
   private final ModelData modelData;
   private final Point3D32 wholeModelCentroid;

   private record OriginalVertexRecord(Point3D32 originalVertex, short index) { }
   private record PartRecord(ModelMeshPart modelMeshPart,
                             ModelMesh modelMesh,
                             RigidBodyTransform transform,
                             int floatsPerVertex,
                             int numberOfVertices,
                             ArrayList<OriginalVertexRecord> originalVertices) { }
   private final ArrayList<PartRecord> partRecords = new ArrayList<>();
   private ModelInstance modelInstance;
   private final FramePoint3D scaledVertex = new FramePoint3D();
   private final Vector3D32 centroidToVertex = new Vector3D32();
   private final MutableReferenceFrame centroidFrame = new MutableReferenceFrame(ReferenceFrame.getWorldFrame());
   private final Stopwatch stopwatch = new Stopwatch();

   public RDXModelInstanceScaler(String modelFileName)
   {
      this(RDXModelLoader.loadModelData(modelFileName));
   }

   public RDXModelInstanceScaler(ModelData modelData)
   {
      this.modelData = modelData;

      stopwatch.start();
      wholeModelCentroid = new Point3D32();
      int totalNumberOfVertices = 0;
      for (int nodeIndex = 0; nodeIndex < modelData.nodes.size; nodeIndex++)
      {
         ModelNode node = modelData.nodes.get(nodeIndex);

         if (node.parts != null)
         {
            for (ModelNodePart part : node.parts)
            {
               String meshPartId = part.meshPartId;
               ModelMeshPart modelMeshPart = LibGDXTools.findModelMeshPart(modelData, meshPartId);
               ModelMesh modelMesh = LibGDXTools.findMeshContainingPart(modelData, meshPartId);

               // Each vertex is usually something like 8 floats: x,y,z,nx,ny,nz,u,v
               int floatsPerVertex = LibGDXTools.calculateFloatsPerVertex(modelMesh);
               int numberOfVertices = modelMeshPart.indices.length;
               totalNumberOfVertices += numberOfVertices;

               RigidBodyTransform transform = new RigidBodyTransform();
               Quaternion quaternion = new Quaternion();
               if (node.translation != null)
                  LibGDXTools.toEuclid(node.translation, transform.getTranslation());
               if (node.rotation != null)
                  LibGDXTools.toEuclid(node.rotation, quaternion);
               transform.getRotation().set(quaternion);

               ArrayList<OriginalVertexRecord> originalPartVertices = new ArrayList<>();

               for (short index : modelMeshPart.indices)
               {
                  Point3D32 originalVertex = new Point3D32(modelMesh.vertices[floatsPerVertex * index],
                                                           modelMesh.vertices[floatsPerVertex * index + 1],
                                                           modelMesh.vertices[floatsPerVertex * index + 2]);
                  transform.transform(originalVertex);

                  originalPartVertices.add(new OriginalVertexRecord(originalVertex, index));
                  wholeModelCentroid.add(originalVertex);
               }

               partRecords.add(new PartRecord(modelMeshPart, modelMesh, transform, floatsPerVertex, numberOfVertices, originalPartVertices));
            }
         }
      }
      if (stopwatch.totalElapsed() > 0.5)
         LogTools.warn("Took {} s to initialize, which is a little long.", stopwatch.lapElapsed());

      wholeModelCentroid.scale(1.0 / totalNumberOfVertices);
      centroidFrame.update(transformToParent -> transformToParent.getTranslation().set(wholeModelCentroid));
   }

   public void scale(double scaleFactor)
   {
      Model model = scaleForModel(scaleFactor);
      modelInstance = new ModelInstance(model);
   }

   public Model scaleForModel(double scaleFactor)
   {
      stopwatch.start();
      float scaleFactorFloat = (float) scaleFactor;

      for (PartRecord partRecord : partRecords)
      {
         for (int i = 0; i < partRecord.numberOfVertices(); i++)
         {
            OriginalVertexRecord originalVertexRecord = partRecord.originalVertices().get(i);

            centroidToVertex.set(originalVertexRecord.originalVertex());
            centroidToVertex.sub(wholeModelCentroid);
            centroidToVertex.scale(scaleFactorFloat);

            scaledVertex.setIncludingFrame(centroidFrame.getReferenceFrame(), centroidToVertex);
            scaledVertex.changeFrame(ReferenceFrame.getWorldFrame());

            partRecord.transform().inverseTransform(scaledVertex);

            LibGDXTools.setFloatVertexPosition(partRecord.modelMesh().vertices, partRecord.floatsPerVertex(), originalVertexRecord.index(), scaledVertex);
         }
      }

      Model model = new Model(modelData);

      if (stopwatch.totalElapsed() > 0.1)
         LogTools.warn("Took {} s to scale, which is a little long.", stopwatch.lapElapsed());

      return model;
   }

   public Point3D32 getWholeModelCentroid()
   {
      return wholeModelCentroid;
   }

   public Matrix4 getPoseTransform()
   {
      return modelInstance.transform;
   }

   public ModelInstance getModelInstance()
   {
      return modelInstance;
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      modelInstance.getRenderables(renderables, pool);
   }
}
