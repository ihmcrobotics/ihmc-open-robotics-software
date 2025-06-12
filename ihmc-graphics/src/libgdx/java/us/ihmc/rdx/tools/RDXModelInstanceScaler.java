package us.ihmc.rdx.tools;

import com.badlogic.gdx.graphics.Mesh;
import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.model.MeshPart;
import com.badlogic.gdx.graphics.g3d.model.Node;
import com.badlogic.gdx.graphics.g3d.model.NodePart;
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

import java.nio.FloatBuffer;
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
   private final Model model;
   private final Point3D32 wholeModelCentroid = new Point3D32();

   private record OriginalVertexRecord(Point3D32 originalVertex, int index) { }
   private record PartRecord(MeshPart meshPart,
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
      this(RDXModelLoader.load(modelFileName));
   }

   public RDXModelInstanceScaler(Model model)
   {
      this.model = LibGDXModelCopier.deepCopy(model);

      stopwatch.start();

      int totalNumberOfVertices = 0;

      for (int nodeIndex = 0; nodeIndex < model.nodes.size; nodeIndex++)
      {
         Node node = model.nodes.get(nodeIndex);

         // Store the node transform for scaling calculations
         RigidBodyTransform transform = new RigidBodyTransform();
         Quaternion quaternion = new Quaternion();
         if (node.translation != null)
            LibGDXTools.toEuclid(node.translation, transform.getTranslation());
         if (node.rotation != null)
            LibGDXTools.toEuclid(node.rotation, quaternion);
         transform.getRotation().set(quaternion);

         for (int partIndex = 0; partIndex < node.parts.size; partIndex++)
         {
            NodePart part = node.parts.get(partIndex);

            MeshPart meshPart = part.meshPart;
            Mesh mesh = meshPart.mesh;

            // Iterate over all vertices, creating a Point3D32 for each in the loop
            // Each vertex is usually something like 8 floats: x,y,z,nx,ny,nz,u,v
            int floatsPerVertex = LibGDXTools.calculateFloatsPerVertex(meshPart);

            // Iterate over all vertices, creating a Point3D32 for each in the loop
            ArrayList<OriginalVertexRecord> originalPartVertices = new ArrayList<>();
            for (int vertexIndex = 0; vertexIndex < meshPart.size; vertexIndex++)
            {
               int actualIndex = meshPart.offset + vertexIndex;

               // Extract x, y, z coordinates from the vertex data
               float x = mesh.getVerticesBuffer().get(actualIndex * floatsPerVertex);
               float y = mesh.getVerticesBuffer().get(actualIndex * floatsPerVertex + 1);
               float z = mesh.getVerticesBuffer().get(actualIndex * floatsPerVertex + 2);

               Point3D32 originalVertex = new Point3D32(x, y, z);
               transform.transform(originalVertex);

               // Add to total count and do centroid calculation
               totalNumberOfVertices++;
               wholeModelCentroid.add(originalVertex);
               originalPartVertices.add(new OriginalVertexRecord(originalVertex, vertexIndex));
            }

            partRecords.add(new PartRecord(meshPart, transform, floatsPerVertex, meshPart.size, originalPartVertices));
         }
      }

      if (stopwatch.totalElapsed() > 0.5)
         LogTools.warn("Took {} s to initialize, which is a little long.", stopwatch.lapElapsed());

      wholeModelCentroid.scale(1.0 / totalNumberOfVertices);
      centroidFrame.update(transformToParent -> transformToParent.getTranslation().set(wholeModelCentroid));
   }

   private void scaleInternal(Model model, double scaleFactor)
   {
      stopwatch.start();
      float scaleFactorFloat = (float) scaleFactor;

      for (int j = 0; j < partRecords.size(); j++)
      {
         PartRecord partRecord = partRecords.get(j);
         FloatBuffer verticesBuffer = model.meshParts.get(j).mesh.getVerticesBuffer(true);
         verticesBuffer.clear();

         for (int i = 0; i < partRecord.numberOfVertices(); i++)
         {
            OriginalVertexRecord originalVertexRecord = partRecord.originalVertices().get(i);

            centroidToVertex.set(originalVertexRecord.originalVertex());
            centroidToVertex.sub(wholeModelCentroid);
            centroidToVertex.scale(scaleFactorFloat);

            scaledVertex.setIncludingFrame(centroidFrame.getReferenceFrame(), centroidToVertex);
            scaledVertex.changeFrame(ReferenceFrame.getWorldFrame());

            partRecord.transform().inverseTransform(scaledVertex);

            verticesBuffer.position(i * partRecord.floatsPerVertex);
            verticesBuffer.put(scaledVertex.getX32());
            verticesBuffer.put(scaledVertex.getY32());
            verticesBuffer.put(scaledVertex.getZ32());
         }
      }

      if (stopwatch.totalElapsed() > 0.1)
         LogTools.warn("Took {} s to scale, which is a little long.", stopwatch.lapElapsed());
   }

   public void scale(double scaleFactor)
   {
      scaleInternal(model, scaleFactor);
      modelInstance = new RDXModelInstance(model);
   }

   /**
    * Scales this Model and returns a deep copy.
    */
   public Model scaleForModel(double scaleFactor)
   {
      Model copiedModel = LibGDXModelCopier.deepCopy(model);
      scaleInternal(copiedModel, scaleFactor);
      return copiedModel;
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
