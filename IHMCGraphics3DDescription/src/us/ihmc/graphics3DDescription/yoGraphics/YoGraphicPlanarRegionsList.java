package us.ihmc.graphics3DDescription.yoGraphics;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Deque;
import java.util.List;

import javax.vecmath.Point3f;
import javax.vecmath.TexCoord2f;
import javax.vecmath.Vector3f;

import us.ihmc.graphics3DDescription.Graphics3DObject;
import us.ihmc.graphics3DDescription.MeshDataHolder;
import us.ihmc.graphics3DDescription.appearance.AppearanceDefinition;
import us.ihmc.graphics3DDescription.appearance.YoAppearance;
import us.ihmc.graphics3DDescription.instructions.Graphics3DAddMeshDataInstruction;
import us.ihmc.plotting.artifact.Artifact;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.geometry.Transform3d;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.math.frames.YoFramePoseUsingQuaternions;
import us.ihmc.robotics.math.frames.YoFrameQuaternion;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.tools.gui.GraphicsUpdatable;

public class YoGraphicPlanarRegionsList extends YoGraphic implements RemoteYoGraphic, GraphicsUpdatable
{
   private enum YoGraphicJob
   {
      /** The YoGraphic is the one processing planar regions and updating the YoVariables. */
      UPDATER,
      /** The YoGraphic reads the YoVariable to create the meshes only. */
      READER
   };

   private final YoGraphicJob yoGraphicJob;

   private static final AppearanceDefinition appearance = YoAppearance.Aquamarine();
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final int maxNumberOfVerticesPerPolygon;
   private final int maxNumberOfPolygons;

   private final BooleanYoVariable waitForReader;
   private final BooleanYoVariable hasReaderProcessedMesh;

   private final List<YoFramePoint2d> poolOfVertices;
   private final IntegerYoVariable currentPolygonIndex;
   private final IntegerYoVariable currentPolygonSize;
   private final YoFramePoseUsingQuaternions currentPolygonPose;

   private final Graphics3DObject graphics3dObject;
   private final List<Graphics3DAddMeshDataInstruction> polygonMeshInstructions;

   public YoGraphicPlanarRegionsList(String name, int maxNumberOfVerticesPerPolygon, int maxNumberOfPolygons, YoVariableRegistry registry)
   {
      super(name);

      yoGraphicJob = YoGraphicJob.UPDATER;

      this.maxNumberOfVerticesPerPolygon = maxNumberOfVerticesPerPolygon;
      this.maxNumberOfPolygons = maxNumberOfPolygons;

      waitForReader = new BooleanYoVariable(name + "WaitForReader", registry);
      hasReaderProcessedMesh = new BooleanYoVariable(name + "HasReaderProcessedMesh", registry);

      poolOfVertices = new ArrayList<>(maxNumberOfVerticesPerPolygon);

      for (int i = 0; i < maxNumberOfVerticesPerPolygon; i++)
      {
         YoFramePoint2d vertex = new YoFramePoint2d(name + "Vertex" + i, worldFrame, registry);
         vertex.setToNaN();
         poolOfVertices.add(vertex);
      }

      currentPolygonIndex = new IntegerYoVariable(name + "CurrentPolygonIndex", registry);
      currentPolygonSize = new IntegerYoVariable(name + "CurrentPolygonSize", registry);
      currentPolygonPose = new YoFramePoseUsingQuaternions(name + "CurrentPolygonPose", worldFrame, registry);

      clearYoVariables();

      graphics3dObject = new Graphics3DObject();
      graphics3dObject.setChangeable(true);
      polygonMeshInstructions = new ArrayList<>(maxNumberOfPolygons);

      for (int polygonIndex = 0; polygonIndex < maxNumberOfPolygons; polygonIndex++)
         polygonMeshInstructions.add(graphics3dObject.addMeshData(null, appearance));
   }

   /**
    * This constructor is only for remote visualizer.
    * It is automatically handled and should not be used for creating a new YoGraphic.
    * For the latter, use the other constructor(s).
    */
   YoGraphicPlanarRegionsList(String name, YoVariable<?>[] yoVariables, Double[] constants, AppearanceDefinition appearance)
   {
      super(name);

      yoGraphicJob = YoGraphicJob.READER;

      maxNumberOfVerticesPerPolygon = constants[0].intValue();
      maxNumberOfPolygons = constants[1].intValue();

      poolOfVertices = new ArrayList<>(maxNumberOfVerticesPerPolygon);

      int variableIndex = 0;

      waitForReader = (BooleanYoVariable) yoVariables[variableIndex++];
      hasReaderProcessedMesh = (BooleanYoVariable) yoVariables[variableIndex++];

      for (int vertexIndex = 0; vertexIndex < maxNumberOfVerticesPerPolygon; vertexIndex++)
      {
         DoubleYoVariable x = (DoubleYoVariable) yoVariables[variableIndex++];
         DoubleYoVariable y = (DoubleYoVariable) yoVariables[variableIndex++];
         YoFramePoint2d vertex = new YoFramePoint2d(x, y, worldFrame);
         poolOfVertices.add(vertex);
      }

      currentPolygonIndex = (IntegerYoVariable) yoVariables[variableIndex++];
      currentPolygonSize = (IntegerYoVariable) yoVariables[variableIndex++];

      DoubleYoVariable x = (DoubleYoVariable) yoVariables[variableIndex++];
      DoubleYoVariable y = (DoubleYoVariable) yoVariables[variableIndex++];
      DoubleYoVariable z = (DoubleYoVariable) yoVariables[variableIndex++];
      DoubleYoVariable qx = (DoubleYoVariable) yoVariables[variableIndex++];
      DoubleYoVariable qy = (DoubleYoVariable) yoVariables[variableIndex++];
      DoubleYoVariable qz = (DoubleYoVariable) yoVariables[variableIndex++];
      DoubleYoVariable qs = (DoubleYoVariable) yoVariables[variableIndex++];
      YoFramePoint position = new YoFramePoint(x, y, z, worldFrame);
      YoFrameQuaternion orientation = new YoFrameQuaternion(qx, qy, qz, qs, worldFrame);
      currentPolygonPose = new YoFramePoseUsingQuaternions(position, orientation);

      graphics3dObject = new Graphics3DObject();
      graphics3dObject.setChangeable(true);
      polygonMeshInstructions = new ArrayList<>(maxNumberOfPolygons);

      for (int polygonIndex = 0; polygonIndex < maxNumberOfPolygons; polygonIndex++)
         polygonMeshInstructions.add(graphics3dObject.addMeshData(null, appearance));
   }

   private MeshDataHolder createCurrentPolygonMesh()
   {
      int numberOfVertices = currentPolygonSize.getIntegerValue();

      if (numberOfVertices <= 0)
         return null;

      int numberOfTriangles = numberOfVertices - 2;

      if (numberOfTriangles <= 0)
         return null;

      Point3f[] vertices = new Point3f[numberOfVertices];
      TexCoord2f[] texturePoints = new TexCoord2f[numberOfVertices];
      Vector3f[] vertexNormals = new Vector3f[numberOfVertices];

      RigidBodyTransform transform = new RigidBodyTransform();
      currentPolygonPose.getPose(transform);

      for (int vertexIndex = 0; vertexIndex < numberOfVertices; vertexIndex++)
      {
         Point3f vertex = new Point3f();
         poolOfVertices.get(vertexIndex).get(vertex);
         transform.transform(vertex);
         vertices[vertexIndex] = vertex;
      }

      for (int vertexIndex = 0; vertexIndex < numberOfVertices; vertexIndex++)
      {
         Vector3f normal = new Vector3f();
         normal.setX((float) transform.getM02());
         normal.setY((float) transform.getM12());
         normal.setZ((float) transform.getM22());
         vertexNormals[vertexIndex] = normal;
      }

      // no texture for now
      for (int vertexIndex = 0; vertexIndex < numberOfVertices; vertexIndex++)
         texturePoints[vertexIndex] = new TexCoord2f();

      int[] triangleIndices = new int[3 * numberOfTriangles];
      int index = 0;
      for (int j = 2; j < numberOfVertices; j++)
      {
         triangleIndices[index++] = 0;
         triangleIndices[index++] = j - 1;
         triangleIndices[index++] = j;
      }

      return new MeshDataHolder(vertices, texturePoints, triangleIndices, vertexNormals);
   }

   private final RigidBodyTransform regionTransform = new RigidBodyTransform();
   private final int maxDequeSize = 2;
   private final Deque<PlanarRegionsList> planarRegionsListsDeque = new ArrayDeque<>();

   /**
    * Submit a new list of planar regions to update.
    * This method does NOT update any YoVariables and thus does not update any graphics.
    * Once a list of planar regions is submitted, the method {@link #updateNextPolygon()} has to be called every update tick in the caller.
    * @param filterEmptyRegions(planarRegionsList) the list of planar regions to be eventually rendered.
    */
   public void submitPlanarRegionsListToRender(PlanarRegionsList planarRegionsList)
   {
      if (planarRegionsListsDeque.size() > maxDequeSize)
         return;

      PlanarRegionsList filteredPlanarRegionsList = filterEmptyRegionsAndEmptyPolygons(planarRegionsList);
      if (filteredPlanarRegionsList != null)
         planarRegionsListsDeque.addLast(filteredPlanarRegionsList);
   }

   /**
    * Filter all the polygons and regions such that only polygon and regions with data remain.
    * Simplyfies the algorithm for updating the YoVariables in {@link #updateNextPolygon()}.
    * @param planarRegionsList the list of planar regions with non-empty regions and non-empty polygons.
    * @return
    */
   private PlanarRegionsList filterEmptyRegionsAndEmptyPolygons(PlanarRegionsList planarRegionsList)
   {
      for (int regionIndex = planarRegionsList.getNumberOfPlanarRegions() - 1; regionIndex >= 0; regionIndex--)
      {
         PlanarRegion planarRegion = planarRegionsList.getPlanarRegion(regionIndex);
         for (int polygonIndex = planarRegion.getNumberOfConvexPolygons() - 1; polygonIndex >= 0; polygonIndex--)
         {
            if (planarRegion.getConvexPolygon(polygonIndex).isEmpty())
               planarRegion.pollConvexPolygon(polygonIndex);
         }

         if (planarRegion.isEmpty())
            planarRegionsList.pollPlanarRegion(regionIndex);
      }

      if (planarRegionsList.isEmpty())
         return null;

      return planarRegionsList;
   }

   /**
    * @return the number of lists of planar regions to process.
    */
   public int getDequeSize()
   {
      return planarRegionsListsDeque.size();
   }

   /**
    * @return true is all the lists of planar regions has been processed.
    */
   public boolean isDequeEmpty()
   {
      return planarRegionsListsDeque.isEmpty();
   }

   /**
    * Update one polygon mesh that belong to a planar region.
    * This method only reads the YoVariables to update the mesh.
    */
   @Override
   public void update()
   {
      if (yoGraphicJob == YoGraphicJob.READER)
      {
         waitForReader.set(true);
         hasReaderProcessedMesh.set(true);
      }

      if (currentPolygonIndex.getIntegerValue() == -1)
         return;
      MeshDataHolder polygonMesh = createCurrentPolygonMesh();
      polygonMeshInstructions.get(currentPolygonIndex.getIntegerValue()).setMesh(polygonMesh);
   }

   /**
    * Processes the queue of lists of planar regions to render and updates the graphics.
    * This method need to called every update tick in the caller.
    */
   public void updateNextPolygon()
   {
      if (waitForReader.getBooleanValue() && !hasReaderProcessedMesh.getBooleanValue())
         return;

      if (planarRegionsListsDeque.isEmpty())
      {
         clearYoVariables();
         return;
      }

      int currentIndex = currentPolygonIndex.getIntegerValue();

      if (currentIndex >= maxNumberOfPolygons)
      {
         // Won't be able to process any additional polygons.
         // Clear the current list of planar regions to update and reset the index.
         currentIndex = -1;
         planarRegionsListsDeque.removeFirst();
      }

      PlanarRegionsList planarRegionsListToProcess = planarRegionsListsDeque.peekFirst();
      PlanarRegion planarRegionToProcess = null;
      ConvexPolygon2d polygonToProcess = null;

      // Find the next polygon to update
      while (polygonToProcess == null)
      {
         if (planarRegionsListToProcess.isEmpty())
         {
            // Done processing the list of planar regions.
            // Reset the index as we start over the update.
            currentIndex = -1;
            planarRegionsListsDeque.removeFirst();

            if (planarRegionsListsDeque.isEmpty())
            {
               clearYoVariables();
               return;
            }
            else
            {
               planarRegionsListToProcess = planarRegionsListsDeque.peekFirst();
            }
         }

         planarRegionToProcess = planarRegionsListToProcess.getLastPlanarRegion();

         if (planarRegionToProcess.isEmpty())
         {
            // This region has been fully processed.
            planarRegionsListToProcess.pollLastPlanarRegion();
            // Get rid of the reference to this empty region.
            planarRegionToProcess = null;
            // Continue to make sure there is still something in the list of planar regions.
            continue;
         }

         polygonToProcess = planarRegionToProcess.pollLastConvexPolygon();

         if (polygonToProcess.getNumberOfVertices() > maxNumberOfVerticesPerPolygon)
         {
            // The polygon has too many vertices, skip it.
            polygonToProcess = null;
            // Continue to make sure there is still something in the list of planar regions.
            continue;
         }
      }

      currentIndex++;

      int numberOfVertices = polygonToProcess.getNumberOfVertices();
      for (int vertexIndex = 0; vertexIndex < numberOfVertices; vertexIndex++)
      {
         poolOfVertices.get(vertexIndex).set(polygonToProcess.getVertex(vertexIndex));
      }

      for (int vertexIndex = numberOfVertices; vertexIndex < maxNumberOfVerticesPerPolygon; vertexIndex++)
      {
         poolOfVertices.get(vertexIndex).setToNaN();
      }

      currentPolygonIndex.set(currentIndex);
      currentPolygonSize.set(numberOfVertices);

      planarRegionToProcess.getTransformToWorld(regionTransform);
      currentPolygonPose.setPose(regionTransform);

      hasReaderProcessedMesh.set(false);

      // Update the polygon mesh
      update();
   }

   /**
    * Nothing is to be updated, the YoVariables have to be cleared so calling {@link #update()} does not do anything.
    */
   private void clearYoVariables()
   {
      for (int vertexIndex = 0; vertexIndex < maxNumberOfVerticesPerPolygon; vertexIndex++)
         poolOfVertices.get(vertexIndex).setToNaN();
      currentPolygonIndex.set(-1);
      currentPolygonSize.set(-1);
      currentPolygonPose.setToNaN();
   }

   @Override
   public RemoteGraphicType getRemoteGraphicType()
   {
      return RemoteGraphicType.PLANAR_REGIONS_LIST_DGO;
   }

   @Override
   public YoVariable<?>[] getVariables()
   {
      List<YoVariable<?>> allVariables = new ArrayList<>();

      allVariables.add(waitForReader);
      allVariables.add(hasReaderProcessedMesh);

      for (int i = 0; i < maxNumberOfVerticesPerPolygon; i++)
      {
         allVariables.add(poolOfVertices.get(i).getYoX());
         allVariables.add(poolOfVertices.get(i).getYoY());
      }

      allVariables.add(currentPolygonIndex);
      allVariables.add(currentPolygonSize);

      allVariables.add(currentPolygonPose.getYoX());
      allVariables.add(currentPolygonPose.getYoY());
      allVariables.add(currentPolygonPose.getYoZ());
      allVariables.add(currentPolygonPose.getYoQx());
      allVariables.add(currentPolygonPose.getYoQy());
      allVariables.add(currentPolygonPose.getYoQz());
      allVariables.add(currentPolygonPose.getYoQs());

      return allVariables.toArray(new YoVariable[0]);
   }

   @Override
   public double[] getConstants()
   {
      return new double[] {maxNumberOfVerticesPerPolygon, maxNumberOfPolygons};
   }

   @Override
   public AppearanceDefinition getAppearance()
   {
      return appearance;
   }

   @Override
   public Graphics3DObject getLinkGraphics()
   {
      return graphics3dObject;
   }

   @Override
   protected void computeRotationTranslation(Transform3d transform3d)
   { // Dealing with the transform here.
      transform3d.setIdentity();
   }

   @Override
   protected boolean containsNaN()
   { // Only used to determine if the graphics from this object is valid, and whether to display or hide.
      return false;
   }

   @Override
   public Artifact createArtifact()
   {
      throw new RuntimeException("Implement Me!");
   }
}
