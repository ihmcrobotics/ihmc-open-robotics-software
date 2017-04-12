package us.ihmc.graphicsDescription.yoGraphics;

import java.awt.Color;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Deque;
import java.util.List;
import java.util.Random;

import gnu.trove.map.TIntObjectMap;
import gnu.trove.map.hash.TIntObjectHashMap;
import us.ihmc.euclid.transform.AffineTransform;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.Vector3D32;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.MeshDataHolder;
import us.ihmc.graphicsDescription.ModifiableMeshDataHolder;
import us.ihmc.graphicsDescription.TexCoord2f;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.instructions.Graphics3DAddMeshDataInstruction;
import us.ihmc.graphicsDescription.plotting.artifact.Artifact;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.math.frames.YoFramePoseUsingQuaternions;
import us.ihmc.robotics.math.frames.YoFrameQuaternion;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.tools.gui.GraphicsUpdatable;

/**
 * {@link YoGraphic} that can display {@link PlanarRegionsList} locally, with SCS for instance, and remotely, with SCSVisualizer for instance.
 * The implementation is complex because the overall number of vertices of a {@link PlanarRegionsList} tends to get really big overtime.
 * Instead of creating a gazillion of {@link YoVariable}s to store all vertices of any given {@link PlanarRegionsList},
 * a small buffer of vertices is used to update only a portion of the given {@link PlanarRegionsList}.
 *
 * <p>
 * In addition to the complexity due to the constantly growing number of vertices, some synchronization has to be done when displaying this {@link YoGraphic} remotely.
 * When the display thread is rendering at a slower pace than the {@link YoVariable}s are being updated, without any synchronization, only portions
 * of the meshes will be rendered.
 *  
 * <p>
 * Usage of this {@code YoGraphic}:
 * <li> {@link #submitPlanarRegionsListToRender(PlanarRegionsList)} informs that a new {@link PlanarRegionsList} is to be processed to get eventually rendered.
 * The method does not change {@link YoVariable}s and does not compute any mesh. Up to {@value MAX_PLANAR_REGIONS_LIST_DEQUE_SIZE} {@link PlanarRegionsList} can be queued.
 * Beyond that limit, new {@link PlanarRegionsList} will simply be ignored.
 * <li> Once a {@link PlanarRegionsList} is submitted, {@link #processPlanarRegionsListQueue()} will pack a portion the {@link PlanarRegionsList} in this {@link #vertexBuffer}, {@link #currentRegionId}, and {@link #currentRegionPose}.
 * Then {@link #update()} is called.
 * <li> When rendering this {@link YoGraphic} locally, {@link #update()} simply computes the current mesh to be updated, if any.
 * When rendering this {@link YoGraphic} remotely, {@link #update()} will in addition inform the WRITER version of the YoGraphic that it is ready to process a new mesh.
 * This allows to amke sure that remote graphic are properly displayed.
 * 
 * @author Sylvain
 *
 */
public class YoGraphicPlanarRegionsList extends YoGraphic implements RemoteYoGraphic, GraphicsUpdatable
{
   private static final int MAX_PLANAR_REGIONS_LIST_DEQUE_SIZE = 2;

   private final YoGraphicJob yoGraphicJob;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final Random random = new Random(165165L);
   /** Region Id based appearances. They are recycled to ensure continuity in coloring the regions over time. */
   private final TIntObjectMap<AppearanceDefinition> regionIdToAppearance = new TIntObjectHashMap<>();

   private final int vertexBufferSize;
   private final int meshBufferSize;

   /** When this is created as a {@link RemoteYoGraphic}, it is consider as a READER and thus turns on this flag to let the WRITER know that it has to synchronize.*/
   private final BooleanYoVariable waitForReader;
   /** When this is created as a {@link RemoteYoGraphic}, it is consider as a READER and thus turns on this flag to let the WRITER know that it has processed the current mesh. */
   private final BooleanYoVariable hasReaderProcessedMesh;

   /**
    * Buffer of vertices used to store one or more polygons to render.
    * When more than one polygon is stored, each pair of polygons is separated by a vertex that is set to {@link Double#NaN}.
    * All the polygons stored in this buffer are always part of the same {@link PlanarRegion} such that they have the same transform to world.
    * <p>
    * Any non used vertex is set to {@link Double#NaN}.
    */
   private final List<YoFramePoint2d> vertexBuffer;
   /**
    * Corresponds the current mesh index in the {@link #meshBuffer} to be updated.
    * When there is no mesh to render, {@link #currentMeshIndex} equals to {@code -1}.
    */
   private final IntegerYoVariable currentMeshIndex;
   /**
    * Indicates the id of the region that the new mesh belongs to.
    * It is used to find the corresponding {@link AppearanceDefinition}.
    * When there is no mesh to render, {@link #currentRegionId} equals to {@code -1}.
    */
   private final IntegerYoVariable currentRegionId;
   /**
    * Indicates that the current mesh to render is the last one of the current {@link PlanarRegionsList}.
    * It is used to clear the unused part of the {@link #meshBuffer}.
    */
   private final BooleanYoVariable isPlanarRegionsListComplete;
   /**
    * Indicates that this {@link YoGraphic} is clearing the graphics and resetting its current state.
    */
   private final BooleanYoVariable clear;
   /**
    * Indicates the pose with respect to world of the region that the new mesh belongs to.
    * It is used to transform the vertices so they're expressed in world.
    * When there is no mesh to render, {@link #currentRegionId} equals to {@code -1}.
    */
   private final YoFramePoseUsingQuaternions currentRegionPose;
   /**
    * Stores custom colors for planar regions.
    * When constructed without using custom colors, {@link #planarRegionsColorBuffer} is {@code null}
    * and random colors are used.
    */   
   private final IntegerYoVariable[] planarRegionsColorBuffer;

   private final Graphics3DObject graphics3dObject;
   private final List<Graphics3DAddMeshDataInstruction> meshBuffer;

   /**
    * Create a {@link YoGraphic} for rendering {@link PlanarRegionsList}s.
    * 
    * @param name
    * @param vertexBufferSize
    * @param meshBufferSize
    * @param registry
    */
   public YoGraphicPlanarRegionsList(String name, int vertexBufferSize, int meshBufferSize, YoVariableRegistry registry)
   {
      this(name, vertexBufferSize, meshBufferSize, false, -1, registry);
   }

   /**
    * Create a {@link YoGraphic} for rendering {@link PlanarRegionsList}s.
    * 
    * @param name
    * @param vertexBufferSize
    * @param meshBufferSize
    * @param registry
    */
   public YoGraphicPlanarRegionsList(String name, int vertexBufferSize, int meshBufferSize, boolean useCustomColors, int numberOfPlanarRegions, YoVariableRegistry registry)
   {
      super(name);

      yoGraphicJob = YoGraphicJob.WRITER;

      this.vertexBufferSize = vertexBufferSize;
      this.meshBufferSize = meshBufferSize;

      waitForReader = new BooleanYoVariable(name + "WaitForReader", registry);
      hasReaderProcessedMesh = new BooleanYoVariable(name + "HasReaderProcessedMesh", registry);

      vertexBuffer = new ArrayList<>(vertexBufferSize);

      for (int i = 0; i < vertexBufferSize; i++)
      {
         YoFramePoint2d vertex = new YoFramePoint2d(name + "Vertex" + i, worldFrame, registry);
         vertex.setToNaN();
         vertexBuffer.add(vertex);
      }

      currentMeshIndex = new IntegerYoVariable(name + "CurrentMeshIndex", registry);
      currentRegionId = new IntegerYoVariable(name + "CurrentRegionId", registry);
      isPlanarRegionsListComplete = new BooleanYoVariable(name + "IsComplete", registry);
      clear = new BooleanYoVariable(name + "Clear", registry);
      currentRegionPose = new YoFramePoseUsingQuaternions(name + "CurrentRegionPose", worldFrame, registry);

      clearYoVariables();

      graphics3dObject = new Graphics3DObject();
      graphics3dObject.setChangeable(true);
      meshBuffer = new ArrayList<>(meshBufferSize);

      for (int polygonIndex = 0; polygonIndex < meshBufferSize; polygonIndex++)
         meshBuffer.add(graphics3dObject.addMeshData(null, YoAppearance.AliceBlue()));
      
      if(useCustomColors)
      {
         planarRegionsColorBuffer = new IntegerYoVariable[numberOfPlanarRegions];
         for(int i = 0; i < numberOfPlanarRegions; i++)
         {
            planarRegionsColorBuffer[i] = new IntegerYoVariable("customColor" + i, registry);
         }
      }
      else
      {
         planarRegionsColorBuffer = null;
      }
   }

   /**
    * Create a YoGraphic for remote visualization.
    * @param name name of this YoGraphic.
    * @param yoVariables the list of YoVariables needed for this YoGraphic expected to be in the same order as packed in {@link #getVariables()}.
    * @param constants the list of constants (variables that will never change) needed for this YoGraphic expected to be in the same order as packed in {@link #getConstants()}.
    * @return a YoGraphic setup for remote visualization.
    */
   static YoGraphicPlanarRegionsList createAsRemoteYoGraphic(String name, YoVariable<?>[] yoVariables, double[] constants)
   {
      return new YoGraphicPlanarRegionsList(name, yoVariables, constants);
   }

   /**
    * This constructor is only for remote visualizer.
    * It is automatically handled and should not be used for creating a new YoGraphic.
    * For the latter, use the other constructor(s).
    */
   private YoGraphicPlanarRegionsList(String name, YoVariable<?>[] yoVariables, double[] constants)
   {
      super(name);

      yoGraphicJob = YoGraphicJob.READER;

      vertexBufferSize = (int)constants[0];
      meshBufferSize = (int)constants[1];

      vertexBuffer = new ArrayList<>(vertexBufferSize);

      int variableIndex = 0;

      waitForReader = (BooleanYoVariable) yoVariables[variableIndex++];
      hasReaderProcessedMesh = (BooleanYoVariable) yoVariables[variableIndex++];

      for (int vertexIndex = 0; vertexIndex < vertexBufferSize; vertexIndex++)
      {
         DoubleYoVariable x = (DoubleYoVariable) yoVariables[variableIndex++];
         DoubleYoVariable y = (DoubleYoVariable) yoVariables[variableIndex++];
         YoFramePoint2d vertex = new YoFramePoint2d(x, y, worldFrame);
         vertexBuffer.add(vertex);
      }

      currentMeshIndex = (IntegerYoVariable) yoVariables[variableIndex++];
      currentRegionId = (IntegerYoVariable) yoVariables[variableIndex++];
      isPlanarRegionsListComplete = (BooleanYoVariable) yoVariables[variableIndex++];
      clear = (BooleanYoVariable) yoVariables[variableIndex++];

      DoubleYoVariable x = (DoubleYoVariable) yoVariables[variableIndex++];
      DoubleYoVariable y = (DoubleYoVariable) yoVariables[variableIndex++];
      DoubleYoVariable z = (DoubleYoVariable) yoVariables[variableIndex++];
      DoubleYoVariable qx = (DoubleYoVariable) yoVariables[variableIndex++];
      DoubleYoVariable qy = (DoubleYoVariable) yoVariables[variableIndex++];
      DoubleYoVariable qz = (DoubleYoVariable) yoVariables[variableIndex++];
      DoubleYoVariable qs = (DoubleYoVariable) yoVariables[variableIndex++];
      YoFramePoint position = new YoFramePoint(x, y, z, worldFrame);
      YoFrameQuaternion orientation = new YoFrameQuaternion(qx, qy, qz, qs, worldFrame);
      currentRegionPose = new YoFramePoseUsingQuaternions(position, orientation);

      graphics3dObject = new Graphics3DObject();
      graphics3dObject.setChangeable(true);
      meshBuffer = new ArrayList<>(meshBufferSize);

      for (int polygonIndex = 0; polygonIndex < meshBufferSize; polygonIndex++)
         meshBuffer.add(graphics3dObject.addMeshData(null, YoAppearance.AliceBlue()));
      
      if(yoVariables.length > variableIndex)
      {
         int numberOfPlanarRegions = yoVariables.length - variableIndex;
         planarRegionsColorBuffer = new IntegerYoVariable[numberOfPlanarRegions];
         
         for(int i = 0; i < numberOfPlanarRegions; i++)
            planarRegionsColorBuffer[i] = (IntegerYoVariable) yoVariables[variableIndex++];    
      }
      else
      {
         planarRegionsColorBuffer = null;
      }
   }

   /**
    * Update the mesh that will display a portion of the {@link PlanarRegionsList} being processed.
    * This method only reads the YoVariables to update the mesh.
    * 
    * When used as a remote YoGraphic, only this method should be called.
    */
   @Override
   public void update()
   {
      switch (yoGraphicJob)
      {
      case READER:
      {
         // Notify the updater that a reader exists and the updater must synchronize.
         waitForReader.set(true);
         hasReaderProcessedMesh.set(true);

         if (clear.getBooleanValue())
         {
            for (int meshIndex = 0; meshIndex < meshBufferSize; meshIndex++)
               meshBuffer.get(meshIndex).setMesh(null);
            clear.set(false);
            return;
         }
         break;
      }
      case WRITER:
      {
         if (clear.getBooleanValue())
         {
            for (int meshIndex = 0; meshIndex < meshBufferSize; meshIndex++)
               meshBuffer.get(meshIndex).setMesh(null);
            if (!waitForReader.getBooleanValue())
               clear.set(false);
            return;
         }
         break;
      }
      default:
         throw new RuntimeException("Unknown job: " + yoGraphicJob);
      }

      if (currentMeshIndex.getIntegerValue() == -1)
         return;
      MeshDataHolder polygonMesh = createCurrentMesh();
      polygonMesh.setName("PlanarRegion");
      AppearanceDefinition appearance = getCurrentAppearance();

      Graphics3DAddMeshDataInstruction instructionToUpdate = meshBuffer.get(currentMeshIndex.getIntegerValue());
      instructionToUpdate.setMesh(polygonMesh);
      instructionToUpdate.setAppearance(appearance);

      if (isPlanarRegionsListComplete.getBooleanValue())
      {
         // Clear the rest of meshes that are not needed.
         for (int meshIndex = currentMeshIndex.getIntegerValue() + 1; meshIndex < meshBufferSize; meshIndex++)
            meshBuffer.get(meshIndex).setMesh(null);
      }
   }

   /**
    * Retrieve or create a random {@link AppearanceDefinition} given the id of the region to which the current mesh belongs to.
    * @return the mesh appearance.
    */
   private AppearanceDefinition getCurrentAppearance()
   {
      AppearanceDefinition appearance;
      if(planarRegionsColorBuffer != null)
      {
         int requestedColorRGB = planarRegionsColorBuffer[currentMeshIndex.getIntegerValue()].getIntegerValue();
         appearance = YoAppearance.RGBColorFromHex(requestedColorRGB);
      }
      else if (regionIdToAppearance.containsKey(currentRegionId.getIntegerValue()))
      {
         appearance = regionIdToAppearance.get(currentRegionId.getIntegerValue());
      }
      else
      {
         appearance = YoAppearance.randomColor(random);
         regionIdToAppearance.put(currentRegionId.getIntegerValue(), appearance);
      }
      return appearance;
   }

   /**
    * @return a new mesh given the current values of the YoVariables. The mesh belongs to a single region but can contain several polygons.
    */
   private MeshDataHolder createCurrentMesh()
   {
      ModifiableMeshDataHolder modifiableMeshDataHolder = new ModifiableMeshDataHolder();

      int firstVertexIndex = 0;
      boolean isDoneExtractingPolygons = false;

      while (!isDoneExtractingPolygons)
      {
         int numberOfVertices = findPolygonSize(firstVertexIndex);
         modifiableMeshDataHolder.add(createPolygonMesh(firstVertexIndex, numberOfVertices), true);

         firstVertexIndex += numberOfVertices + 1;

         if (firstVertexIndex >= vertexBufferSize)
         {
            isDoneExtractingPolygons = true;
         }
         else
         {
            isDoneExtractingPolygons = vertexBuffer.get(firstVertexIndex).containsNaN();
         }
      }

      return modifiableMeshDataHolder.createMeshDataHolder();
   }

   /**
    * Creates the mesh for a polygon which vertices are extracted from the {@link #vertexBuffer}.
    * @param indexInVertexBuffer the index in the {@link #vertexBuffer} of the first vertex of the polygon.
    * @param numberOfVertices teh polygon size.
    * @return the polygon's mesh.
    */
   private MeshDataHolder createPolygonMesh(int indexInVertexBuffer, int numberOfVertices)
   {
      if (numberOfVertices <= 0)
         return null;

      int numberOfTriangles = numberOfVertices - 2;

      if (numberOfTriangles <= 0)
         return null;

      Point3D32[] vertices = new Point3D32[numberOfVertices];
      TexCoord2f[] texturePoints = new TexCoord2f[numberOfVertices];
      Vector3D32[] vertexNormals = new Vector3D32[numberOfVertices];

      RigidBodyTransform transform = new RigidBodyTransform();
      currentRegionPose.getPose(transform);

      for (int vertexIndex = 0; vertexIndex < numberOfVertices; vertexIndex++)
      {
         Point3D32 vertex = new Point3D32();
         vertexBuffer.get(vertexIndex + indexInVertexBuffer).get(vertex);
         transform.transform(vertex);
         vertices[vertexIndex] = vertex;
      }

      for (int vertexIndex = 0; vertexIndex < numberOfVertices; vertexIndex++)
      {
         Vector3D32 normal = new Vector3D32();
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

   /**
    * Given the index of the first vertex of a polygon contained in the {@link #vertexBuffer},
    * this method retrieves its size knowing that polygons are delimited by {@link Double#NaN} in the buffer.
    * @param firstVertexIndex the index of the first vertex of a polygon contained in the buffer.
    * @return the polygon's size.
    */
   private int findPolygonSize(int firstVertexIndex)
   {
      for (int i = firstVertexIndex; i < vertexBuffer.size(); i++)
      {
         if (vertexBuffer.get(i).containsNaN())
            return i - firstVertexIndex;
      }
      return vertexBuffer.size() - firstVertexIndex;
   }

   private final RigidBodyTransform regionTransform = new RigidBodyTransform();
   private final Deque<PlanarRegionsList> planarRegionsListsDeque = new ArrayDeque<>();

   /**
    * Submit a new list of planar regions to eventually render.
    * This method does NOT update any YoVariables and does not update any graphics.
    * Once a list of planar regions is submitted, the method {@link #processPlanarRegionsListQueue()} has to be called every update tick in the caller.
    * @param planarRegionsList the list of planar regions to be eventually rendered.
    * @param requestedAppearance appearance to render the planar regions
    */   
   public void submitPlanarRegionsListToRender(PlanarRegionsList planarRegionsList, Color requestedColor)
   {
      for(int i = 0; i < planarRegionsList.getNumberOfPlanarRegions(); i++)
      {
         int index = i + currentMeshIndex.getIntegerValue();         
         if(index < 0 || index >= planarRegionsColorBuffer.length)
            break;
         planarRegionsColorBuffer[index].set(requestedColor.getRGB());
      }
      
      submitPlanarRegionsListToRender(planarRegionsList);
   }

   /**
    * Submit a new list of planar regions to eventually render.
    * This method does NOT update any YoVariables and does not update any graphics.
    * Once a list of planar regions is submitted, the method {@link #processPlanarRegionsListQueue()} has to be called every update tick in the caller.
    * @param planarRegionsList the list of planar regions to be eventually rendered.
    */
   public void submitPlanarRegionsListToRender(PlanarRegionsList planarRegionsList)
   {
      if (planarRegionsList == null)
      {
         //TODO: Clear the viz when the planarRegionsList is null;
         
         return;
      }

      if (planarRegionsListsDeque.size() > MAX_PLANAR_REGIONS_LIST_DEQUE_SIZE)
         return;

      // This YoGraphic modifies the planarRegionsList.
      // A full depth copy has to be created to prevent changing the argument.
      PlanarRegionsList copy = planarRegionsList.copy();
      PlanarRegionsList filteredPlanarRegionsList = filterEmptyRegionsAndEmptyPolygons(copy);
      if (filteredPlanarRegionsList != null)
         planarRegionsListsDeque.addLast(filteredPlanarRegionsList);
   }

   /**
    * Filter all the polygons and regions such that only polygon and regions with data remain.
    * Simplifies the algorithm for updating the YoVariables in {@link #processPlanarRegionsListQueue()}.
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

   public void clear()
   {
      clear.set(true);
      planarRegionsListsDeque.clear();
      clearYoVariables();
      update();
   }

   /**
    * Processes the queue of lists of planar regions to render and updates the graphics.
    * This method need to called every update tick in the caller.
    */
   public void processPlanarRegionsListQueue()
   {
      if (waitForReader.getBooleanValue() && !hasReaderProcessedMesh.getBooleanValue())
         return;

      if (clear.getBooleanValue())
         return;

      if (planarRegionsListsDeque.isEmpty())
      {
         clearYoVariables();
         return;
      }

      int currentIndex = currentMeshIndex.getIntegerValue();

      if (currentIndex >= meshBufferSize - 1)
      {
         // Won't be able to process any additional polygons.
         // Clear the current list of planar regions to update and reset the index.
         currentIndex = -1;
         planarRegionsListsDeque.removeFirst();
         if (planarRegionsListsDeque.isEmpty())
         {
            clearYoVariables();
            return;
         }
      }

      PlanarRegionsList planarRegionsListToProcess = planarRegionsListsDeque.peekFirst();
      PlanarRegion planarRegionToProcess = null;
      ConvexPolygon2d polygonToProcess = null;

      // Find the next polygon to update
      while (polygonToProcess == null)
      {
         if (planarRegionsListToProcess == null || planarRegionsListToProcess.isEmpty())
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

         if (polygonToProcess.getNumberOfVertices() > vertexBufferSize)
         {
            // The polygon has too many vertices, skip it.
            polygonToProcess = null;
            // Continue to make sure there is still something in the list of planar regions.
            continue;
         }
      }

      currentIndex++;
      currentMeshIndex.set(currentIndex);
      planarRegionToProcess.getTransformToWorld(regionTransform);
      currentRegionPose.setPose(regionTransform);
      currentRegionId.set(planarRegionToProcess.getRegionId());

      boolean isDonePackingPolygons = false;
      int vertexIndexOffset = 0;

      while (!isDonePackingPolygons)
      {
         int numberOfVertices = polygonToProcess.getNumberOfVertices();

         for (int vertexIndex = 0; vertexIndex < numberOfVertices; vertexIndex++)
         {
            vertexBuffer.get(vertexIndex + vertexIndexOffset).set(polygonToProcess.getVertex(vertexIndex));
         }

         vertexIndexOffset += numberOfVertices;

         if (vertexIndexOffset >= vertexBufferSize)
         {
            isDonePackingPolygons = true;
         }
         else
         {
            // Set the vertex to NaN to mark the end of this polygon
            vertexBuffer.get(vertexIndexOffset).setToNaN();
            vertexIndexOffset++;

            polygonToProcess = planarRegionToProcess.getLastConvexPolygon();
            // Check if another polygon can be packed in the poolOfVertices
            if (polygonToProcess == null)
            {
               isDonePackingPolygons = true;
            }
            else if (vertexIndexOffset + polygonToProcess.getNumberOfVertices() > vertexBufferSize)
            {
               isDonePackingPolygons = true;
            }
            else
            {
               polygonToProcess = planarRegionToProcess.pollLastConvexPolygon();
               isDonePackingPolygons = false;
            }
         }
      }

      for (int vertexIndex = vertexIndexOffset; vertexIndex < vertexBufferSize; vertexIndex++)
      {
         vertexBuffer.get(vertexIndex).setToNaN();
      }

      if (planarRegionToProcess.isEmpty())
         planarRegionsListToProcess.pollLastPlanarRegion();
      isPlanarRegionsListComplete.set(planarRegionsListToProcess.isEmpty());

      hasReaderProcessedMesh.set(false);

      // Update the polygon mesh
      update();
   }

   /**
    * @return the number of lists of planar regions to process by this YoGraphic.
    */
   public int getQueueSize()
   {
      return planarRegionsListsDeque.size();
   }

   /**
    * @return true when no list of planar regions has to be processed.
    */
   public boolean isQueueEmpty()
   {
      return planarRegionsListsDeque.isEmpty();
   }

   /**
    * Nothing is to be updated, the YoVariables have to be cleared so calling {@link #update()} does not do anything.
    */
   private void clearYoVariables()
   {
      for (int vertexIndex = 0; vertexIndex < vertexBufferSize; vertexIndex++)
         vertexBuffer.get(vertexIndex).setToNaN();
      currentMeshIndex.set(-1);
      currentRegionId.set(-1);
      currentRegionPose.setToNaN();
   }

   @Override
   public RemoteGraphicType getRemoteGraphicType()
   {
      return RemoteGraphicType.PLANAR_REGIONS_LIST_DGO;
   }

   /**
    * @return The YoVariables needed to create a remote version of this YoGraphic.
    */
   @Override
   public YoVariable<?>[] getVariables()
   {
      List<YoVariable<?>> allVariables = new ArrayList<>();

      allVariables.add(waitForReader);
      allVariables.add(hasReaderProcessedMesh);

      for (int i = 0; i < vertexBufferSize; i++)
      {
         allVariables.add(vertexBuffer.get(i).getYoX());
         allVariables.add(vertexBuffer.get(i).getYoY());
      }

      allVariables.add(currentMeshIndex);
      allVariables.add(currentRegionId);
      allVariables.add(isPlanarRegionsListComplete);
      allVariables.add(clear);

      allVariables.add(currentRegionPose.getYoX());
      allVariables.add(currentRegionPose.getYoY());
      allVariables.add(currentRegionPose.getYoZ());
      allVariables.add(currentRegionPose.getYoQx());
      allVariables.add(currentRegionPose.getYoQy());
      allVariables.add(currentRegionPose.getYoQz());
      allVariables.add(currentRegionPose.getYoQs());

      return allVariables.toArray(new YoVariable[0]);
   }

   /**
    * @return the constants needed to create a remote version of this YoGraphic.
    */
   @Override
   public double[] getConstants()
   {
      return new double[] {vertexBufferSize, meshBufferSize};
   }

   @Override
   public AppearanceDefinition getAppearance()
   {
      // Does not matter as the appearance is generated internally
      return YoAppearance.AliceBlue();
   }

   @Override
   public Graphics3DObject getLinkGraphics()
   {
      return graphics3dObject;
   }

   @Override
   protected void computeRotationTranslation(AffineTransform transform3d)
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
