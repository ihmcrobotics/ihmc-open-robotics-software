package us.ihmc.graphics3DDescription.yoGraphics;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;

import javax.vecmath.Point3f;
import javax.vecmath.TexCoord2f;
import javax.vecmath.Vector3f;

import us.ihmc.graphics3DDescription.Graphics3DObject;
import us.ihmc.graphics3DDescription.MeshDataHolder;
import us.ihmc.graphics3DDescription.appearance.AppearanceDefinition;
import us.ihmc.graphics3DDescription.appearance.YoAppearance;
import us.ihmc.graphics3DDescription.instructions.Graphics3DAddMeshDataInstruction;
import us.ihmc.plotting.artifact.Artifact;
import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
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
   private static final AppearanceDefinition appearance = YoAppearance.Aquamarine();
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final int maxNumberOfVertices;
   private final int maxNumberOfPolygons;

   private final List<YoFramePoint2d> poolOfVertices;
   private final List<IntegerYoVariable> poolOfPolygonSizes;
   private final List<YoFramePoseUsingQuaternions> poolOfPolygonPoses;

   private final Graphics3DObject graphics3dObject;
   private final List<Graphics3DAddMeshDataInstruction> polygonMeshInstructions;

   private final AtomicBoolean planarRegionsChanges = new AtomicBoolean(true);

   public YoGraphicPlanarRegionsList(String name, int maxNumberOfVertices, int maxNumberOfPolygons, YoVariableRegistry registry)
   {
      super(name);

      this.maxNumberOfVertices = maxNumberOfVertices;
      this.maxNumberOfPolygons = maxNumberOfPolygons;

      poolOfVertices = new ArrayList<>(maxNumberOfVertices);
      poolOfPolygonSizes = new ArrayList<>(maxNumberOfPolygons);
      poolOfPolygonPoses = new ArrayList<>(maxNumberOfPolygons);

      VariableChangedListener notifyRegionsChanged = new VariableChangedListener()
      {
         @Override
         public void variableChanged(YoVariable<?> v)
         {
            planarRegionsChanges.set(true);
         }
      };

      for (int i = 0; i < maxNumberOfVertices; i++)
      {
         YoFramePoint2d vertex = new YoFramePoint2d(name + "Vertex" + i, worldFrame, registry);
         vertex.setToNaN();
         vertex.attachVariableChangedListener(notifyRegionsChanged);
         poolOfVertices.add(vertex);
      }

      for (int i = 0; i < maxNumberOfPolygons; i++)
      {
         IntegerYoVariable polygonSize = new IntegerYoVariable(name + "Polygon" + i + "Size", registry);
         polygonSize.set(-1);
         polygonSize.addVariableChangedListener(notifyRegionsChanged);
         poolOfPolygonSizes.add(polygonSize);
         YoFramePoseUsingQuaternions polygonPose = new YoFramePoseUsingQuaternions(name + "Polygon" + i + "Pose", worldFrame, registry);
         polygonPose.setToNaN();
         polygonPose.attachVariableChangedListener(notifyRegionsChanged);
         poolOfPolygonPoses.add(polygonPose);
      }

      graphics3dObject = new Graphics3DObject();
      graphics3dObject.setChangeable(true);
      polygonMeshInstructions = new ArrayList<>(maxNumberOfPolygons);

      for (int polygonIndex = 0; polygonIndex < maxNumberOfPolygons; polygonIndex++)
         polygonMeshInstructions.add(graphics3dObject.addMeshData(createPolygonMesh(polygonIndex), appearance));
   }

   YoGraphicPlanarRegionsList(String name, YoVariable<?>[] yoVariables, Double[] constants, AppearanceDefinition appearance)
   {
      super(name);

      maxNumberOfVertices = constants[0].intValue();
      maxNumberOfPolygons = constants[1].intValue();

      poolOfVertices = new ArrayList<>(maxNumberOfVertices);
      poolOfPolygonSizes = new ArrayList<>(maxNumberOfPolygons);
      poolOfPolygonPoses = new ArrayList<>(maxNumberOfPolygons);


      VariableChangedListener notifyRegionsChanged = new VariableChangedListener()
      {
         @Override
         public void variableChanged(YoVariable<?> v)
         {
            planarRegionsChanges.set(true);
         }
      };

      for (YoVariable<?> yoVariable : yoVariables)
      {
         yoVariable.addVariableChangedListener(notifyRegionsChanged);
      }
      
      int variableIndex = 0;

      for (int vertexIndex = 0; vertexIndex < maxNumberOfVertices; vertexIndex++)
      {
         DoubleYoVariable x = (DoubleYoVariable) yoVariables[variableIndex++];
         DoubleYoVariable y = (DoubleYoVariable) yoVariables[variableIndex++];
         YoFramePoint2d vertex = new YoFramePoint2d(x, y, worldFrame);
         poolOfVertices.add(vertex);
      }

      for (int polygonIndex = 0; polygonIndex < maxNumberOfPolygons; polygonIndex++)
      {
         IntegerYoVariable polygonSize = (IntegerYoVariable) yoVariables[variableIndex++];
         poolOfPolygonSizes.add(polygonSize);
      }

      for (int polygonIndex = 0; polygonIndex < maxNumberOfPolygons; polygonIndex++)
      {
         DoubleYoVariable x = (DoubleYoVariable) yoVariables[variableIndex++];
         DoubleYoVariable y = (DoubleYoVariable) yoVariables[variableIndex++];
         DoubleYoVariable z = (DoubleYoVariable) yoVariables[variableIndex++];
         DoubleYoVariable qx = (DoubleYoVariable) yoVariables[variableIndex++];
         DoubleYoVariable qy = (DoubleYoVariable) yoVariables[variableIndex++];
         DoubleYoVariable qz = (DoubleYoVariable) yoVariables[variableIndex++];
         DoubleYoVariable qs = (DoubleYoVariable) yoVariables[variableIndex++];
         YoFramePoint position = new YoFramePoint(x, y, z, worldFrame);
         YoFrameQuaternion orientation = new YoFrameQuaternion(qx, qy, qz, qs, worldFrame);
         YoFramePoseUsingQuaternions polygonPose = new YoFramePoseUsingQuaternions(position, orientation);
         poolOfPolygonPoses.add(polygonPose);
      }

      graphics3dObject = new Graphics3DObject();
      graphics3dObject.setChangeable(true);
      polygonMeshInstructions = new ArrayList<>(maxNumberOfPolygons);

      for (int polygonIndex = 0; polygonIndex < maxNumberOfPolygons; polygonIndex++)
         polygonMeshInstructions.add(graphics3dObject.addMeshData(createPolygonMesh(polygonIndex), appearance));
   }

   private MeshDataHolder createPolygonMesh(int polygonIndex)
   {
      int numberOfVertices = poolOfPolygonSizes.get(polygonIndex).getIntegerValue();

      if (numberOfVertices <= 0)
         return null;

      int firstVertexIndex = findFirstVertexIndexGivenPolygonIndex(polygonIndex);

      int numberOfTriangles = numberOfVertices - 2;

      if(numberOfTriangles <= 0)
         return null;

      YoFramePoseUsingQuaternions pose = poolOfPolygonPoses.get(polygonIndex);

      Point3f[] vertices = new Point3f[numberOfVertices];
      TexCoord2f[] texturePoints = new TexCoord2f[numberOfVertices];
      Vector3f[] vertexNormals = new Vector3f[numberOfVertices];

      RigidBodyTransform transform = new RigidBodyTransform();
      pose.getPose(transform);

      for (int vertexIndex = 0; vertexIndex < numberOfVertices; vertexIndex++)
      {
         Point3f vertex = new Point3f();
         poolOfVertices.get(vertexIndex + firstVertexIndex).get(vertex);
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

   private int findFirstVertexIndexGivenPolygonIndex(int polygonIndex)
   {
      int vertexIndex = 0;
      for (int i = 0; i < polygonIndex; i++)
         vertexIndex += poolOfPolygonSizes.get(i).getIntegerValue();
      return vertexIndex;
   }

   private final RigidBodyTransform regionTransform = new RigidBodyTransform();

   public void update(PlanarRegionsList planarRegionsList)
   {
      int numberOfRegions = planarRegionsList.getNumberOfPlanarRegions();
      int numberOfPolygons = 0;
      int numberOfVertices = 0;

      boolean bufferFull = false;

      for (int regionIndex = 0; regionIndex < numberOfRegions; regionIndex++)
      {
         PlanarRegion region = planarRegionsList.getPlanarRegion(regionIndex);
         region.getTransformToWorld(regionTransform);

         for (int i = 0; i < region.getNumberOfConvexPolygons(); i++)
         {
            ConvexPolygon2d convexPolygon = region.getConvexPolygon(i);

            int polygonSize = convexPolygon.getNumberOfVertices();
            if (numberOfVertices + polygonSize > maxNumberOfVertices)
            {
               bufferFull = true;
               break;
            }

            poolOfPolygonSizes.get(numberOfPolygons).set(polygonSize);
            poolOfPolygonPoses.get(numberOfPolygons).setPose(regionTransform);

            for (int vertexIndex = 0; vertexIndex < polygonSize; vertexIndex++)
            {
               poolOfVertices.get(vertexIndex + numberOfVertices).set(convexPolygon.getVertex(vertexIndex));
            }

            numberOfVertices += polygonSize;
            numberOfPolygons ++;

            if (numberOfPolygons == maxNumberOfPolygons)
            {
               bufferFull = true;
               break;
            }
         }

         if (bufferFull)
            break;
      }

      for (int vertexIndex = numberOfVertices; vertexIndex < maxNumberOfVertices; vertexIndex++)
      {
         poolOfVertices.get(vertexIndex).setToNaN();
      }

      for (int polygonIndex = numberOfPolygons; polygonIndex < maxNumberOfPolygons; polygonIndex++)
      {
         poolOfPolygonSizes.get(polygonIndex).set(-1);
         poolOfPolygonPoses.get(polygonIndex).setToNaN();
      }

      update();
   }

   @Override
   public void update()
   {
      if (!planarRegionsChanges.getAndSet(false))
         return;
      for (int polygonIndex = 0; polygonIndex < maxNumberOfPolygons; polygonIndex++)
      {
         MeshDataHolder polygonMesh = createPolygonMesh(polygonIndex);
         polygonMeshInstructions.get(polygonIndex).setMesh(polygonMesh);
      }
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

      for (int i = 0; i < maxNumberOfVertices; i++)
      {
         allVariables.add(poolOfVertices.get(i).getYoX());
         allVariables.add(poolOfVertices.get(i).getYoY());
      }

      allVariables.addAll(poolOfPolygonSizes);

      for (int i = 0; i < maxNumberOfPolygons; i++)
      {
         allVariables.add(poolOfPolygonPoses.get(i).getYoX());
         allVariables.add(poolOfPolygonPoses.get(i).getYoY());
         allVariables.add(poolOfPolygonPoses.get(i).getYoZ());
         allVariables.add(poolOfPolygonPoses.get(i).getYoQx());
         allVariables.add(poolOfPolygonPoses.get(i).getYoQy());
         allVariables.add(poolOfPolygonPoses.get(i).getYoQz());
         allVariables.add(poolOfPolygonPoses.get(i).getYoQs());
      }
      return allVariables.toArray(new YoVariable[0]);
   }

   @Override
   public double[] getConstants()
   {
      return new double[]{maxNumberOfVertices, maxNumberOfPolygons};
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
