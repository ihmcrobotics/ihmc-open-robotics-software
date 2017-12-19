package us.ihmc.graphicsDescription.yoGraphics;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.transform.AffineTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.GraphicsUpdatable;
import us.ihmc.graphicsDescription.MeshDataGenerator;
import us.ihmc.graphicsDescription.MeshDataHolder;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.instructions.Graphics3DAddMeshDataInstruction;
import us.ihmc.graphicsDescription.plotting.artifact.Artifact;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoVariable;

/**
 * When using this class create and attach a {@code PlaybackListener} to update this object,
 * otherwise the graphics will not be in sync
 */
public class YoGraphicPolygon3D extends YoGraphic implements RemoteYoGraphic, GraphicsUpdatable
{
   private static final MeshDataHolder EMPTY_MESH = MeshDataGenerator.Tetrahedron(0.0);

   private final YoInteger numberOfPoints;
   private final YoFramePoint[] ccwOrderedYoFramePoints;
   private final List<Point3D> ccwOrderedPoints;

   private final double height;
   private AppearanceDefinition appearance;

   private final Graphics3DObject graphics3dObject;
   private final Graphics3DAddMeshDataInstruction instruction;

   public YoGraphicPolygon3D(String name, int maxNumberOfPolygonVertices, double height, AppearanceDefinition appearance, YoVariableRegistry registry)
   {
      super(name);

      ccwOrderedYoFramePoints = new YoFramePoint[maxNumberOfPolygonVertices];
      ccwOrderedPoints = new ArrayList<>(maxNumberOfPolygonVertices);

      for (int i = 0; i < maxNumberOfPolygonVertices; i++)
      {
         ccwOrderedYoFramePoints[i] = new YoFramePoint(name + "Point" + i, ReferenceFrame.getWorldFrame(), registry);
         ccwOrderedYoFramePoints[i].setToNaN();
         ccwOrderedPoints.add(new Point3D());
      }
      this.height = height;
      this.appearance = appearance;

      numberOfPoints = new YoInteger(name + "NumberOfPoints", registry);

      graphics3dObject = new Graphics3DObject();
      graphics3dObject.setChangeable(true);
      instruction = new Graphics3DAddMeshDataInstruction(EMPTY_MESH, appearance);
      graphics3dObject.addInstruction(instruction);
   }

   @Override
   public RemoteGraphicType getRemoteGraphicType()
   {
      return RemoteGraphicType.POLYGON_3D;
   }

   @Override
   public YoVariable<?>[] getVariables()
   {
      YoVariable<?>[] yoVariableList = new YoVariable<?>[3 * ccwOrderedYoFramePoints.length + 1];

      for (int i = 0; i < ccwOrderedYoFramePoints.length; i++)
      {
         yoVariableList[i * 3] = ccwOrderedYoFramePoints[i].getYoX();
         yoVariableList[i * 3 + 1] = ccwOrderedYoFramePoints[i].getYoY();
         yoVariableList[i * 3 + 2] = ccwOrderedYoFramePoints[i].getYoZ();
      }
      yoVariableList[ccwOrderedYoFramePoints.length * 3] = numberOfPoints;

      return yoVariableList;
   }

   public void setAppearance(AppearanceDefinition appearance)
   {
      this.appearance = appearance;
   }

   public void set(FramePoint3DReadOnly[] points)
   {
      for (int i = 0; i < points.length; i++)
         points[i].checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());
      set((Point3DReadOnly[]) points);
   }

   public void set(Point3DReadOnly[] points)
   {
      if (points.length > this.ccwOrderedYoFramePoints.length)
         throw new RuntimeException("Cannot plot more vertices than the maximum number");

      numberOfPoints.set(points.length);

      for (int i = 0; i < numberOfPoints.getValue(); i++)
         ccwOrderedYoFramePoints[i].set(points[i]);

      for (int i = numberOfPoints.getValue(); i < ccwOrderedYoFramePoints.length; i++)
         ccwOrderedYoFramePoints[i].setToNaN();
   }

   public void set(List<? extends Point3DReadOnly> points)
   {
      if (points.size() > this.ccwOrderedYoFramePoints.length)
         throw new RuntimeException("Cannot plot more vertices than the maximum number");

      numberOfPoints.set(points.size());

      for (int i = 0; i < numberOfPoints.getValue(); i++)
         ccwOrderedYoFramePoints[i].set(points.get(i));

      for (int i = numberOfPoints.getValue(); i < ccwOrderedYoFramePoints.length; i++)
         ccwOrderedYoFramePoints[i].setToNaN();
   }

   @Override
   protected void computeRotationTranslation(AffineTransform transform3d)
   {
   }

   @Override
   public void update()
   {
      if (numberOfPoints.getIntegerValue() < 3)
      {
         instruction.setMesh(EMPTY_MESH);
         return;
      }

      for (int i = 0; i < numberOfPoints.getIntegerValue(); i++)
         ccwOrderedYoFramePoints[i].get(ccwOrderedPoints.get(i));

      instruction.setMesh(MeshDataGenerator.Polygon(ccwOrderedPoints, numberOfPoints.getIntegerValue()));
      instruction.setAppearance(appearance);
   }

   public void setToNaN()
   {
      for (int i = 0; i < ccwOrderedYoFramePoints.length; i++)
         ccwOrderedYoFramePoints[i].setToNaN();
   }

   @Override
   public double[] getConstants()
   {
      return new double[] {height};
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
   protected boolean containsNaN()
   {
      for (int i = 0; i < numberOfPoints.getIntegerValue(); i++)
      {
         if (ccwOrderedYoFramePoints[i].containsNaN())
            return true;
      }
      return false;
   }

   @Override
   public Artifact createArtifact()
   {
      throw new RuntimeException("Implement Me!");
   }
}