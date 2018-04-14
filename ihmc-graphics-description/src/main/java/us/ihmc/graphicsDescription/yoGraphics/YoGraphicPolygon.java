package us.ihmc.graphicsDescription.yoGraphics;

import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.GraphicsUpdatable;
import us.ihmc.graphicsDescription.MeshDataGenerator;
import us.ihmc.graphicsDescription.MeshDataHolder;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.instructions.Graphics3DAddMeshDataInstruction;
import us.ihmc.graphicsDescription.plotting.artifact.Artifact;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.variable.YoFramePoint2D;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFramePoseUsingYawPitchRoll;
import us.ihmc.yoVariables.variable.YoFrameYawPitchRoll;
import us.ihmc.yoVariables.variable.YoVariable;


public class YoGraphicPolygon extends YoGraphicAbstractShape implements RemoteYoGraphic, GraphicsUpdatable
{
   private static final double DEFAULT_HEIGHT = 0.01;
   private final double height;

   private YoFrameConvexPolygon2D yoFrameConvexPolygon2d;
   private final Graphics3DObject graphics3dObject;
   private final Graphics3DAddMeshDataInstruction instruction;

   private final AppearanceDefinition appearance;

   public YoGraphicPolygon(String name, YoFrameConvexPolygon2D yoFrameConvexPolygon2d, YoFramePoseUsingYawPitchRoll framePose, double scale, AppearanceDefinition appearance)
   {
      this(name, yoFrameConvexPolygon2d, framePose.getPosition(), framePose.getOrientation(), scale, appearance);
   }

   public YoGraphicPolygon(String name, YoFrameConvexPolygon2D convexPolygon2d, String namePrefix, String nameSuffix, YoVariableRegistry registry, double scale, AppearanceDefinition appearance)
   {
      this(name, convexPolygon2d, new YoFramePoint3D(namePrefix, nameSuffix, ReferenceFrame.getWorldFrame(), registry), new YoFrameYawPitchRoll(namePrefix, nameSuffix, ReferenceFrame.getWorldFrame(), registry), scale, appearance);
   }

   public YoGraphicPolygon(String name, YoFrameConvexPolygon2D convexPolygon2d, YoVariableRegistry registry, double scale, AppearanceDefinition appearance)
   {
      this(name, convexPolygon2d, new YoFramePoint3D(name + "Position", ReferenceFrame.getWorldFrame(), registry), new YoFrameYawPitchRoll(name + "Orientation", ReferenceFrame.getWorldFrame(), registry), scale, appearance);
   }

   public YoGraphicPolygon(String name, YoFramePoseUsingYawPitchRoll framePose, int maxNumberOfVertices, YoVariableRegistry registry, double scale,
                           AppearanceDefinition appearance)
   {
      this(name, new YoFrameConvexPolygon2D(name + "ConvexPolygon2d", ReferenceFrame.getWorldFrame(), maxNumberOfVertices, registry), framePose.getPosition(),
           framePose.getOrientation(), scale, appearance);
   }

   public YoGraphicPolygon(String name, int maxNumberOfVertices, YoVariableRegistry registry, double scale, AppearanceDefinition appearance)
   {
      this(name, new YoFrameConvexPolygon2D(name + "ConvexPolygon2d", ReferenceFrame.getWorldFrame(), maxNumberOfVertices, registry),
           new YoFramePoint3D(name + "Position", ReferenceFrame.getWorldFrame(), registry),
           new YoFrameYawPitchRoll(name + "Orientation", ReferenceFrame.getWorldFrame(), registry), scale, appearance);
   }

   public YoGraphicPolygon(String name, YoFrameConvexPolygon2D yoFrameConvexPolygon2d, YoFramePoint3D framePoint, YoFrameYawPitchRoll orientation, double scale, AppearanceDefinition appearance)
   {
      this(name, yoFrameConvexPolygon2d, framePoint, orientation, scale, DEFAULT_HEIGHT, appearance);
   }

   public YoGraphicPolygon(String name, YoFrameConvexPolygon2D yoFrameConvexPolygon2d, YoFramePoint3D framePoint, YoFrameYawPitchRoll orientation, double scale, double height, AppearanceDefinition appearance)
   {
      super(name, framePoint, orientation, scale);

      if (yoFrameConvexPolygon2d.getNumberOfVertices() <= 0)
         yoFrameConvexPolygon2d.set(Vertex2DSupplier.emptyVertex2DSupplier());

      this.yoFrameConvexPolygon2d = yoFrameConvexPolygon2d;
      this.appearance = appearance;
      this.height = height;

      graphics3dObject = new Graphics3DObject();
      graphics3dObject.setChangeable(true);

      MeshDataHolder meshDataHolder = MeshDataGenerator.ExtrudedPolygon(yoFrameConvexPolygon2d, height);
      instruction = new Graphics3DAddMeshDataInstruction(meshDataHolder, appearance);
      graphics3dObject.addInstruction(instruction);
   }

   @Override
   public Artifact createArtifact()
   {
      throw new RuntimeException("Implement Me!");
   }

   @Override
   public void update()
   {
      instruction.setMesh(MeshDataGenerator.ExtrudedPolygon(yoFrameConvexPolygon2d, height));
   }

   public void updateAppearance(AppearanceDefinition appearance)
   {
      instruction.setAppearance(appearance);
   }

   public void updateConvexPolygon2d(FrameConvexPolygon2DReadOnly frameConvexPolygon2d)
   {
      yoFrameConvexPolygon2d.set(frameConvexPolygon2d);
      update();
   }

   public void updateConvexPolygon2d(ConvexPolygon2DReadOnly convexPolygon2d)
   {
      yoFrameConvexPolygon2d.set(convexPolygon2d);
      update();
   }

   @Override
   public Graphics3DObject getLinkGraphics()
   {
      return graphics3dObject;
   }

   @Override
   public RemoteGraphicType getRemoteGraphicType()
   {
      return RemoteGraphicType.YO_FRAME_POLYGON_DGO;
   }

   @Override
   public YoVariable<?>[] getVariables()
   {
      //poly + framePoint + frameOrientation
      YoVariable<?>[] vars = new YoVariable[1 + 2 * yoFrameConvexPolygon2d.getMaxNumberOfVertices() + 6];
      int i = 0;
      vars[i++] = yoFrameConvexPolygon2d.getYoNumberOfVertices();

      for (YoFramePoint2D p : yoFrameConvexPolygon2d.getVertexBuffer())
      {
         vars[i++] = p.getYoX();
         vars[i++] = p.getYoY();
      }

      vars[i++] = yoFramePoint.getYoX();
      vars[i++] = yoFramePoint.getYoY();
      vars[i++] = yoFramePoint.getYoZ();

      vars[i++] = yoFrameOrientation.getYaw();
      vars[i++] = yoFrameOrientation.getPitch();
      vars[i++] = yoFrameOrientation.getRoll();

      return vars;
   }

   @Override
   public double[] getConstants()
   {
      return new double[] { scale, yoFrameConvexPolygon2d.getVertexBuffer().size() };
   }

   @Override
   public AppearanceDefinition getAppearance()
   {
      return appearance;
   }
}
