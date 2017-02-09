package us.ihmc.graphicsDescription.yoGraphics;

import javax.vecmath.Point2d;

import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.MeshDataGenerator;
import us.ihmc.graphicsDescription.MeshDataHolder;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.instructions.Graphics3DAddMeshDataInstruction;
import us.ihmc.graphicsDescription.plotting.artifact.Artifact;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.math.frames.YoFrameConvexPolygon2d;
import us.ihmc.robotics.math.frames.YoFrameOrientation;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.math.frames.YoFramePose;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.tools.gui.GraphicsUpdatable;


public class YoGraphicPolygon extends YoGraphicAbstractShape implements RemoteYoGraphic, GraphicsUpdatable
{
   private static final double DEFAULT_HEIGHT = 0.01;
   private final double height;

   private YoFrameConvexPolygon2d yoFrameConvexPolygon2d;
   private final Graphics3DObject graphics3dObject;
   private final Graphics3DAddMeshDataInstruction instruction;

   private final AppearanceDefinition appearance;

   public YoGraphicPolygon(String name, YoFrameConvexPolygon2d yoFrameConvexPolygon2d, YoFramePose framePose, double scale, AppearanceDefinition appearance)
   {
      this(name, yoFrameConvexPolygon2d, framePose.getPosition(), framePose.getOrientation(), scale, appearance);
   }

   public YoGraphicPolygon(String name, YoFrameConvexPolygon2d convexPolygon2d, String namePrefix, String nameSuffix, YoVariableRegistry registry, double scale, AppearanceDefinition appearance)
   {
      this(name, convexPolygon2d, new YoFramePoint(namePrefix, nameSuffix, ReferenceFrame.getWorldFrame(), registry), new YoFrameOrientation(namePrefix, nameSuffix, ReferenceFrame.getWorldFrame(), registry), scale, appearance);
   }

   public YoGraphicPolygon(String name, YoFrameConvexPolygon2d convexPolygon2d, YoVariableRegistry registry, double scale, AppearanceDefinition appearance)
   {
      this(name, convexPolygon2d, new YoFramePoint(name + "Position", ReferenceFrame.getWorldFrame(), registry), new YoFrameOrientation(name + "Orientation", ReferenceFrame.getWorldFrame(), registry), scale, appearance);
   }

   public YoGraphicPolygon(String name, YoFramePose framePose, int maxNumberOfVertices, YoVariableRegistry registry, double scale,
                           AppearanceDefinition appearance)
   {
      this(name, new YoFrameConvexPolygon2d(name + "ConvexPolygon2d", ReferenceFrame.getWorldFrame(), maxNumberOfVertices, registry), framePose.getPosition(),
           framePose.getOrientation(), scale, appearance);
   }

   public YoGraphicPolygon(String name, int maxNumberOfVertices, YoVariableRegistry registry, double scale, AppearanceDefinition appearance)
   {
      this(name, new YoFrameConvexPolygon2d(name + "ConvexPolygon2d", ReferenceFrame.getWorldFrame(), maxNumberOfVertices, registry),
           new YoFramePoint(name + "Position", ReferenceFrame.getWorldFrame(), registry),
           new YoFrameOrientation(name + "Orientation", ReferenceFrame.getWorldFrame(), registry), scale, appearance);
   }

   public YoGraphicPolygon(String name, YoFrameConvexPolygon2d yoFrameConvexPolygon2d, YoFramePoint framePoint, YoFrameOrientation orientation, double scale, AppearanceDefinition appearance)
   {
      this(name, yoFrameConvexPolygon2d, framePoint, orientation, scale, DEFAULT_HEIGHT, appearance);
   }

   public YoGraphicPolygon(String name, YoFrameConvexPolygon2d yoFrameConvexPolygon2d, YoFramePoint framePoint, YoFrameOrientation orientation, double scale, double height, AppearanceDefinition appearance)
   {
      super(name, framePoint, orientation, scale);

      if (yoFrameConvexPolygon2d.getNumberOfVertices() <= 0)
         yoFrameConvexPolygon2d.setConvexPolygon2d(new ConvexPolygon2d(new Point2d[] {new Point2d()}));

      this.yoFrameConvexPolygon2d = yoFrameConvexPolygon2d;
      this.appearance = appearance;
      this.height = height;

      graphics3dObject = new Graphics3DObject();
      graphics3dObject.setChangeable(true);

      ConvexPolygon2d convexPolygon2d = yoFrameConvexPolygon2d.getConvexPolygon2d();
      MeshDataHolder meshDataHolder = MeshDataGenerator.ExtrudedPolygon(convexPolygon2d, height);
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
      if (yoFrameConvexPolygon2d.getHasChangedAndReset())
      {
         instruction.setMesh(MeshDataGenerator.ExtrudedPolygon(yoFrameConvexPolygon2d.getConvexPolygon2d(), height));
      }
   }

   public void updateAppearance(AppearanceDefinition appearance)
   {
      instruction.setAppearance(appearance);
   }

   public void updateConvexPolygon2d(FrameConvexPolygon2d frameConvexPolygon2d)
   {
      yoFrameConvexPolygon2d.setFrameConvexPolygon2d(frameConvexPolygon2d);
      update();
   }

   public void updateConvexPolygon2d(ConvexPolygon2d convexPolygon2d)
   {
      yoFrameConvexPolygon2d.setConvexPolygon2d(convexPolygon2d);
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
      vars[i++] = yoFrameConvexPolygon2d.getYoNumberVertices();

      for (YoFramePoint2d p : yoFrameConvexPolygon2d.getYoFramePoints())
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
      return new double[] { scale, yoFrameConvexPolygon2d.getYoFramePoints().size() };
   }

   @Override
   public AppearanceDefinition getAppearance()
   {
      return appearance;
   }
}
