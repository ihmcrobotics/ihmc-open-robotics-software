package us.ihmc.simulationconstructionset.yoUtilities.graphics;

import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.graphics3DAdapter.graphics.MeshDataGenerator;
import us.ihmc.graphics3DAdapter.graphics.appearances.AppearanceDefinition;
import us.ihmc.graphics3DAdapter.graphics.instructions.Graphics3DAddMeshDataInstruction;
import us.ihmc.plotting.Artifact;
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

   public YoGraphicPolygon(String name, YoFrameConvexPolygon2d yoFrameConvexPolygon2d, YoFramePoint framePoint, YoFrameOrientation orientation, double scale, AppearanceDefinition appearance)
   {
      super(name, framePoint, orientation, scale);

      this.yoFrameConvexPolygon2d = yoFrameConvexPolygon2d;
      this.appearance = appearance;

      graphics3dObject = new Graphics3DObject();
      graphics3dObject.setChangeable(true);

      ConvexPolygon2d convexPolygon2d = yoFrameConvexPolygon2d.getConvexPolygon2d();
      instruction = graphics3dObject.addPolygon(convexPolygon2d, appearance);
   }

   public Artifact createArtifact()
   {
      throw new RuntimeException("Implement Me!");
   }

   @Override
   public void update()
   {
      if (yoFrameConvexPolygon2d.getHasChangedAndReset())
      {
         instruction.setMesh(MeshDataGenerator.Polygon(yoFrameConvexPolygon2d.getConvexPolygon2d()));
      }
   }

   public void updateConvexPolygon2d(FrameConvexPolygon2d frameConvexPolygon2d)
   {
      yoFrameConvexPolygon2d.setFrameConvexPolygon2d(frameConvexPolygon2d);
      update();
   }

   public Graphics3DObject getLinkGraphics()
   {
      return graphics3dObject;
   }

   public RemoteGraphicType getRemoteGraphicType()
   {
      return RemoteGraphicType.YO_FRAME_POLYGON_DGO;
   }

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

   public double[] getConstants()
   {
      return new double[] { scale, yoFrameConvexPolygon2d.getYoFramePoints().size() };
   }

   public AppearanceDefinition getAppearance()
   {
      return appearance;
   }
}
