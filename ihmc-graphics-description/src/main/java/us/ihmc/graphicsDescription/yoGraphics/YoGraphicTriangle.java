package us.ihmc.graphicsDescription.yoGraphics;

import java.util.concurrent.atomic.AtomicBoolean;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.AffineTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.GraphicsUpdatable;
import us.ihmc.graphicsDescription.MeshDataGenerator;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.instructions.Graphics3DAddMeshDataInstruction;
import us.ihmc.graphicsDescription.plotting.artifact.Artifact;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVariable;

public class YoGraphicTriangle extends YoGraphic implements RemoteYoGraphic, GraphicsUpdatable
{
   private final YoFramePoint pointOne;
   private final YoFramePoint pointTwo;
   private final YoFramePoint pointThree;

   private final Graphics3DObject graphics3dObject;
   private final Graphics3DAddMeshDataInstruction instruction;

   private final AppearanceDefinition appearance;

   private final AtomicBoolean hasChanged = new AtomicBoolean(false);

   public YoGraphicTriangle(String name, AppearanceDefinition appearance, YoVariableRegistry registry)
   {
      this(name, new YoFramePoint(name + "0", ReferenceFrame.getWorldFrame(), registry), new YoFramePoint(name + "1", ReferenceFrame.getWorldFrame(), registry),
            new YoFramePoint(name + "2", ReferenceFrame.getWorldFrame(), registry), appearance);
   }

   public YoGraphicTriangle(String name, YoFramePoint pointOne, YoFramePoint pointTwo, YoFramePoint pointThree, AppearanceDefinition appearance)
   {
      super(name);

      this.pointOne = pointOne;
      this.pointTwo = pointTwo;
      this.pointThree = pointThree;

      this.appearance = appearance;

      graphics3dObject = new Graphics3DObject();
      graphics3dObject.setChangeable(true);

      instruction = graphics3dObject.addPolygon(appearance, pointOne, pointTwo, pointThree);

      VariableChangedListener listener = new VariableChangedListener()
      {
         @Override
         public void notifyOfVariableChange(YoVariable<?> v)
         {
            hasChanged.set(true);
         }
      };

      pointOne.attachVariableChangedListener(listener);
      pointTwo.attachVariableChangedListener(listener);
      pointThree.attachVariableChangedListener(listener);
   }

   public YoGraphicTriangle(String name, YoDouble pointOneX, YoDouble pointOneY, YoDouble pointOneZ, YoDouble pointTwoX, YoDouble pointTwoY, YoDouble pointTwoZ,
         YoDouble pointThreeX, YoDouble pointThreeY, YoDouble pointThreeZ, AppearanceDefinition appearance)
   {
      this(name, new YoFramePoint(pointOneX, pointOneY, pointOneZ, ReferenceFrame.getWorldFrame()), new YoFramePoint(pointTwoX, pointTwoY, pointTwoZ, ReferenceFrame.getWorldFrame()),
            new YoFramePoint(pointThreeX, pointThreeY, pointThreeZ, ReferenceFrame.getWorldFrame()), appearance);
   }

   public Artifact createArtifact()
   {
      throw new RuntimeException("Implement Me!");
   }

   @Override
   public void update()
   {
      if (hasChanged.getAndSet(false))
      {
         if ((!pointOne.containsNaN()) && (!pointTwo.containsNaN()) && (!pointThree.containsNaN()))
         {
            instruction.setMesh(MeshDataGenerator.Polygon(new Point3DReadOnly[] { pointOne, pointTwo, pointThree }));
         }
         else
         {
            instruction.setMesh(null);
         }
      }
   }

   public void updatePointOne(FramePoint3D framePointOne)
   {
      pointOne.set(framePointOne);
      update();
   }

   public void updatePointTwo(FramePoint3D framePointTwo)
   {
      pointTwo.set(framePointTwo);
      update();
   }

   public void updatePointThree(FramePoint3D framePointThree)
   {
      pointThree.set(framePointThree);
      update();
   }

   public void updatePoints(Point3D pointOne, Point3D pointTwo, Point3D pointThree)
   {
      this.pointOne.set(pointOne);
      this.pointTwo.set(pointTwo);
      this.pointThree.set(pointThree);

      update();
   }

   public Graphics3DObject getLinkGraphics()
   {
      return graphics3dObject;
   }

   public RemoteGraphicType getRemoteGraphicType()
   {
      return RemoteGraphicType.TRIANGLE_DGO;
   }

   public YoVariable<?>[] getVariables()
   {
      YoVariable<?>[] vars = new YoVariable[9];
      int i = 0;
      vars[i++] = pointOne.getYoX();
      vars[i++] = pointOne.getYoY();
      vars[i++] = pointOne.getYoZ();

      vars[i++] = pointTwo.getYoX();
      vars[i++] = pointTwo.getYoY();
      vars[i++] = pointTwo.getYoZ();

      vars[i++] = pointThree.getYoX();
      vars[i++] = pointThree.getYoY();
      vars[i++] = pointThree.getYoZ();

      return vars;
   }

   public double[] getConstants()
   {
      // No constants...
      return new double[] {};
   }

   public AppearanceDefinition getAppearance()
   {
      return appearance;
   }

   @Override
   protected void computeRotationTranslation(AffineTransform transform3d)
   {
      transform3d.setIdentity();
   }

   @Override
   protected boolean containsNaN()
   {
      if (pointOne.containsNaN())
         return true;
      if (pointTwo.containsNaN())
         return true;
      if (pointThree.containsNaN())
         return true;

      return false;
   }

   public void setToNaN()
   {
      pointOne.setToNaN();
      pointTwo.setToNaN();
      pointThree.setToNaN();

      this.update();
   }
}
