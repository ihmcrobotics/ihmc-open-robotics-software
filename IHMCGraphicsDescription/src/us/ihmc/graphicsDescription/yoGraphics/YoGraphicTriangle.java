package us.ihmc.graphicsDescription.yoGraphics;

import java.util.concurrent.atomic.AtomicBoolean;

import javax.vecmath.Point3d;

import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.MeshDataGenerator;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.instructions.Graphics3DAddMeshDataInstruction;
import us.ihmc.graphicsDescription.plotting.artifact.Artifact;
import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.Transform3d;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.tools.gui.GraphicsUpdatable;

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

      instruction = graphics3dObject.addPolygon(appearance, pointOne.getPoint3dCopy(), pointTwo.getPoint3dCopy(), pointThree.getPoint3dCopy());

      VariableChangedListener listener = new VariableChangedListener()
      {
         @Override
         public void variableChanged(YoVariable<?> v)
         {
            hasChanged.set(true);
         }
      };

      pointOne.attachVariableChangedListener(listener);
      pointTwo.attachVariableChangedListener(listener);
      pointThree.attachVariableChangedListener(listener);
   }

   public YoGraphicTriangle(String name, DoubleYoVariable pointOneX, DoubleYoVariable pointOneY, DoubleYoVariable pointOneZ, DoubleYoVariable pointTwoX, DoubleYoVariable pointTwoY, DoubleYoVariable pointTwoZ,
         DoubleYoVariable pointThreeX, DoubleYoVariable pointThreeY, DoubleYoVariable pointThreeZ, AppearanceDefinition appearance)
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
            instruction.setMesh(MeshDataGenerator.Polygon(new Point3d[] { pointOne.getPoint3dCopy(), pointTwo.getPoint3dCopy(), pointThree.getPoint3dCopy() }));
         }
         else
         {
            instruction.setMesh(null);
         }
      }
   }

   public void updatePointOne(FramePoint framePointOne)
   {
      pointOne.set(framePointOne);
      update();
   }

   public void updatePointTwo(FramePoint framePointTwo)
   {
      pointTwo.set(framePointTwo);
      update();
   }

   public void updatePointThree(FramePoint framePointThree)
   {
      pointThree.set(framePointThree);
      update();
   }

   public void updatePoints(Point3d pointOne, Point3d pointTwo, Point3d pointThree)
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
   protected void computeRotationTranslation(Transform3d transform3d)
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
