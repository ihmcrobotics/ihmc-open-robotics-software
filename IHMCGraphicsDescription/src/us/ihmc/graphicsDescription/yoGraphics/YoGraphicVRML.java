package us.ihmc.graphicsDescription.yoGraphics;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class YoGraphicVRML extends YoGraphicCoordinateSystem
{
   private final ReferenceFrame referenceFrame;
   private final String modelFilePath;
   private final Vector3d graphicOffset;
   private final Matrix3d graphicRotation;
   private final boolean showCoordinateSystem;
   private final AppearanceDefinition appearance;

   public YoGraphicVRML(String name, ReferenceFrame referenceFrame, YoVariableRegistry registry, String modelFilePath, boolean showCoordinateSystem)
   {
      this(name, referenceFrame, registry, modelFilePath, new Vector3d(), new Matrix3d(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0), showCoordinateSystem);
   }

   public YoGraphicVRML(String name, ReferenceFrame referenceFrame, YoVariableRegistry registry, String modelFilePath, Vector3d graphicOffset,
         Matrix3d graphicRotation, boolean showCoordinateSystem)
   {
      this(name, referenceFrame, registry, modelFilePath, graphicOffset, graphicRotation, null, showCoordinateSystem, 0.1);
   }

   public YoGraphicVRML(String name, ReferenceFrame referenceFrame, YoVariableRegistry registry, String modelFilePath, Vector3d graphicOffset,
         Matrix3d graphicRotation, AppearanceDefinition appearance, boolean showCoordinateSystem, double coordinateAxisLength)
   {
      super(name, "", registry, coordinateAxisLength);
      this.referenceFrame = referenceFrame;
      this.modelFilePath = modelFilePath;
      this.graphicOffset = graphicOffset;
      this.graphicRotation = graphicRotation;
      this.appearance = appearance;
      this.showCoordinateSystem = showCoordinateSystem;
   }

   public void update()
   {
      this.setToReferenceFrame(referenceFrame);
   }

   public Graphics3DObject getLinkGraphics()
   {
      Graphics3DObject linkGraphics = new Graphics3DObject();

      if (showCoordinateSystem)
      {
         linkGraphics.addCoordinateSystem(scale, arrowColor);
      }

      linkGraphics.rotate(graphicRotation);
      linkGraphics.translate(graphicOffset);

      linkGraphics.addModelFile(modelFilePath, appearance);

      return linkGraphics;
   }
}