package us.ihmc.graphicsDescription.yoGraphics;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class YoGraphicVRML extends YoGraphicCoordinateSystem
{
   private final ReferenceFrame referenceFrame;
   private final String modelFilePath;
   private final Vector3D graphicOffset;
   private final RotationMatrix graphicRotation;
   private final boolean showCoordinateSystem;
   private final AppearanceDefinition appearance;

   public YoGraphicVRML(String name, ReferenceFrame referenceFrame, YoVariableRegistry registry, String modelFilePath, boolean showCoordinateSystem)
   {
      this(name, referenceFrame, registry, modelFilePath, new Vector3D(), new RotationMatrix(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0), showCoordinateSystem);
   }

   public YoGraphicVRML(String name, ReferenceFrame referenceFrame, YoVariableRegistry registry, String modelFilePath, Vector3D graphicOffset,
         RotationMatrix graphicRotation, boolean showCoordinateSystem)
   {
      this(name, referenceFrame, registry, modelFilePath, graphicOffset, graphicRotation, null, showCoordinateSystem, 0.1);
   }

   public YoGraphicVRML(String name, ReferenceFrame referenceFrame, YoVariableRegistry registry, String modelFilePath, Vector3D graphicOffset,
         RotationMatrix graphicRotation, AppearanceDefinition appearance, boolean showCoordinateSystem, double coordinateAxisLength)
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