package us.ihmc.commonWalkingControlModules.visualizer;

import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class BasisVectorVisualizer
{
   private static final double BASIS_VECTOR_SCALE = 0.05;
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final Map<Integer, YoFrameVector> yoBasisVectors = new LinkedHashMap<>();
   private final Map<Integer, YoFramePoint> pointOfBases = new LinkedHashMap<>();
   private final Map<Integer, YoGraphicVector> basisVisualizers = new LinkedHashMap<>();

   private final int rhoSize;

   public BasisVectorVisualizer(String name, int rhoSize, double vizScaling, YoGraphicsListRegistry yoGraphicsListRegistry, YoVariableRegistry parentRegistry)
   {
      AppearanceDefinition basisAppearance = YoAppearance.Aqua();

      this.rhoSize = rhoSize;

      YoGraphicsList yoGraphicsList = new YoGraphicsList(name);

      for (int i = 0; i < rhoSize; i++)
      {
         String prefix = name + i;

         YoFrameVector basisVector = new YoFrameVector(prefix + "BasisVector", ReferenceFrame.getWorldFrame(), registry);
         yoBasisVectors.put(i, basisVector);

         YoFramePoint pointOfBasis = new YoFramePoint(prefix + "PointOfBasis", ReferenceFrame.getWorldFrame(), registry);
         pointOfBases.put(i, pointOfBasis);

         YoGraphicVector basisVisualizer = new YoGraphicVector(prefix + "BasisViz", pointOfBasis, basisVector, vizScaling, basisAppearance, true);
         basisVisualizers.put(i, basisVisualizer);

         yoGraphicsListRegistry.registerArtifact(name, basisVisualizer.createArtifact());
         yoGraphicsList.add(basisVisualizer);
      }

      yoGraphicsListRegistry.registerYoGraphicsList(yoGraphicsList);

      parentRegistry.addChild(registry);
   }

   public void visualize(List<FrameVector3D> basisVectors, List<FramePoint3D> contactPoints)
   {
      for (int i = 0; i < rhoSize; i++)
      {
         YoFrameVector yoBasisVector = yoBasisVectors.get(i);
         yoBasisVector.setAndMatchFrame(basisVectors.get(i));
         yoBasisVector.scale(BASIS_VECTOR_SCALE);

         YoFramePoint pointOfBasis = pointOfBases.get(i);
         pointOfBasis.setAndMatchFrame(contactPoints.get(i));
      }
   }
}
