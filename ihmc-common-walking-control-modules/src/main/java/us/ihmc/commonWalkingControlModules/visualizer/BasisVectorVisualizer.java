package us.ihmc.commonWalkingControlModules.visualizer;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.SCS2YoGraphicHolder;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;

public class BasisVectorVisualizer implements SCS2YoGraphicHolder
{
   private static final double BASIS_VECTOR_SCALE = 0.05;
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final List<YoFrameVector3D> yoBasisVectors = new ArrayList<>();
   private final List<YoFramePoint3D> pointOfBases = new ArrayList<>();

   private final int rhoSize;
   private final double vizScaling;

   public BasisVectorVisualizer(String name, int rhoSize, double vizScaling, YoGraphicsListRegistry yoGraphicsListRegistry, YoRegistry parentRegistry)
   {
      this.vizScaling = vizScaling;
      AppearanceDefinition basisAppearance = YoAppearance.Aqua();

      this.rhoSize = rhoSize;

      YoGraphicsList yoGraphicsList = new YoGraphicsList(name);

      for (int i = 0; i < rhoSize; i++)
      {
         String prefix = name + i;

         YoFrameVector3D basisVector = new YoFrameVector3D(prefix + "BasisVector", ReferenceFrame.getWorldFrame(), registry);
         yoBasisVectors.add(basisVector);

         YoFramePoint3D pointOfBasis = new YoFramePoint3D(prefix + "PointOfBasis", ReferenceFrame.getWorldFrame(), registry);
         pointOfBases.add(pointOfBasis);

         YoGraphicVector basisVisualizer = new YoGraphicVector(prefix + "BasisViz", pointOfBasis, basisVector, vizScaling, basisAppearance, true);
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
         YoFrameVector3D yoBasisVector = yoBasisVectors.get(i);
         yoBasisVector.setMatchingFrame(basisVectors.get(i));
         yoBasisVector.scale(BASIS_VECTOR_SCALE);

         YoFramePoint3D pointOfBasis = pointOfBases.get(i);
         pointOfBasis.setMatchingFrame(contactPoints.get(i));
      }
   }

   @Override
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(getClass().getSimpleName());
      for (int i = 0; i < rhoSize; i++)
      {
         YoFramePoint3D origin = pointOfBases.get(i);
         YoFrameVector3D basisVector = yoBasisVectors.get(i);
         group.addChild(YoGraphicDefinitionFactory.newYoGraphicArrow3D(basisVector.getNamePrefix(), origin, basisVector, vizScaling, ColorDefinitions.Aqua()));
      }
      return group;
   }
}
