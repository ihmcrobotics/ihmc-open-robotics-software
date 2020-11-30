package us.ihmc.commonWalkingControlModules.modelPredictiveController;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;

public class ContactPointForceViewer
{
   private final YoFramePoint3D position;
   private final YoFrameVector3D magnitude;

   private final YoFrameVector3D[] basisMagnitudes;

   public ContactPointForceViewer(String prefix, int numberOfBasisVectors, YoRegistry registry, YoGraphicsListRegistry graphicsListRegistry)
   {
      position = new YoFramePoint3D(prefix + "ContactPosition", ReferenceFrame.getWorldFrame(), registry);
      magnitude = new YoFrameVector3D(prefix + "ContactAcceleration", ReferenceFrame.getWorldFrame(), registry);
      basisMagnitudes = new YoFrameVector3D[numberOfBasisVectors];
      for (int i = 0; i < numberOfBasisVectors; i++)
      {
         basisMagnitudes[i] = new YoFrameVector3D(prefix + "ContactBasisMagnitude" + i, ReferenceFrame.getWorldFrame(), registry);

         YoGraphicVector basisVis = new YoGraphicVector(prefix + "BasisForce" + i, position, basisMagnitudes[i], ContactPlaneForceViewer.scale, YoAppearance.Green());
         graphicsListRegistry.registerYoGraphic("ContactForce", basisVis);
      }

      YoGraphicVector vis = new YoGraphicVector(prefix + "ContactForce", position, magnitude, ContactPlaneForceViewer.scale, YoAppearance.Red());
      graphicsListRegistry.registerYoGraphic("ContactForce", vis);
   }

   public void reset()
   {
      position.setToNaN();
      magnitude.setToNaN();
   }

   public void update(FramePoint3DReadOnly position, FrameVector3DReadOnly magnitude, FrameVector3DReadOnly[] basisMagnitudes)
   {
      this.position.setMatchingFrame(position);
      this.magnitude.setMatchingFrame(magnitude);

      int i = 0;
      for (; i < basisMagnitudes.length; i++)
         this.basisMagnitudes[i].setMatchingFrame(basisMagnitudes[i]);
      for (; i < this.basisMagnitudes.length; i++)
         this.basisMagnitudes[i].setToNaN();
   }
}
