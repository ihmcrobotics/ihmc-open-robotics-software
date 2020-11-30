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

public class ContactPlaneForceViewer
{
   static final double scale = 0.05;

   private final int numberOfBasesPerContact;
   private final YoFramePoint3D position;
   private final YoFrameVector3D magnitude;

   private final String prefix;
   private final YoRegistry registry;
   private final YoGraphicsListRegistry graphicsListRegistry;

   public ContactPlaneForceViewer(String prefix, int numberOfBasesPerContact, YoRegistry registry, YoGraphicsListRegistry graphicsListRegistry)
   {
      this.prefix = prefix;
      this.numberOfBasesPerContact = numberOfBasesPerContact;
      this.registry = registry;
      this.graphicsListRegistry = graphicsListRegistry;

      position = new YoFramePoint3D(prefix + "TotalContactPosition", ReferenceFrame.getWorldFrame(), registry);
      magnitude = new YoFrameVector3D(prefix + "TotalContactAcceleration", ReferenceFrame.getWorldFrame(), registry);

      YoGraphicVector vis = new YoGraphicVector(prefix + "TotalContactForce", position, magnitude, scale, YoAppearance.Red());
      graphicsListRegistry.registerYoGraphic("ContactForce", vis);
   }

   int pointNumber = 0;
   public ContactPointForceViewer getNextPointForceViewer()
   {
      return new ContactPointForceViewer(prefix + pointNumber++, numberOfBasesPerContact, registry, graphicsListRegistry);
   }

   public void reset()
   {
      position.setToNaN();
      magnitude.setToNaN();
   }

   public void update(FramePoint3DReadOnly position, FrameVector3DReadOnly magnitude)
   {
      this.position.setMatchingFrame(position);
      this.magnitude.setMatchingFrame(magnitude);
   }
}
