package us.ihmc.simulationconstructionset.util.graphics;

import java.awt.event.ItemEvent;
import java.awt.event.ItemListener;
import java.util.ArrayList;

import javax.swing.JCheckBoxMenuItem;

import us.ihmc.graphicsDescription.yoGraphics.YoGraphic;

public class DynamicGraphicCheckBoxMenuItem extends JCheckBoxMenuItem implements ItemListener
{
   private static final long serialVersionUID = -1641762511153430886L;
   private ArrayList<YoGraphic> yoGraphics = new ArrayList<YoGraphic>();

   public DynamicGraphicCheckBoxMenuItem(String label, ArrayList<YoGraphic> yoGraphics)
   {
      this(label, yoGraphics, true);
   }

   public DynamicGraphicCheckBoxMenuItem(String label, ArrayList<YoGraphic> yoGraphics, boolean selectedState)
   {
      super(label, selectedState);
      this.yoGraphics = yoGraphics;
      this.addItemListener(this);
   }

   public void addDynamicGraphicObjects(ArrayList<YoGraphic> yoGraphics)
   {
      this.yoGraphics.addAll(yoGraphics);
   }

   @Override
   public void itemStateChanged(ItemEvent ie)
   {
      if (ie.getStateChange() == ItemEvent.SELECTED)
      {
         showDynamicGraphicObjects();
      }
      else
      {
         hideDynamicGraphicObjects();
      }
   }

   public void showDynamicGraphicObjects()
   {
      this.setSelected(true);

      for (int i = 0; i < yoGraphics.size(); i++)
      {
         YoGraphic yoGraphic = yoGraphics.get(i);
         yoGraphic.showGraphicObject();
      }
   }

   public void hideDynamicGraphicObjects()
   {
      this.setSelected(false);

      for (int i = 0; i < yoGraphics.size(); i++)
      {
         YoGraphic yoGraphic = yoGraphics.get(i);
         yoGraphic.hideGraphicObject();
      }
   }
}
