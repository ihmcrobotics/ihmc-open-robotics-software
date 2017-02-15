package us.ihmc.graphicsDescription.yoGraphics;

import java.util.ArrayList;

import us.ihmc.robotics.geometry.RigidBodyTransform;


public class YoGraphicsList
{
   private String label;
   private ArrayList<YoGraphic> yoGraphics;

   public YoGraphicsList(String label, ArrayList<? extends YoGraphic> yoGraphics)
   {
      checkLabelNonEmpty(label);
      this.label = label;
      this.yoGraphics = new ArrayList<YoGraphic>(yoGraphics);
   }

   public YoGraphicsList(String label, YoGraphic[] yoGraphicsArray)
   {
      checkLabelNonEmpty(label);
      this.label = label;

      ArrayList<YoGraphic> yoGraphics = new ArrayList<YoGraphic>(yoGraphicsArray.length);

      for (YoGraphic yoGraphic : yoGraphicsArray)
      {
         yoGraphics.add(yoGraphic);
      }

      this.yoGraphics = yoGraphics;
   }
   
   public void setRootTransform(RigidBodyTransform rootTransform)
   {
      for(int i = 0; i < yoGraphics.size(); i++)
      {
         yoGraphics.get(i).setRootTransform(rootTransform);
      }
   }

   public YoGraphicsList(String label, YoGraphic yoGraphic)
   {
      checkLabelNonEmpty(label);
      this.label = label;

      ArrayList<YoGraphic> yoGraphics = new ArrayList<YoGraphic>(1);

      yoGraphics.add(yoGraphic);

      this.yoGraphics = yoGraphics;
   }

   public YoGraphicsList(String label)
   {
      checkLabelNonEmpty(label);
      this.label = label;
      yoGraphics = new ArrayList<YoGraphic>();
   }

   public String getLabel()
   {
      return label;
   }

   public ArrayList<YoGraphic> getYoGraphics()
   {
      return yoGraphics;
   }

   public void add(YoGraphic yoGraphic)
   {
      yoGraphics.add(yoGraphic);
   }

   public void addAll(ArrayList<YoGraphic> yoGraphics)
   {
      this.yoGraphics.addAll(yoGraphics);
   }

   public void hideYoGraphics()
   {
      int numberOfElements = yoGraphics.size();

      for (int i = 0; i < numberOfElements; i++)
      {
         YoGraphic yoGraphic = yoGraphics.get(i);
         yoGraphic.hideGraphicObject();
      }
   }

   public boolean checkAllYoGraphicsAreShowing()
   {
      int numberOfElements = yoGraphics.size();

      for (int i = 0; i < numberOfElements; i++)
      {
         YoGraphic yoGraphic = yoGraphics.get(i);
         if (!yoGraphic.isGraphicObjectShowing())
            return false;
      }

      return true;
   }

   public void setVisible(boolean visible)
   {
      int numberOfElements = yoGraphics.size();

      for (int i = 0; i < numberOfElements; i++)
      {
         YoGraphic yoGraphic = yoGraphics.get(i);
         yoGraphic.setVisible(visible);
      }
   }

   private void checkLabelNonEmpty(String label)
   {
      if (label.isEmpty())
         throw new RuntimeException("Label is empty.");
   }

   @Override
   public String toString()
   {
      String ret = "";

      ret += label + ":";

      int numberOfElements = yoGraphics.size();

      for (int i = 0; i < numberOfElements; i++)
      {
         YoGraphic yoGraphic = yoGraphics.get(i);
         ret += "\n" + yoGraphic.toString();
      }

      return ret;
   }
}
