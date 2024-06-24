package us.ihmc.rdx.imgui;

import imgui.type.ImDouble;

public class ImGuiInputDouble extends ImGuiFancyWidget
{
   private final ImDouble imDouble;

   public ImGuiInputDouble(String label, String format)
   {
      this(label, format, 0.0);
   }

   public ImGuiInputDouble(String label, String format, double initialValue)
   {
      super(label, format);
      imDouble = new ImDouble(initialValue);
   }

   public ImGuiInputDouble(String label, String format, ImDouble imDouble)
   {
      super(label, format);
      this.imDouble = imDouble;
   }

   /**
    * Won't show the + and - buttons.
    */
   public void render()
   {
      render(0.0, 0.0);
   }

   /**
    * Shows the + and - buttons.
    * @param step normal step
    * @param stepFast step when holding ctrl key and clicking + and - buttons
    */
   public boolean render(double step, double stepFast)
   {
      beforeWidgetRender();
      boolean valueChanged = ImGuiTools.volatileInputDouble(label, imDouble, step, stepFast, format);
      afterWidgetRender();
      return valueChanged;
   }

   public void setDoubleValue(double value)
   {
      imDouble.set(value);
   }

   public double getDoubleValue()
   {
      return imDouble.get();
   }

   public ImDouble getImDouble()
   {
      return imDouble;
   }
}
