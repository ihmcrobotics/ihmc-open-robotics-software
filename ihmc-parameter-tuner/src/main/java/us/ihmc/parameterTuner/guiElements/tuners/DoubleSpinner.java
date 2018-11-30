package us.ihmc.parameterTuner.guiElements.tuners;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.apache.commons.lang3.tuple.ImmutablePair;

import javafx.scene.control.MenuItem;
import javafx.scene.control.SeparatorMenuItem;
import javafx.scene.control.TextInputDialog;

public class DoubleSpinner extends NumericSpinner<Double>
{
   public DoubleSpinner()
   {
      super(new DoubleSpinnerValueFactory(0.1));
   }

   @Override
   public Double convertStringToNumber(String numberString)
   {
      if (numberString == null)
      {
         return 0.0;
      }
      if (numberString.endsWith("e") || numberString.endsWith("E"))
      {
         return Double.parseDouble(numberString.substring(0, numberString.length() - 1));
      }
      else if (numberString.endsWith("e-") || numberString.endsWith("E-"))
      {
         return Double.parseDouble(numberString.substring(0, numberString.length() - 2));
      }
      return Double.parseDouble(numberString);
   }

   @Override
   public String convertNumberToString(Double number)
   {
      return Double.toString(number);
   }

   @Override
   public List<MenuItem> getContextMenuOptions()
   {
      List<MenuItem> items = new ArrayList<>();
      for (ImmutablePair<String, String> option : getSpecialStringOptions())
      {
         MenuItem menuItem = new MenuItem(option.getLeft());
         Double number = getValueFactory().getConverter().fromString(option.getRight());
         menuItem.setOnAction(actionEvent -> setValue(number));
         items.add(menuItem);
      }
      items.add(new SeparatorMenuItem());
      items.add(getAngleConversionOption());
      return items;
   }

   private List<ImmutablePair<String, String>> getSpecialStringOptions()
   {
      List<ImmutablePair<String, String>> ret = new ArrayList<>();
      ret.add(new ImmutablePair<String, String>("Infinity", convertNumberToString(Double.POSITIVE_INFINITY)));
      ret.add(new ImmutablePair<String, String>("Negative Infinity", convertNumberToString(Double.NEGATIVE_INFINITY)));
      return ret;
   }

   private MenuItem getAngleConversionOption()
   {
      MenuItem menuItem = new MenuItem("Input Angle");

      menuItem.setOnAction(actionEvent -> {
         TextInputDialog dialog = new TextInputDialog();
         dialog.setTitle("Convert Angle");
         dialog.setHeaderText("Convert Degree to Radian");
         dialog.setContentText("Enter Angle in Degrees:");

         Optional<String> result = dialog.showAndWait();
         if (result.isPresent() && isValidString(result.get()))
         {
            Double number = getValueFactory().getConverter().fromString(result.get());
            setValue(Math.toRadians(number));
         }
      });

      return menuItem;
   }
}
