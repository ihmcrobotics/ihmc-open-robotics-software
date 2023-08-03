package us.ihmc.tools.property;

import org.junit.jupiter.api.Test;
import us.ihmc.log.LogTools;

import static org.junit.jupiter.api.Assertions.*;

public class StoredPropertySetTest
{
   @Test
   public void testLoading()
   {
      StoredPropertyKeyList keyList = new StoredPropertyKeyList();
      BooleanStoredPropertyKey keyOne = keyList.addBooleanKey("KeyOne");
      DoubleStoredPropertyKey keyTwo = keyList.addDoubleKey("KeyTwo");
      IntegerStoredPropertyKey keyThree = keyList.addIntegerKey("KeyThree");

      StoredPropertySet storedPropertySet = new StoredPropertySet(keyList, StoredPropertySetTest.class);
      storedPropertySet.load();

      assertEquals(false, storedPropertySet.get(keyOne));
      assertEquals(0.1, storedPropertySet.get(keyTwo));
      assertEquals(5, storedPropertySet.get(keyThree));
   }

   @Test
   public void testLoadingDifferentVersion()
   {
      StoredPropertyKeyList keyList = new StoredPropertyKeyList();
      BooleanStoredPropertyKey keyOne = keyList.addBooleanKey("KeyOne");
      DoubleStoredPropertyKey keyTwo = keyList.addDoubleKey("KeyTwo");
      IntegerStoredPropertyKey keyThree = keyList.addIntegerKey("KeyThree");

      StoredPropertySet storedPropertySet = new StoredPropertySet(keyList, StoredPropertySetTest.class);
      storedPropertySet.load();

      assertEquals(false, storedPropertySet.get(keyOne));
      assertEquals(0.1, storedPropertySet.get(keyTwo));
      assertEquals(5, storedPropertySet.get(keyThree));

      storedPropertySet.updateBackingSaveFile("Version2");
      storedPropertySet.load();

      assertEquals(true, storedPropertySet.get(keyOne));
      assertEquals(4.3, storedPropertySet.get(keyTwo));
      assertEquals(1, storedPropertySet.get(keyThree));

      storedPropertySet.set(keyOne, false);
      storedPropertySet.set(keyTwo, 0.4);
      storedPropertySet.set(keyThree, 27);
      storedPropertySet.updateBackingSaveFile("Version3");
      storedPropertySet.save();
      storedPropertySet.load();

      assertEquals(false, storedPropertySet.get(keyOne));
      assertEquals(0.4, storedPropertySet.get(keyTwo));
      assertEquals(27, storedPropertySet.get(keyThree));
   }

   @Test
   public void testDefaults()
   {
      StoredPropertyKeyList keyList = new StoredPropertyKeyList();
      BooleanStoredPropertyKey keyOne = keyList.addBooleanKey("KeyOne", true);
      DoubleStoredPropertyKey keyTwo = keyList.addDoubleKey("KeyTwo", 0.4);
      IntegerStoredPropertyKey keyThree = keyList.addIntegerKey("KeyThree", 8);

      StoredPropertySet storedPropertySet = new StoredPropertySet(keyList, StoredPropertySetTest.class);
      assertEquals(true, storedPropertySet.get(keyOne));
      assertEquals(0.4, storedPropertySet.get(keyTwo));
      assertEquals(8, storedPropertySet.get(keyThree));

      storedPropertySet.load();

      assertEquals(false, storedPropertySet.get(keyOne));
      assertEquals(0.1, storedPropertySet.get(keyTwo));
      assertEquals(5, storedPropertySet.get(keyThree));
   }

   @Test
   public void testEquals()
   {

      StoredPropertyKeyList keyList = new StoredPropertyKeyList();
      BooleanStoredPropertyKey keyOne = keyList.addBooleanKey("KeyOne", true);
      DoubleStoredPropertyKey keyTwo = keyList.addDoubleKey("KeyTwo", 0.4);
      IntegerStoredPropertyKey keyThree = keyList.addIntegerKey("KeyThree", 8);

      StoredPropertySet storedPropertySet = new StoredPropertySet(keyList, StoredPropertySetTest.class);
      assertEquals(true, storedPropertySet.get(keyOne));
      assertEquals(0.4, storedPropertySet.get(keyTwo));
      assertEquals(8, storedPropertySet.get(keyThree));

      StoredPropertySet storedPropertySet2 = new StoredPropertySet(keyList, StoredPropertySetTest.class);
      assertEquals(true, storedPropertySet2.get(keyOne));
      assertEquals(0.4, storedPropertySet2.get(keyTwo));
      assertEquals(8, storedPropertySet2.get(keyThree));

      assertTrue(storedPropertySet.equals(storedPropertySet2));

      storedPropertySet2.load();

      assertFalse(storedPropertySet.equals(storedPropertySet2));
   }

   @Test
   public void testLoadingStoredPropertySetGeneratorTest()
   {
      StoredPropertySetTestParameters storedPropertySetTestParameters = new StoredPropertySetTestParameters();
      LogTools.info(storedPropertySetTestParameters.getTheFirstBooleanProperty());
      LogTools.info(storedPropertySetTestParameters.getTheFirstDoubleProperty());
      LogTools.info(storedPropertySetTestParameters.getTheFirstIntegerProperty());
   }

   @Test
   public void testToFromString()
   {
      StoredPropertySetTestParameters storedPropertySetTestParameters = new StoredPropertySetTestParameters();
      LogTools.info(storedPropertySetTestParameters.toString());
      boolean originalBoolean = storedPropertySetTestParameters.getTheFirstBooleanProperty();
      LogTools.info(originalBoolean);
      double originalDouble = storedPropertySetTestParameters.getTheFirstDoubleProperty();
      LogTools.info(originalDouble);
      int originalInteger = storedPropertySetTestParameters.getTheFirstIntegerProperty();
      LogTools.info(originalInteger);
      storedPropertySetTestParameters.setFromColonCommaString(storedPropertySetTestParameters.toString());
      LogTools.info(storedPropertySetTestParameters.getTheFirstBooleanProperty());
      LogTools.info(storedPropertySetTestParameters.getTheFirstDoubleProperty());
      LogTools.info(storedPropertySetTestParameters.getTheFirstIntegerProperty());
      assertEquals(originalBoolean, storedPropertySetTestParameters.getTheFirstBooleanProperty());
      assertEquals(originalDouble, storedPropertySetTestParameters.getTheFirstDoubleProperty());
      assertEquals(originalInteger, storedPropertySetTestParameters.getTheFirstIntegerProperty());
   }

   public static void main(String[] args)
   {
      StoredPropertySetTestParameters storedPropertySetTestParameters = new StoredPropertySetTestParameters();
      LogTools.info(storedPropertySetTestParameters.getTheFirstBooleanProperty());
      LogTools.info(storedPropertySetTestParameters.getTheFirstDoubleProperty());
      LogTools.info(storedPropertySetTestParameters.getTheFirstIntegerProperty());
      storedPropertySetTestParameters.load();
      LogTools.info(storedPropertySetTestParameters.getTheFirstBooleanProperty());
      LogTools.info(storedPropertySetTestParameters.getTheFirstDoubleProperty());
      LogTools.info(storedPropertySetTestParameters.getTheFirstIntegerProperty());
      storedPropertySetTestParameters.save();
      storedPropertySetTestParameters.load();
      storedPropertySetTestParameters.save();
      storedPropertySetTestParameters.load();
      storedPropertySetTestParameters.load();
      storedPropertySetTestParameters.save();
      storedPropertySetTestParameters.save();
   }
}
