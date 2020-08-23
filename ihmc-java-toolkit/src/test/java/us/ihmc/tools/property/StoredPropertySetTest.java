package us.ihmc.tools.property;

import org.junit.jupiter.api.Test;

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

      StoredPropertySet storedPropertySet = new StoredPropertySet(keyList, StoredPropertySetTest.class, "ihmc-java-toolkit", "src/test/resources");
      storedPropertySet.load();

      assertEquals(false, storedPropertySet.get(keyOne));
      assertEquals(0.1, storedPropertySet.get(keyTwo));
      assertEquals(5, storedPropertySet.get(keyThree));
   }

   @Test
   public void testDefaults()
   {
      StoredPropertyKeyList keyList = new StoredPropertyKeyList();
      BooleanStoredPropertyKey keyOne = keyList.addBooleanKey("KeyOne", true);
      DoubleStoredPropertyKey keyTwo = keyList.addDoubleKey("KeyTwo", 0.4);
      IntegerStoredPropertyKey keyThree = keyList.addIntegerKey("KeyThree", 8);

      StoredPropertySet storedPropertySet = new StoredPropertySet(keyList, StoredPropertySetTest.class, "ihmc-java-toolkit", "src/test/resources");
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

      StoredPropertySet storedPropertySet = new StoredPropertySet(keyList, StoredPropertySetTest.class, "ihmc-java-toolkit", "src/test/resources");
      assertEquals(true, storedPropertySet.get(keyOne));
      assertEquals(0.4, storedPropertySet.get(keyTwo));
      assertEquals(8, storedPropertySet.get(keyThree));

      StoredPropertySet storedPropertySet2 = new StoredPropertySet(keyList, StoredPropertySetTest.class, "ihmc-java-toolkit", "src/test/resources");
      assertEquals(true, storedPropertySet2.get(keyOne));
      assertEquals(0.4, storedPropertySet2.get(keyTwo));
      assertEquals(8, storedPropertySet2.get(keyThree));

      assertTrue(storedPropertySet.equals(storedPropertySet2));

      storedPropertySet2.load();

      assertFalse(storedPropertySet.equals(storedPropertySet2));
   }
}
