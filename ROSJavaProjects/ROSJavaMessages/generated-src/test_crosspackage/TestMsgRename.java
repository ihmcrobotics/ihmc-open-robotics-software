package test_crosspackage;

public interface TestMsgRename extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "test_crosspackage/TestMsgRename";
  static final java.lang.String _DEFINITION = "string field1\nint32 field2\nTestSubMsgRename field3";
  java.lang.String getField1();
  void setField1(java.lang.String value);
  int getField2();
  void setField2(int value);
  test_crosspackage.TestSubMsgRename getField3();
  void setField3(test_crosspackage.TestSubMsgRename value);
}
